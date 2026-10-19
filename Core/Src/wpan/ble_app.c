/*
 * Bluetooth Low Energy поверх библиотеки ST (STM32_WPAN).
 *
 * Как запускается радио:
 *   1. TL_Enable() включает CPU2 (радио-ядро). Оно загружается и присылает
 *      системное событие READY — «я готов, во мне работает BLE-стек версии X».
 *   2. Командой SHCI_C2_BLE_Init передаём настройки стека (из app_conf.h).
 *   3. Дальше обычные BLE-команды (HCI/ACI): создать сервисы, включить рекламу.
 *   4. События от стека (подключился телефон, записал данные...) приходят через
 *      прерывание IPCC в очередь, а разбираем их в задаче ble_task.
 *
 * Все BLE-команды вызываются только из ble_task — так нет гонок за радио.
 * Другие задачи кладут текст в tx_sb и «пинают» ble_task уведомлением.
 */
#include "ble_app.h"
#include "app_conf.h"
#include "ble_common.h"
#include "ble_core.h"
#include "tl.h"
#include "shci.h"
#include "shci_tl.h"
#include "hci_tl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <string.h>

void HW_IPCC_Reset(void);
extern volatile int32_t g_temp_x10;

volatile uint32_t g_ble_fw_version;
volatile uint32_t g_ble_error;
static volatile ble_state_t state = BLE_STATE_STARTING;

/* ---- Буферы в общей памяти SRAM2A: их видят оба ядра ---- */
#define POOL_SIZE (CFG_TLBLE_EVT_QUEUE_LENGTH * 4U * DIVC((sizeof(TL_PacketHeader_t) + TL_BLE_EVENT_FRAME_SIZE), 4U))
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t BleSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

/* ---- Задачи и синхронизация ---- */
#define EVT_SYS_READY  (1U << 0)
#define EVT_HCI        (1U << 1)
#define EVT_TX         (1U << 2)

static TaskHandle_t sys_task_h, ble_task_h;
static SemaphoreHandle_t shci_sem, shci_mtx, hci_sem, hci_mtx, tx_mtx;
static StreamBufferHandle_t tx_sb;

/* ---- GATT ---- */
static const char dev_name[] = "WB55-OLED";

/* 128-битные UUID в BLE передаются задом наперёд (младший байт первым).
   Nordic UART Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E */
#define NUS_UUID(b) { 0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,0x93,0xF3,0xA3,0xB5, b,0x00,0x40,0x6E }
static const uint8_t nus_svc_uuid[16] = NUS_UUID(0x01);
static const uint8_t nus_rx_uuid[16]  = NUS_UUID(0x02);   /* телефон -> плата (write) */
static const uint8_t nus_tx_uuid[16]  = NUS_UUID(0x03);   /* плата -> телефон (notify) */

#define ESS_UUID       0x181A   /* Environmental Sensing Service (стандартный) */
#define TEMP_CHAR_UUID 0x2A6E   /* Temperature: int16, единица 0.01 °C */

static uint16_t nus_svc, nus_rx, nus_tx, ess_svc, temp_char;
static uint16_t conn_handle = 0xFFFF;
static uint8_t  tx_notify_on;            /* телефон подписался на уведомления TX */
static uint16_t att_mtu = 23;            /* по умолчанию в BLE пакет данных = MTU-3 = 20 байт */

/* ------------------------------------------------------------------ */
/*      «Клей» между транспортным слоем ST и FreeRTOS                  */
/* ------------------------------------------------------------------ */

/* Эти функции транспортный слой вызывает и из прерываний, и из задач */
static void notify(TaskHandle_t t, uint32_t bits)
{
    if (xPortIsInsideInterrupt()) {
        BaseType_t woken = pdFALSE;
        xTaskNotifyFromISR(t, bits, eSetBits, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xTaskNotify(t, bits, eSetBits);
    }
}

static void give(SemaphoreHandle_t s)
{
    if (xPortIsInsideInterrupt()) {
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xSemaphoreGive(s);
    }
}

/* Системный канал (SHCI) */
void shci_notify_asynch_evt(void *p)    { (void)p; notify(sys_task_h, 1); }
void shci_cmd_resp_release(uint32_t f)  { (void)f; give(shci_sem); }
void shci_cmd_resp_wait(uint32_t t)     { (void)t; xSemaphoreTake(shci_sem, portMAX_DELAY); }

static void shci_status(SHCI_TL_CmdStatus_t s)
{
    if (s == SHCI_TL_CmdBusy) xSemaphoreTake(shci_mtx, portMAX_DELAY);
    else                      xSemaphoreGive(shci_mtx);
}

/* BLE-канал (HCI) */
void hci_notify_asynch_evt(void *p)     { (void)p; notify(ble_task_h, EVT_HCI); }
void hci_cmd_resp_release(uint32_t f)   { (void)f; give(hci_sem); }
void hci_cmd_resp_wait(uint32_t t)      { (void)t; xSemaphoreTake(hci_sem, portMAX_DELAY); }

static void hci_status(HCI_TL_CmdStatus_t s)
{
    if (s == HCI_TL_CmdBusy) xSemaphoreTake(hci_mtx, portMAX_DELAY);
    else                     xSemaphoreGive(hci_mtx);
}

/* ------------------------------------------------------------------ */
/*                    Системные события от CPU2                        */
/* ------------------------------------------------------------------ */

static void sys_event(void *payload)
{
    tSHCI_UserEvtRxParam *p = payload;
    TL_AsynchEvt_t *evt = (TL_AsynchEvt_t *)p->pckt->evtserial.evt.payload;

    if (evt->subevtcode == SHCI_SUB_EVT_CODE_READY) {
        SHCI_C2_Ready_Evt_t *ready = (SHCI_C2_Ready_Evt_t *)evt->payload;
        if (ready->sysevt_ready_rsp == WIRELESS_FW_RUNNING) {
            notify(ble_task_h, EVT_SYS_READY);
        } else {
            /* Во втором ядре запущен не BLE-стек, а сервисная прошивка FUS */
            g_ble_error = 0xF0;
            state = BLE_STATE_ERROR;
            p->status = SHCI_TL_UserEventFlow_Disable;
        }
    } else if (evt->subevtcode == SHCI_SUB_EVT_ERROR_NOTIF) {
        g_ble_error = 0xE0 | (*(uint8_t *)evt->payload & 0x0F);
        state = BLE_STATE_ERROR;
    }
}

static void sys_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        shci_user_evt_proc();
    }
}

/* ------------------------------------------------------------------ */
/*                       BLE-события от стека                          */
/* ------------------------------------------------------------------ */

static void start_advertising(void)
{
    static const uint8_t local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'W','B','5','5','-','O','L','E','D' };
    /* Интервал рекламы 80..100 мс (в единицах по 0.625 мс) */
    tBleStatus st = aci_gap_set_discoverable(ADV_IND, 0x80, 0xA0, GAP_PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                             sizeof local_name, local_name, 0, NULL, 0, 0);
    if (st == BLE_STATUS_SUCCESS) state = BLE_STATE_ADVERTISING;
    else { g_ble_error = 0x100 | st; state = BLE_STATE_ERROR; }
}

/* Пришли данные от телефона: каждая запись — одна или несколько строк */
static void on_rx(const uint8_t *data, uint16_t len)
{
    char line[64];
    size_t n = 0;
    for (uint16_t i = 0; i <= len; i++) {
        char c = (i < len) ? (char)data[i] : '\n';   /* конец записи = конец строки */
        if (c == '\r' || c == '\n') {
            if (n) { line[n] = '\0'; app_handle_line(line, ble_print); n = 0; }
        } else if (n < sizeof line - 1 && c >= 0x20 && c < 0x7F) {
            line[n++] = c;
        }
    }
}

static void ble_event(void *payload)
{
    tHCI_UserEvtRxParam *p = payload;
    hci_event_pckt *evt = (hci_event_pckt *)((hci_uart_pckt *)&p->pckt->evtserial)->data;
    p->status = HCI_TL_UserEventFlow_Enable;

    switch (evt->evt) {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
        conn_handle = 0xFFFF;
        tx_notify_on = 0;
        att_mtu = 23;
        start_advertising();                       /* телефон ушёл — снова ждём */
        break;

    case HCI_LE_META_EVT_CODE: {
        evt_le_meta_event *meta = (evt_le_meta_event *)evt->data;
        if (meta->subevent == HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE ||
            meta->subevent == HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE) {
            hci_le_connection_complete_event_rp0 *c = (hci_le_connection_complete_event_rp0 *)meta->data;
            if (c->Status == 0) {
                conn_handle = c->Connection_Handle;
                state = BLE_STATE_CONNECTED;
            }
        }
        break;
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE: {
        evt_blecore_aci *vs = (evt_blecore_aci *)evt->data;
        if (vs->ecode == ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE) {
            aci_gatt_attribute_modified_event_rp0 *m = (aci_gatt_attribute_modified_event_rp0 *)vs->data;
            /* Хэндлы: +1 — значение характеристики, +2 — её дескриптор подписки (CCCD) */
            if (m->Attr_Handle == nus_rx + 1)
                on_rx(m->Attr_Data, m->Attr_Data_Length);
            else if (m->Attr_Handle == nus_tx + 2)
                tx_notify_on = m->Attr_Data[0] & 1;
        } else if (vs->ecode == ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE) {
            aci_att_exchange_mtu_resp_event_rp0 *m = (aci_att_exchange_mtu_resp_event_rp0 *)vs->data;
            att_mtu = m->Server_RX_MTU < CFG_BLE_MAX_ATT_MTU ? m->Server_RX_MTU : CFG_BLE_MAX_ATT_MTU;
        }
        break;
    }
    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/*                          Инициализация                              */
/* ------------------------------------------------------------------ */

/* Bluetooth-адрес из уникального номера чипа — у каждой платы свой */
static void bd_address(uint8_t a[6])
{
    uint32_t udn     = *(volatile uint32_t *)UID64_BASE;
    uint32_t w1      = *((volatile uint32_t *)UID64_BASE + 1);
    uint8_t  dev_id  = (uint8_t)w1;
    uint32_t company = (w1 >> 8) & 0xFFFFFF;    /* у ST = 0x0080E1 */
    a[0] = (uint8_t)udn;
    a[1] = (uint8_t)(udn >> 8);
    a[2] = dev_id;
    a[3] = (uint8_t)company;
    a[4] = (uint8_t)(company >> 8);
    a[5] = (uint8_t)(company >> 16);
}

#define CHECK(step, call) do { tBleStatus _s = (call); \
    if (_s != BLE_STATUS_SUCCESS) { g_ble_error = ((step) << 8) | _s; state = BLE_STATE_ERROR; return; } } while (0)

static void gatt_init(void)
{
    uint16_t gap_svc, name_char, appearance_char;
    uint8_t addr[6];
    Service_UUID_t su;
    Char_UUID_t cu;

    bd_address(addr);
    CHECK(0x10, hci_reset());
    CHECK(0x11, aci_hal_write_config_data(CONFIG_DATA_PUBLIC_ADDRESS_OFFSET, 6, addr));
    CHECK(0x12, aci_hal_set_tx_power_level(1, 0x18));          /* около 0 дБм */
    CHECK(0x13, aci_gatt_init());
    CHECK(0x14, aci_gap_init(GAP_PERIPHERAL_ROLE, PRIVACY_DISABLED, sizeof dev_name - 1,
                             &gap_svc, &name_char, &appearance_char));
    CHECK(0x15, aci_gatt_update_char_value(gap_svc, name_char, 0, sizeof dev_name - 1, (const uint8_t *)dev_name));

    /* Nordic UART Service: RX (запись с телефона) + TX (уведомления на телефон) */
    memcpy(su.Service_UUID_128, nus_svc_uuid, 16);
    CHECK(0x20, aci_gatt_add_service(UUID_TYPE_128, &su, PRIMARY_SERVICE, 8, &nus_svc));
    memcpy(cu.Char_UUID_128, nus_rx_uuid, 16);
    CHECK(0x21, aci_gatt_add_char(nus_svc, UUID_TYPE_128, &cu, CFG_BLE_MAX_ATT_MTU - 3,
                                  CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE,
                                  GATT_NOTIFY_ATTRIBUTE_WRITE, 16, CHAR_VALUE_LEN_VARIABLE, &nus_rx));
    memcpy(cu.Char_UUID_128, nus_tx_uuid, 16);
    CHECK(0x22, aci_gatt_add_char(nus_svc, UUID_TYPE_128, &cu, CFG_BLE_MAX_ATT_MTU - 3,
                                  CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE,
                                  GATT_DONT_NOTIFY_EVENTS, 16, CHAR_VALUE_LEN_VARIABLE, &nus_tx));

    /* Environmental Sensing: одна характеристика «Температура» (чтение + уведомления) */
    su.Service_UUID_16 = ESS_UUID;
    CHECK(0x30, aci_gatt_add_service(UUID_TYPE_16, &su, PRIMARY_SERVICE, 4, &ess_svc));
    cu.Char_UUID_16 = TEMP_CHAR_UUID;
    CHECK(0x31, aci_gatt_add_char(ess_svc, UUID_TYPE_16, &cu, 2, CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                                  ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS, 16,
                                  CHAR_VALUE_LEN_CONSTANT, &temp_char));

    start_advertising();
}

static void stack_init(void)
{
    /* Сообщаем CPU2, какие системные события нам интересны */
    SHCI_C2_CONFIG_Cmd_Param_t cfg = { 0 };
    cfg.PayloadCmdSize = SHCI_C2_CONFIG_PAYLOAD_CMD_SIZE;
    cfg.EvtMask1 = SHCI_C2_CONFIG_EVTMASK1_BIT0_ERROR_NOTIF_ENABLE;
    cfg.RevisionID = (uint16_t)(DBGMCU->IDCODE >> 16);
    cfg.DeviceID = (uint16_t)(DBGMCU->IDCODE & 0xFFF);
    SHCI_C2_Config(&cfg);

    /* Версия BLE-стека — из общей таблицы, которую заполнило CPU2 */
    WirelessFwInfo_t info;
    if (SHCI_GetWirelessFwInfo(&info) == SHCI_Success)
        g_ble_fw_version = ((uint32_t)info.VersionMajor << 16) | ((uint32_t)info.VersionMinor << 8) | info.VersionSub;

    HCI_TL_HciInitConf_t hci_conf = { .p_cmdbuffer = (uint8_t *)&BleCmdBuffer, .StatusNotCallBack = hci_status };
    hci_init(ble_event, &hci_conf);

    SHCI_C2_Ble_Init_Cmd_Packet_t init = {
        {{0, 0, 0}},
        {
            0, 0,
            CFG_BLE_NUM_GATT_ATTRIBUTES, CFG_BLE_NUM_GATT_SERVICES, CFG_BLE_ATT_VALUE_ARRAY_SIZE,
            CFG_BLE_NUM_LINK, CFG_BLE_DATA_LENGTH_EXTENSION, CFG_BLE_PREPARE_WRITE_LIST_SIZE,
            CFG_BLE_MBLOCK_COUNT, CFG_BLE_MAX_ATT_MTU, CFG_BLE_PERIPHERAL_SCA, CFG_BLE_CENTRAL_SCA,
            CFG_BLE_LS_SOURCE, CFG_BLE_MAX_CONN_EVENT_LENGTH, CFG_BLE_HSE_STARTUP_TIME,
            CFG_BLE_VITERBI_MODE, CFG_BLE_OPTIONS, 0, CFG_BLE_MAX_COC_INITIATOR_NBR,
            CFG_BLE_MIN_TX_POWER, CFG_BLE_MAX_TX_POWER, CFG_BLE_RX_MODEL_CONFIG,
            CFG_BLE_MAX_ADV_SET_NBR, CFG_BLE_MAX_ADV_DATA_LEN, CFG_BLE_TX_PATH_COMPENS,
            CFG_BLE_RX_PATH_COMPENS, CFG_BLE_CORE_VERSION, CFG_BLE_OPTIONS_EXT,
            CFG_BLE_MAX_ADD_EATT_BEARERS,
        }
    };
    SHCI_CmdStatus_t st = SHCI_C2_BLE_Init(&init);
    if (st != SHCI_Success) { g_ble_error = 0x0B00 | st; state = BLE_STATE_ERROR; return; }

    gatt_init();
}

/* ------------------------------------------------------------------ */
/*                           Задача BLE                                */
/* ------------------------------------------------------------------ */

static void send_pending(void)
{
    static uint8_t chunk[CFG_BLE_MAX_ATT_MTU];
    static size_t chunk_len;

    if (state != BLE_STATE_CONNECTED || !tx_notify_on) {
        xStreamBufferReset(tx_sb);                  /* никто не слушает — выбрасываем */
        chunk_len = 0;
        return;
    }
    for (;;) {
        if (!chunk_len)
            chunk_len = xStreamBufferReceive(tx_sb, chunk, att_mtu - 3, 0);
        if (!chunk_len) return;
        tBleStatus st = aci_gatt_update_char_value(nus_svc, nus_tx, 0, (uint8_t)chunk_len, chunk);
        if (st == BLE_STATUS_INSUFFICIENT_RESOURCES) return;   /* очередь радио полна — допошлём позже */
        chunk_len = 0;
    }
}

static void update_temperature(void)
{
    int16_t t = (int16_t)(g_temp_x10 * 10);        /* десятые -> сотые доли градуса */
    aci_gatt_update_char_value(ess_svc, temp_char, 0, 2, (const uint8_t *)&t);
}

static void ble_task(void *arg)
{
    (void)arg;

    /* 1. Транспортный слой и запуск CPU2 */
    TL_Init();
    SHCI_TL_HciInitConf_t shci_conf = { .p_cmdbuffer = (uint8_t *)&SystemCmdBuffer, .StatusNotCallBack = shci_status };
    shci_init(sys_event, &shci_conf);
    TL_MM_Config_t mm = {
        .p_BleSpareEvtBuffer    = BleSpareEvtBuffer,
        .p_SystemSpareEvtBuffer = SystemSpareEvtBuffer,
        .p_AsynchEvtPool        = EvtPool,
        .AsynchEvtPoolSize      = POOL_SIZE,
    };
    TL_MM_Init(&mm);
    TL_Enable();

    /* 2. Ждём READY от CPU2 */
    uint32_t bits = 0;
    xTaskNotifyWait(0, EVT_SYS_READY, &bits, pdMS_TO_TICKS(5000));
    if (!(bits & EVT_SYS_READY)) {
        if (state != BLE_STATE_ERROR) { g_ble_error = 0xA0; state = BLE_STATE_ERROR; }   /* CPU2 молчит */
    } else {
        stack_init();
    }

    /* 3. Основной цикл: события стека, отправка текста, температура раз в секунду */
    TickType_t last_temp = xTaskGetTickCount();
    for (;;) {
        xTaskNotifyWait(0, 0xFFFFFFFF, &bits, pdMS_TO_TICKS(xStreamBufferIsEmpty(tx_sb) ? 200 : 10));
        if (state == BLE_STATE_ERROR) continue;
        if (bits & EVT_HCI) hci_user_evt_proc();
        send_pending();
        if (xTaskGetTickCount() - last_temp >= pdMS_TO_TICKS(1000)) {
            last_temp = xTaskGetTickCount();
            update_temperature();
        }
    }
}

void ble_app_start(void)
{
    HW_IPCC_Reset();
    /* CPU2 в режиме сна может уходить в «выключение» — так разрешили в примерах ST */
    PWR->C2CR1 = (PWR->C2CR1 & ~PWR_C2CR1_LPMS_Msk) | (4U << PWR_C2CR1_LPMS_Pos);
    /* Генератор случайных чисел (RNG) нужен BLE-стеку для ключей и адресов.
       Пользуется им CPU2, но включить тактирование должно наше ядро — иначе
       CPU2 бесконечно ждёт число и SHCI_C2_BLE_Init не возвращается.
       Источник RNG — CLK48 (RNGSEL = 00), а CLK48 у нас — HSI48. */
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) { }
    RCC->CCIPR &= ~(RCC_CCIPR_RNGSEL_Msk | RCC_CCIPR_CLK48SEL_Msk);
    RCC->AHB3ENR |= RCC_AHB3ENR_RNGEN;

    /* Линии EXTI 36 (IPCC) и 38 (HSEM) должны уметь будить наше ядро */
    EXTI->IMR2 |= EXTI_IMR2_IM36 | EXTI_IMR2_IM38;

    shci_sem = xSemaphoreCreateBinary();
    shci_mtx = xSemaphoreCreateMutex();
    hci_sem  = xSemaphoreCreateBinary();
    hci_mtx  = xSemaphoreCreateMutex();
    tx_mtx   = xSemaphoreCreateMutex();
    tx_sb    = xStreamBufferCreate(1024, 1);

    xTaskCreate(sys_task, "ble_sys", 384, NULL, 3, &sys_task_h);
    xTaskCreate(ble_task, "ble",     768, NULL, 2, &ble_task_h);
}

ble_state_t ble_app_state(void) { return state; }

void ble_print(const char *s)
{
    if (state != BLE_STATE_CONNECTED || !tx_notify_on) return;
    xSemaphoreTake(tx_mtx, portMAX_DELAY);
    xStreamBufferSend(tx_sb, s, strlen(s), pdMS_TO_TICKS(50));
    xSemaphoreGive(tx_mtx);
    if (xTaskGetCurrentTaskHandle() != ble_task_h)
        xTaskNotify(ble_task_h, EVT_TX, eSetBits);
}
