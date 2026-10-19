# Сборка: make         Прошивка: make flash        Очистка: make clean
# Тулчейн берётся из установленного STM32CubeIDE.

CUBEIDE  ?= C:/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins
GCC_DIR  ?= $(CUBEIDE)/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin
PROG     ?= $(CUBEIDE)/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.400.202601091506/tools/bin/STM32_Programmer_CLI.exe

CC      = $(GCC_DIR)/arm-none-eabi-gcc
OBJCOPY = $(GCC_DIR)/arm-none-eabi-objcopy
SIZE    = $(GCC_DIR)/arm-none-eabi-size

TARGET = oled_freertos
BUILD  = build

SRCS = \
  Core/Src/main.c \
  Core/Src/i2c.c \
  Core/Src/ssd1306.c \
  Core/Src/usb_cdc.c \
  Core/Src/temp.c \
  Core/Src/system_stm32wbxx.c \
  Middlewares/FreeRTOS/tasks.c \
  Middlewares/FreeRTOS/queue.c \
  Middlewares/FreeRTOS/list.c \
  Middlewares/FreeRTOS/timers.c \
  Middlewares/FreeRTOS/event_groups.c \
  Middlewares/FreeRTOS/stream_buffer.c \
  Middlewares/FreeRTOS/portable/port.c \
  Middlewares/FreeRTOS/portable/heap_4.c \
  $(BLE_SRCS)
ASRCS = Core/startup_stm32wb55xx_cm4.s

# Bluetooth: наш «клей» + транспорт и команды из библиотеки ST (STM32_WPAN)
WPAN = Middlewares/STM32_WPAN
BLE_SRCS = \
  Core/Src/wpan/ble_app.c \
  Core/Src/wpan/hw_ipcc.c \
  $(WPAN)/interface/tl/tl_mbox.c \
  $(WPAN)/interface/tl/hci_tl.c \
  $(WPAN)/interface/tl/hci_tl_if.c \
  $(WPAN)/interface/tl/shci_tl.c \
  $(WPAN)/interface/tl/shci_tl_if.c \
  $(WPAN)/interface/shci/shci.c \
  $(WPAN)/utilities/stm_list.c \
  $(WPAN)/ble/core/template/osal.c \
  $(WPAN)/ble/core/auto/ble_gap_aci.c \
  $(WPAN)/ble/core/auto/ble_gatt_aci.c \
  $(WPAN)/ble/core/auto/ble_hal_aci.c \
  $(WPAN)/ble/core/auto/ble_hci_le.c \
  $(WPAN)/ble/core/auto/ble_gen_aci.c

INCS = -ICore/Inc -ICore/Inc/wpan -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device \
       -IMiddlewares/FreeRTOS/include -IMiddlewares/FreeRTOS/portable \
       -I$(WPAN) -I$(WPAN)/ble -I$(WPAN)/ble/core -I$(WPAN)/ble/core/auto -I$(WPAN)/ble/core/template \
       -I$(WPAN)/interface -I$(WPAN)/interface/tl -I$(WPAN)/interface/shci -I$(WPAN)/utilities

MCU    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS = $(MCU) -DSTM32WB55xx $(INCS) -Og -g3 -Wall -ffunction-sections -fdata-sections -std=gnu11
LDFLAGS = $(MCU) -Tstm32wb55xx_flash_cm4.ld --specs=nano.specs --specs=nosys.specs \
          -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

OBJS = $(addprefix $(BUILD)/,$(SRCS:.c=.o) $(ASRCS:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	$(SIZE) $<

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

flash: $(BUILD)/$(TARGET).elf
	"$(PROG)" -c port=SWD mode=UR -w $< -v -rst

clean:
	rm -rf $(BUILD)

.PHONY: all flash clean
