##############################################################################
BUILD = build
BIN = dualdoser

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS += -W -Wall -g3 --std=gnu11 -Os
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=linker/samd11d14.ld

INCLUDES += \
  -Iinclude \
  -I. \
  -Itinyusb/src \
  -Itinyusb/hw/mcu/microchip/samd11/include

SRCS += \
  main.c \
  tinyusb/src/tusb.c \
  tinyusb/src/device/usbd_control.c \
  tinyusb/src/device/usbd.c \
  tinyusb/src/portable/microchip/samd/dcd_samd.c \
  tinyusb/src/class/vendor/vendor_device.c \
  tinyusb/src/common/tusb_fifo.c \
  usb_descriptors.c \
  startup_samd11.c

DEFINES += \
  -D__SAMD11C14A__ \
  -DDONT_USE_CMSIS_INIT \
  -DSTARTUP_FROM_RESET \
  -DF_CPU=48000000 \
  -DCFG_TUSB_MCU=OPT_MCU_SAMD11

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(subst .c,.o, $(SRCS)))

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	$(CC) $(CFLAGS) $(subst build/,,$(subst .o,.c,$@)) -c -o $@

directory:
	@mkdir -p $(BUILD)
	@mkdir -p $(BUILD)/tinyusb/src/device/
	@mkdir -p $(BUILD)/tinyusb/src/portable/microchip/samd/
	@mkdir -p $(BUILD)/tinyusb/src/class/vendor
	@mkdir -p $(BUILD)/tinyusb/src/common

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

