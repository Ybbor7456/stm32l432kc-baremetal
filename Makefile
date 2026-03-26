# ---- tools ----
CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy

# J-Link tools (Linux usually installs these)
JLINK ?= JLinkExe

# ---- config ----
BUILD := build
TARGET := firmware

# Set your exact MCU here (matches what you typed in JLinkExe)
JLINK_DEVICE ?= STM32L432KC
JLINK_IF ?= SWD
JLINK_SPEED ?= 4000

CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)

LDFLAGS ?= -Tplatform/link.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).elf.map

SOURCES := $(wildcard app/*.c) platform/startup.c platform/syscalls.c \
           $(wildcard bsp/*.c) $(wildcard drivers/*.c) $(wildcard devices/*.c)

OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SOURCES))

.PHONY: build flash flash-jlink flash-stlink erase reset clean

build: $(BUILD)/$(TARGET).bin

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(CFLAGS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# ---------- flashing ----------
# Default flash now uses J-Link (since your probe enumerates as SEGGER J-Link)
flash: flash-jlink

# Create a J-Link command file, then run it
$(BUILD)/jlink-flash.jlink: $(BUILD)/$(TARGET).bin
	@mkdir -p $(BUILD)
	@printf "si $(JLINK_IF)\n" >  $@
	@printf "speed $(JLINK_SPEED)\n" >> $@
	@printf "device $(JLINK_DEVICE)\n" >> $@
	@printf "r\n" >> $@
	@printf "h\n" >> $@
	@printf "loadbin $(BUILD)/$(TARGET).bin,0x08000000\n" >> $@
	@printf "r\n" >> $@
	@printf "g\n" >> $@
	@printf "q\n" >> $@

flash-jlink: $(BUILD)/jlink-flash.jlink
	$(JLINK) -NoGui 1 -CommandFile $<

# Keep ST-LINK method as an alternate target
flash-stlink: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

# Optional helpers
$(BUILD)/jlink-erase.jlink:
	@mkdir -p $(BUILD)
	@printf "si $(JLINK_IF)\n" >  $@
	@printf "speed $(JLINK_SPEED)\n" >> $@
	@printf "device $(JLINK_DEVICE)\n" >> $@
	@printf "r\n" >> $@
	@printf "h\n" >> $@
	@printf "erase\n" >> $@
	@printf "r\n" >> $@
	@printf "q\n" >> $@

erase: $(BUILD)/jlink-erase.jlink
	$(JLINK) -NoGui 1 -CommandFile $<

$(BUILD)/jlink-reset.jlink:
	@mkdir -p $(BUILD)
	@printf "si $(JLINK_IF)\n" >  $@
	@printf "speed $(JLINK_SPEED)\n" >> $@
	@printf "device $(JLINK_DEVICE)\n" >> $@
	@printf "r\n" >> $@
	@printf "g\n" >> $@
	@printf "q\n" >> $@

reset: $(BUILD)/jlink-reset.jlink
	$(JLINK) -NoGui 1 -CommandFile $<

clean:
	rm -rf build
