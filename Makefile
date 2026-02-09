CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy

BUILD := build
TARGET := firmware

CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)


LDFLAGS ?= -Tplatform/link.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).elf.map

SOURCES := app/main.c platform/startup.c platform/syscalls.c \
           $(wildcard bsp/*.c) $(wildcard drivers/*.c)

OBJS := $(patsubst %.c,$(BUILD)/%.o,$(SOURCES))

ifeq ($(OS),Windows_NT)
  RM = cmd /C del /Q /F
else
  RM = rm -f
endif

build: $(BUILD)/$(TARGET).bin

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(CFLAGS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

.PHONY: build flash clean


clean:
	rm -rf build

