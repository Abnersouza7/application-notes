# AN0002 verification images for the NUCLEO-H503RB.
#
#   make            build both images into build/
#   make hex        also emit Intel HEX alongside the raw binaries
#   make clean
#
# Requires the Arm GNU toolchain (arm-none-eabi-gcc). Tested with 14.2.Rel1.
#
# Copyright (c) 2026 Binho Inc. Released under the MIT license.

CROSS   ?= arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

BUILD   := build
TARGETS := $(BUILD)/an0002_image_a.bin $(BUILD)/an0002_image_b.bin

# Cortex-M33 without the FPU or DSP extensions: nothing here needs them, and a
# soft-float build has no startup requirements beyond copying .data.
ARCH    := -mcpu=cortex-m33 -mthumb -mfloat-abi=soft

CFLAGS  := $(ARCH) -Os -g3 -std=c11 \
           -ffunction-sections -fdata-sections \
           -Wall -Wextra -Werror \
           -fno-builtin -nostdlib

# Recursively expanded: $@ must be evaluated when the rule runs, not now.
LDFLAGS = $(ARCH) -T link.ld -nostdlib -Wl,--gc-sections -Wl,-Map=$(@:.elf=.map)

.PHONY: all hex clean
all: $(TARGETS)
	@$(SIZE) $(BUILD)/*.elf

hex: all $(TARGETS:.bin=.hex)

$(BUILD):
	@mkdir -p $(BUILD)

# One source, two images, selected by IMAGE_ID.
$(BUILD)/an0002_image_a.elf: main.c link.ld | $(BUILD)
	$(CC) $(CFLAGS) -DIMAGE_ID="'A'" $< $(LDFLAGS) -o $@

$(BUILD)/an0002_image_b.elf: main.c link.ld | $(BUILD)
	$(CC) $(CFLAGS) -DIMAGE_ID="'B'" $< $(LDFLAGS) -o $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -rf $(BUILD)
