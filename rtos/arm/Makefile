CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy

CPU_FLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
CFLAGS    := $(CPU_FLAGS) -Wall -Wextra -O1 -g -ffreestanding -fno-builtin -std=gnu11
LDFLAGS   := $(CPU_FLAGS) -T linker.ld -nostdlib -nostartfiles -Wl,--gc-sections -Wl,-Map=build/kernel.map

BUILD_DIR := build
SRCS_C    := kernel_arm.c uart.c main.c
SRCS_S    := startup.s context_switch.s
OBJS      := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SRCS_C)) $(patsubst %.s,$(BUILD_DIR)/%.o,$(SRCS_S))

ELF := $(BUILD_DIR)/kernel.elf
BIN := $(BUILD_DIR)/kernel.bin

.PHONY: all clean

all: $(ELF) $(BIN)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) $(CPU_FLAGS) -c $< -o $@

$(ELF): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)
