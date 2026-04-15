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

# ---- ROS2 layer demo (Phase 4): separate entry point, reuses
# kernel_arm.c/uart.c/startup.s/context_switch.s as-is, swaps main.c for
# ros2_demo.c, and pulls in xrce/'s portable sources directly rather than
# duplicating them here. Kept entirely separate from the `all`/$(ELF)
# rules above so the existing, already-verified Phase 8 build is
# untouched. ----
XRCE_DIR      := ../../xrce
ROS2_CFLAGS   := $(CFLAGS) -I$(XRCE_DIR)/include
ROS2_APP_SRCS := kernel_arm.c uart.c libc_shim.c ros2_demo.c
ROS2_XRCE_SRCS := serial_transport.c cdr.c msgs.c session.c
ROS2_OBJS     := $(patsubst %.c,$(BUILD_DIR)/ros2/%.o,$(ROS2_APP_SRCS)) \
                 $(patsubst %.c,$(BUILD_DIR)/ros2/%.o,$(ROS2_XRCE_SRCS)) \
                 $(BUILD_DIR)/startup.o $(BUILD_DIR)/context_switch.o

ROS2_ELF := $(BUILD_DIR)/ros2_demo.elf
ROS2_BIN := $(BUILD_DIR)/ros2_demo.bin

$(BUILD_DIR)/ros2/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/ros2
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(BUILD_DIR)/ros2/serial_transport.o: $(XRCE_DIR)/src/serial_transport.c | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/ros2
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(BUILD_DIR)/ros2/cdr.o: $(XRCE_DIR)/src/cdr.c | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/ros2
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(BUILD_DIR)/ros2/msgs.o: $(XRCE_DIR)/src/msgs.c | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/ros2
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(BUILD_DIR)/ros2/session.o: $(XRCE_DIR)/src/session.c | $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)/ros2
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(ROS2_ELF): $(ROS2_OBJS)
	$(CC) $(LDFLAGS) $(ROS2_OBJS) -o $@

$(ROS2_BIN): $(ROS2_ELF)
	$(OBJCOPY) -O binary $< $@

.PHONY: ros2_demo
ros2_demo: $(ROS2_ELF) $(ROS2_BIN)
