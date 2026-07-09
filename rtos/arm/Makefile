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
#
# NODE_ID (Phase 10, default 0): threaded into ros2_demo.c as -DNODE_ID=N,
# which parametrizes that file's client_key/participant/topic names so N
# separately-built firmware images are N distinct ROS2 nodes rather than N
# copies of one node. Object/ELF/BIN paths are namespaced by NODE_ID too
# so `make ros2_demo NODE_ID=1` and `make ros2_demo NODE_ID=2` don't
# clobber each other's build output -- host/run_multi_node.sh drives this
# for several NODE_ID values in a row.
#
# SECURE_LINK (Phase 11, default 0): threaded in as -DSECURE_LINK=1,
# switches ros2_demo.c to HMAC-authenticate every frame it sends/receives
# (xrce/src/secure_transport.c) instead of talking raw XRCE-DDS directly.
# Default 0 keeps `make ros2_demo` wire-identical to every earlier phase,
# still safe to point straight at a real `MicroXRCEAgent`; SECURE_LINK=1
# firmware must instead be pointed at host/secure_gateway.c, which
# verifies/strips the authentication and forwards plain XRCE-DDS to an
# unmodified agent -- see xrce/docs/design.md's Phase 11 section for why
# the agent itself is never taught about this layer.
NODE_ID       ?= 0
SECURE_LINK   ?= 0
XRCE_DIR      := ../../xrce
ROS2_CFLAGS   := $(CFLAGS) -I$(XRCE_DIR)/include -DNODE_ID=$(NODE_ID) -DSECURE_LINK=$(SECURE_LINK)
ROS2_APP_SRCS := kernel_arm.c uart.c libc_shim.c ros2_demo.c
ROS2_XRCE_SRCS := serial_transport.c cdr.c msgs.c session.c sha256.c hmac.c secure_transport.c
ifeq ($(SECURE_LINK),0)
SEC_SUFFIX :=
else
SEC_SUFFIX := _sec$(SECURE_LINK)
endif
ROS2_BUILD_DIR := $(BUILD_DIR)/ros2/node$(NODE_ID)$(SEC_SUFFIX)
ROS2_OBJS     := $(patsubst %.c,$(ROS2_BUILD_DIR)/%.o,$(ROS2_APP_SRCS)) \
                 $(patsubst %.c,$(ROS2_BUILD_DIR)/%.o,$(ROS2_XRCE_SRCS)) \
                 $(ROS2_BUILD_DIR)/startup.o $(ROS2_BUILD_DIR)/context_switch.o

ROS2_ELF := $(BUILD_DIR)/ros2_demo_node$(NODE_ID)$(SEC_SUFFIX).elf
ROS2_BIN := $(BUILD_DIR)/ros2_demo_node$(NODE_ID)$(SEC_SUFFIX).bin

$(ROS2_BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(ROS2_BUILD_DIR)
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(ROS2_BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@mkdir -p $(ROS2_BUILD_DIR)
	$(AS) $(CPU_FLAGS) -c $< -o $@

# All of ROS2_XRCE_SRCS live in $(XRCE_DIR)/src -- one pattern rule for
# all of them rather than one static rule per file.
$(ROS2_BUILD_DIR)/%.o: $(XRCE_DIR)/src/%.c | $(BUILD_DIR)
	@mkdir -p $(ROS2_BUILD_DIR)
	$(CC) $(ROS2_CFLAGS) -c $< -o $@

$(ROS2_ELF): $(ROS2_OBJS)
	$(CC) $(LDFLAGS) $(ROS2_OBJS) -o $@

$(ROS2_BIN): $(ROS2_ELF)
	$(OBJCOPY) -O binary $< $@

.PHONY: ros2_demo
ros2_demo: $(ROS2_ELF) $(ROS2_BIN)

# ---- Gazebo demo (Phase 14): a second, publish-only entry point sharing
# ros2_demo's session/framing machinery (xrce/ sources, NODE_ID/SECURE_LINK)
# but with its own main.c-equivalent (gazebo_demo.c) that drives a
# simulated vehicle's diff-drive plugin instead of the rt/chatter demo. ----
GZ_CFLAGS   := $(CFLAGS) -I$(XRCE_DIR)/include -DNODE_ID=$(NODE_ID) -DSECURE_LINK=$(SECURE_LINK)
GZ_APP_SRCS := kernel_arm.c uart.c libc_shim.c gazebo_demo.c
GZ_BUILD_DIR := $(BUILD_DIR)/gazebo/node$(NODE_ID)$(SEC_SUFFIX)
GZ_OBJS     := $(patsubst %.c,$(GZ_BUILD_DIR)/%.o,$(GZ_APP_SRCS)) \
               $(patsubst %.c,$(GZ_BUILD_DIR)/%.o,$(ROS2_XRCE_SRCS)) \
               $(GZ_BUILD_DIR)/startup.o $(GZ_BUILD_DIR)/context_switch.o

GZ_ELF := $(BUILD_DIR)/gazebo_demo_node$(NODE_ID)$(SEC_SUFFIX).elf
GZ_BIN := $(BUILD_DIR)/gazebo_demo_node$(NODE_ID)$(SEC_SUFFIX).bin

$(GZ_BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(GZ_BUILD_DIR)
	$(CC) $(GZ_CFLAGS) -c $< -o $@

$(GZ_BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@mkdir -p $(GZ_BUILD_DIR)
	$(AS) $(CPU_FLAGS) -c $< -o $@

$(GZ_BUILD_DIR)/%.o: $(XRCE_DIR)/src/%.c | $(BUILD_DIR)
	@mkdir -p $(GZ_BUILD_DIR)
	$(CC) $(GZ_CFLAGS) -c $< -o $@

$(GZ_ELF): $(GZ_OBJS)
	$(CC) $(LDFLAGS) $(GZ_OBJS) -o $@

$(GZ_BIN): $(GZ_ELF)
	$(OBJCOPY) -O binary $< $@

.PHONY: gazebo_demo
gazebo_demo: $(GZ_ELF) $(GZ_BIN)
