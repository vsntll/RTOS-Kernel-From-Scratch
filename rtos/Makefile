CC      := gcc
CFLAGS  := -std=gnu11 -Wall -Wextra -g -Isrc
LDFLAGS := -lpthread

ifdef SANITIZE
CFLAGS  += -fsanitize=$(SANITIZE)
LDFLAGS += -fsanitize=$(SANITIZE)
endif

BUILD_DIR   := build
SRC_DIR     := src
TEST_DIR    := tests
EXAMPLE_DIR := examples

KERNEL_SRCS := $(wildcard $(SRC_DIR)/*.c)
KERNEL_OBJS := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/obj/%.o,$(KERNEL_SRCS))
LIB         := $(BUILD_DIR)/librtos.a

TEST_SRCS   := $(wildcard $(TEST_DIR)/*.c)
TESTS       := $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/tests/%,$(TEST_SRCS))

EXAMPLE_SRCS := $(wildcard $(EXAMPLE_DIR)/*.c)
EXAMPLES      := $(patsubst $(EXAMPLE_DIR)/%.c,$(BUILD_DIR)/examples/%,$(EXAMPLE_SRCS))

.PHONY: all test examples clean

all: $(LIB) $(TESTS) $(EXAMPLES)

$(BUILD_DIR)/obj/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB): $(KERNEL_OBJS)
	@mkdir -p $(dir $@)
	ar rcs $@ $^

$(BUILD_DIR)/tests/%: $(TEST_DIR)/%.c $(LIB)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< $(LIB) -o $@ $(LDFLAGS)

$(BUILD_DIR)/examples/%: $(EXAMPLE_DIR)/%.c $(LIB)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< $(LIB) -o $@ $(LDFLAGS)

examples: $(EXAMPLES)

test: $(TESTS)
	@for t in $(TESTS); do \
		echo "== $$t =="; \
		./$$t || exit 1; \
	done

clean:
	rm -rf $(BUILD_DIR)
