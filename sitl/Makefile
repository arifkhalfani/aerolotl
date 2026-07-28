CC      := gcc
BIN     := build/aerolotl_sitl

# Path to the FreeRTOS-Kernel checkout. Override on the command line if yours
# lives somewhere else: make KERNEL_DIR=/path/to/FreeRTOS-Kernel
KERNEL_DIR ?= ../FreeRTOS-Kernel

INCLUDES := \
	-Iconfig \
	-I$(KERNEL_DIR)/include \
	-I$(KERNEL_DIR)/portable/ThirdParty/GCC/Posix \
	-I$(KERNEL_DIR)/portable/ThirdParty/GCC/Posix/utils

SRC := \
	src/main.c \
	src/kalman.c \
	src/physics_sim.c \
	src/sim_world.c \
	src/accel_sim.c \
	src/baro_sim.c \
	$(KERNEL_DIR)/tasks.c \
	$(KERNEL_DIR)/queue.c \
	$(KERNEL_DIR)/list.c \
	$(KERNEL_DIR)/timers.c \
	$(KERNEL_DIR)/event_groups.c \
	$(KERNEL_DIR)/portable/MemMang/heap_3.c \
	$(KERNEL_DIR)/portable/ThirdParty/GCC/Posix/port.c \
	$(KERNEL_DIR)/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c

CFLAGS  := -Wall -Wextra -O0 -g -pthread $(INCLUDES)
LDFLAGS := -pthread -lm

all: $(BIN)

build:
	mkdir -p build

$(BIN): build $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(BIN) $(LDFLAGS)

run: $(BIN)
	./$(BIN)

clean:
	rm -rf build

.PHONY: all run clean build