# --- Toolchain Definitions ---
CC       = gcc
CXX      = g++
RM       = rm -f

# --- Project Configuration ---
TARGET    = simulation
BUILD_DIR = build
LIB_DIR   = libs

# --- Source Files ---
# Prefix the C files with the library directory
SRCS_C   = $(LIB_DIR)/tasks.c \
           $(LIB_DIR)/queue.c \
           $(LIB_DIR)/list.c \
           $(LIB_DIR)/timers.c \
           $(LIB_DIR)/heap_4.c \
           $(LIB_DIR)/port.c \
           $(LIB_DIR)/wait_for_event.c

# Your application C++ files (still in root)
SRCS_CXX = target_node.cpp

# --- Header Includes ---
# 1. Added $(LIB_DIR) so the compiler finds FreeRTOSConfig.h and wait_for_event.h
INCLUDES = -I. \
           -I$(LIB_DIR) \
           -IFreeRTOS-Kernel/include \
           -IFreeRTOS-Kernel/portable/ThirdParty/GCC/Posix

# --- Compiler Flags ---
COMMON_FLAGS = -g -Wall -pthread $(INCLUDES)
CFLAGS       = $(COMMON_FLAGS)
CXXFLAGS     = $(COMMON_FLAGS) -std=c++17

# --- Linker Flags ---
LDFLAGS  = -pthread -lrt

# --- Object File Generation ---
# This logic ensures objects are built in $(BUILD_DIR) even if sources are in $(LIB_DIR)
OBJS_C   = $(SRCS_C:$(LIB_DIR)/%.c=$(BUILD_DIR)/%.o)
OBJS_CXX = $(SRCS_CXX:%.cpp=$(BUILD_DIR)/%.o)
OBJS     = $(OBJS_C) $(OBJS_CXX)

# --- Build Rules ---

all: $(BUILD_DIR) $(TARGET)

$(TARGET): $(OBJS)
	@echo "Linking $@"
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# Rule for C files inside the libs folder
$(BUILD_DIR)/%.o: $(LIB_DIR)/%.c
	@echo "Compiling $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Rule for C++ files in the root
$(BUILD_DIR)/%.o: %.cpp
	@echo "Compiling $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	$(RM) -r $(BUILD_DIR) $(TARGET)

.PHONY: all clean
