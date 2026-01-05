# --- Toolchain Definitions ---
CC       = gcc
CXX      = g++
RM       = rm -f

# --- Project Configuration ---
TARGET   = simulation
BUILD_DIR = build

# --- Source Files ---
# All the .c files you listed in your directory
SRCS_C   = tasks.c \
           queue.c \
           list.c \
           timers.c \
           heap_4.c \
           port.c \
           wait_for_event.c

# Your application C++ files
SRCS_CXX = main.cpp

# --- Header Includes ---
# 1. Current directory (.) for FreeRTOSConfig.h and flattened sources
# 2. FreeRTOS-Kernel/include for the core API headers
# 3. You might need a path to 'portmacro.h' if it's not in the root
INCLUDES = -I. \
           -IFreeRTOS-Kernel/include \
           -IFreeRTOS-Kernel/portable/ThirdParty/GCC/Posix

# --- Compiler Flags ---
# -pthread is CRITICAL for the Linux port to work
COMMON_FLAGS = -g -Wall -pthread $(INCLUDES)

# C-specific flags
CFLAGS   = $(COMMON_FLAGS)

# C++-specific flags
CXXFLAGS = $(COMMON_FLAGS) -std=c++17

# --- Linker Flags ---
# Must link against pthread (threading) and rt (real-time clock)
LDFLAGS  = -pthread -lrt

# --- Object File Generation ---
OBJS_C   = $(SRCS_C:%.c=$(BUILD_DIR)/%.o)
OBJS_CXX = $(SRCS_CXX:%.cpp=$(BUILD_DIR)/%.o)
OBJS     = $(OBJS_C) $(OBJS_CXX)

# --- Build Rules ---

# 1. Default Rule: Build the target
all: $(BUILD_DIR) $(TARGET)

# 2. Link Everything Together
$(TARGET): $(OBJS)
	@echo "Linking $@"
	$(CXX) $(OBJS) -o $@ $(LDFLAGS)

# 3. Compile C Files
$(BUILD_DIR)/%.o: %.c
	@echo "Compiling $<"
	$(CC) $(CFLAGS) -c $< -o $@

# 4. Compile C++ Files
$(BUILD_DIR)/%.o: %.cpp
	@echo "Compiling $<"
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 5. Create Build Directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 6. Clean
clean:
	$(RM) -r $(BUILD_DIR) $(TARGET)

.PHONY: all clean
