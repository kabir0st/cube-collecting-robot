# =========================
# Compiler
# =========================
CXX ?= g++

# =========================
# Directories
# =========================
INCLUDE_DIR = include
SRC_DIR = src
BUILD_DIR = build

TARGET = app

# =========================
# Platform detection
# =========================
UNAME_S := $(shell uname -s)

# =========================
# Compiler flags
# =========================
CXXFLAGS = -std=c++11 -Wall -Wextra -O3 -fno-math-errno -I$(INCLUDE_DIR) -pthread -MMD -MP

# Linker flags. -rdynamic so set_terminate's backtrace can resolve symbols.
LDFLAGS = -pthread -rdynamic

CXXFLAGS += -isystem /usr/include/eigen3 -DEIGEN_NO_DEBUG

# ARM-specific tuning (RPi 3B): only when building natively on ARM.
UNAME_M := $(shell uname -m)
ifeq ($(UNAME_M),aarch64)
  CXXFLAGS += -mcpu=cortex-a53
endif
ifeq ($(UNAME_M),armv7l)
  CXXFLAGS += -mcpu=cortex-a53 -mfpu=neon-fp-armv8 -mfloat-abi=hard
endif

# =========================
# Sources and Objects
# =========================
SRCS = main.cpp $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(notdir $(SRCS)))
DEPS = $(OBJS:.o=.d)
-include $(DEPS)

# =========================
# Build rules
# =========================
all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile main.cpp
$(BUILD_DIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile src/*.cpp files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Run
run: $(TARGET)
	./$(TARGET)

# Debug info
info:
	@echo "Compiler: $(CXX)"
	@echo "Platform: $(UNAME_S)"
	@echo "CXXFLAGS: $(CXXFLAGS)"
	@echo "LDFLAGS: $(LDFLAGS)"
	@echo "Sources: $(SRCS)"
	@echo "Objects: $(OBJS)"

.PHONY: all clean run info
