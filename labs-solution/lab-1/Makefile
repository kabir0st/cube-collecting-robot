# Compiler
CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I./libs
CXXFLAGS += $(shell pkg-config --cflags opencv4)

# Remove viz and hdf from libraries
LDFLAGS = $(shell pkg-config --libs opencv4 | sed 's/-lopencv_viz//g' | sed 's/-lopencv_hdf//g')

# Directories
BIN_DIR = ./bin
BUILD_DIR = ./build

# Target
TARGET = $(BIN_DIR)/app
SRCS = main.cpp libs/scanner.cpp
OBJS = $(SRCS:%.cpp=$(BUILD_DIR)/%.o)
OBJS := $(OBJS:libs/%.o=$(BUILD_DIR)/%.o)

# Create directories
$(shell mkdir -p $(BIN_DIR) $(BUILD_DIR))

all: $(TARGET)

# Link
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Compile any .cpp to .o in build directory
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Handle libs/ directory specially
$(BUILD_DIR)/%.o: libs/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BIN_DIR) $(BUILD_DIR)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run