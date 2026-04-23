# Compiler
CXX = g++

# Directories
INCLUDE_DIR = include
SRC_DIR = src
BUILD_DIR = build

# Flags
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I$(INCLUDE_DIR)
CXXFLAGS += $(shell pkg-config --cflags opencv4)

# Get all libraries but remove viz and hdf
LDFLAGS = $(shell pkg-config --libs opencv4 | sed 's/-lopencv_viz//g' | sed 's/-lopencv_hdf//g')

# Target
TARGET = ./bin/app
SRCS = main.cpp $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(SRCS:.cpp=.o)))

all: $(BUILD_DIR) $(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Compile main.cpp
$(BUILD_DIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile src/*.cpp files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run