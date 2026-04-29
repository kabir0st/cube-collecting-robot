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
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I$(INCLUDE_DIR) -pthread

# Linker flags
LDFLAGS = -pthread

CXXFLAGS += -I/usr/include/opencv4
LDFLAGS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio

# Remove problematic modules if they exist
LDFLAGS := $(filter-out -lopencv_viz -lopencv_hdf, $(LDFLAGS))

# =========================
# Sources and Objects
# =========================
SRCS = main.cpp $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(notdir $(SRCS)))

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