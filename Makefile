CXX ?= g++
INCLUDE_DIR = include
SRC_DIR = src
TOOLS_DIR = tools
BUILD_DIR = build
LIB_DIR = libraries

TARGET = app
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I$(INCLUDE_DIR)

MAKEFLAGS += -j3

LIBS = $(wildcard $(LIB_DIR)/*.so)

# OpenCV: prefer opencv4 (Bullseye+), fall back to opencv (older Raspbian).
OPENCV_PKG := $(shell pkg-config --exists opencv4 && echo opencv4 || echo opencv)
OPENCV_CFLAGS := $(shell pkg-config --cflags $(OPENCV_PKG))
OPENCV_LIBS   := $(shell pkg-config --libs   $(OPENCV_PKG))

LDFLAGS = -pthread -lwiringPi $(LIBS) $(OPENCV_LIBS) -Wl,-rpath,'$$ORIGIN/$(LIB_DIR)'

CXXFLAGS += -isystem /usr/include/eigen3 $(OPENCV_CFLAGS)

SRCS = main.cpp $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(notdir $(SRCS)))


all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile main.cpp
$(BUILD_DIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test_main.cpp
$(BUILD_DIR)/test_main.o: test_main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile src/*.cpp files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# for syntax check
check: CXXFLAGS += -Istubs
check: $(OBJS)
	@echo "✔ Compile check successful"

.PHONY: all clean run info