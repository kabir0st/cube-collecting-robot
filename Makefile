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
# servo.c is C, not C++, so it's not picked up by the .cpp wildcard above.
# It has its own compile rule below, but must be linked explicitly.
OBJS += $(BUILD_DIR)/servo.o


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

# Compile servo_test.cpp
$(BUILD_DIR)/servo_test.o: servo_test.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile src/servo.c (C source)
$(BUILD_DIR)/servo.o: $(SRC_DIR)/servo.c | $(BUILD_DIR)
	$(CC) -std=c99 -Wall -O2 -I$(INCLUDE_DIR) -c $< -o $@

# Compile src/*.cpp files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Minimal servo test — only servo.c + wiringPi, no app deps.
test: $(BUILD_DIR) $(BUILD_DIR)/servo_test.o $(BUILD_DIR)/servo.o
	$(CXX) -o servo_test $(BUILD_DIR)/servo_test.o $(BUILD_DIR)/servo.o -lwiringPi
	@echo "Build complete: servo_test — running…"
	./servo_test

# Clean
clean:
	rm -rf $(BUILD_DIR) $(TARGET) servo_test

# for syntax check
check: CXXFLAGS += -Istubs
check: $(OBJS)
	@echo "✔ Compile check successful"

.PHONY: all clean run info test check