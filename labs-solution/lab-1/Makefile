# Compiler
CXX = g++

# Flags
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -I./libs
CXXFLAGS += $(shell pkg-config --cflags opencv4)

# Get all libraries but remove viz and hdf
LDFLAGS = $(shell pkg-config --libs opencv4 | sed 's/-lopencv_viz//g' | sed 's/-lopencv_hdf//g')

# Target
TARGET = lab_1
SRCS = main.cpp
OBJS = main.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c main.cpp -o main.o

clean:
	rm -f $(OBJS) $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run