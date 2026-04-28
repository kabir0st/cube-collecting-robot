CC = g++ # C++ compiler
CFLAGS = -Wall # C flags
LDFLAGS = -shared  # linking flags
RM = rm -f  # rm command
TARGET_LIB = -lpthread # target lib
EXEC = threads

SRCS = main.cpp mythreads.cpp # source files
OBJS = $(SRCS:.c=.o)

.PHONY: all
all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) $^ $@ -o $(EXEC)

$(SRCS:.cpp):%.cpp
	$(CC) $(CFLAGS) -MM $< >$@

include $(SRCS:.cpp)

.PHONY: clean
clean:
	-${RM} $(OBJS)


#${OBJS} 
