CC = g++
FLAGS = -g

TARGET = build/main

SRCS = src/main.cpp

OBJS = $(SRCS: .cpp = .o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC)	$(FLAGS) -o $(TARGET) $(OBJS)

%.o:	%.cpp
	$(CC)	$(FLAGS)	-c	$<	-o	$@

clean:
	rm	-f	$(TARGET)