TARGET = bootutil
CC = gcc
CFLAGS = -g -Wall
ifneq ($(OS),Windows_NT) # ncurses support limited to *NIX for now
LIBS = -lncurses
endif

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
HEADERS = $(wildcard *.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -Wall -o $@ $(LIBS)

clean:
	-rm -f *.o
	-rm -f $(TARGET)
