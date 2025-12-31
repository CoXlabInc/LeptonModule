
TARGET = raspberrypi_capture

RPI_LIBS = ../raspberrypi_libs
LEPTONSDK_DIR = $(RPI_LIBS)/leptonSDKEmb32PUB
LEPTONSDK_LIB = $(LEPTONSDK_DIR)/Debug

# We need to build the SDK first
.PHONY: default all clean sdk

default: sdk $(TARGET)
all: default

CC = gcc
CFLAGS = -g -Wall -std=gnu99 -I$(LEPTONSDK_DIR)
LIBS = -L$(LEPTONSDK_LIB) -lLEPTON_SDK -lm

sdk:
	$(MAKE) -C $(LEPTONSDK_DIR)

OBJECTS = $(patsubst %.c, %.o, $(wildcard *.c))
HEADERS = $(wildcard *.h)

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)
	$(MAKE) -C $(LEPTONSDK_DIR) clean

