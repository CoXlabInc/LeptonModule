
TARGET = raspberrypi_capture

LEPTON_SDK_BASE = ../raspberrypi_libs
LEPTONSDK_DIR = $(LEPTON_SDK_BASE)/lepton_sdk
LEPTONSDK_LIB = $(LEPTONSDK_DIR)

# We need to build the SDK first
.PHONY: default all clean sdk

default: sdk $(TARGET)
all: default

CC = gcc
CFLAGS = -g -Wall -std=gnu99 -I$(LEPTONSDK_DIR) -I$(LEPTON_SDK_BASE)/lepton_control
LIBS = $(LEPTONSDK_LIB)/liblepton_sdk.a -lm

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

