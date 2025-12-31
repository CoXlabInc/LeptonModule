OBJECTS := $(patsubst %.c,%.o,$(wildcard *.c)) ../lepton_control/lep_i2cdev.o
CFLAGS = -g -O2 -I../lepton_control -I.

liblepton_sdk.a: ${OBJECTS}
	${CROSS_COMPILE}ar rcs $@ ${OBJECTS}

../lepton_control/lep_i2cdev.o: ../lepton_control/lep_i2cdev.c ../lepton_control/lep_i2cdev.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o *.a
	rm -f ../lepton_control/*.o

.PHONY: clean
