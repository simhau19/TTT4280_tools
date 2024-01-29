CC = gcc

CFLAGS = -Wall -g
LDFLAGS = -lpthread -lpigpio -lm

SRC = remote_adc_server.c
TARGET = bin/remote_adc_server

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

rebuild:	clean all

.PHONY: all rebuild clean
