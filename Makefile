MCU    = atmega328p
F_CPU  = 16000000UL

CC      = avr-gcc
OBJCOPY = avr-objcopy
CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -D__AVR_ATmega328P__ -Os -I src -Wall -Wextra

SRC_DIR = src

SOURCES := $(wildcard $(SRC_DIR)/*.c)
OBJECTS := $(SOURCES:.c=.o)

TARGET  = $(SRC_DIR)/main.elf
HEXFILE = $(SRC_DIR)/main.hex

all: $(HEXFILE)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o $(TARGET)

$(HEXFILE): $(TARGET)
	$(OBJCOPY) -O ihex -R .eeprom $(TARGET) $(HEXFILE)

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

flash: $(HEXFILE)
	cp $(HEXFILE) /mnt/c/avrdude-v8.1-windows-x64/main.hex
	cmd.exe /c "C:\\avrdude-v8.1-windows-x64\\avrdude.exe -c arduino -p atmega328p -P COM4 -b 115200 -U flash:w:C:\\avrdude-v8.1-windows-x64\\main.hex"
clean:
	rm -f $(SRC_DIR)/*.o $(SRC_DIR)/*.elf $(SRC_DIR)/*.hex
.PHONY: all flash clean