CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

TARGET_NAME = kite
OBJDIR      = build
SRC_DIR     = src
PLATFORM_DIR = platform
INC_DIR     = inc

CFLAGS  = -mcpu=cortex-m4 -mthumb -g3 -Wall -O0 \
          -ffunction-sections -fdata-sections \
          -ffreestanding -nostdlib \
          -I$(INC_DIR) -I$(PLATFORM_DIR)

LDFLAGS = -T $(PLATFORM_DIR)/linker.ld -Wl,--gc-sections \
          -nostdlib

C_SRC = $(wildcard $(SRC_DIR)/*.c) $(wildcard $(PLATFORM_DIR)/*.c)
ASM_SRC = $(PLATFORM_DIR)/startup.S
OBJ   = $(patsubst %.c,$(OBJDIR)/%.o,$(notdir $(C_SRC))) $(OBJDIR)/startup.o

TARGET = $(OBJDIR)/$(TARGET_NAME).elf
BIN    = $(OBJDIR)/$(TARGET_NAME).bin
LST    = $(OBJDIR)/$(TARGET_NAME).lst

all: $(OBJDIR) $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	$(OBJCOPY) -O binary $@ $(BIN)
	$(OBJDUMP) -D $@ > $(LST)
	$(SIZE) $@

$(OBJDIR)/%.o: $(SRC_DIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: $(PLATFORM_DIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/startup.o: $(ASM_SRC) | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR):
	mkdir -p $(OBJDIR)

burn: $(TARGET)
	st-flash --connect-under-reset write $(BIN) 0x08000000

connect:
	openocd -f /usr/share/openocd/scripts/interface/stlink.cfg \
	        -f /usr/share/openocd/scripts/target/stm32f4x.cfg

debug: $(TARGET)
	gdb-multiarch $(TARGET) \
	    -ex "target extended-remote localhost:3333" \
	    -ex "monitor reset halt" \
	    -ex "load" \
	    -ex "break main" \
	    -ex "continue"

clean:
	rm -rf $(OBJDIR)

.PHONY: all clean burn connect debug