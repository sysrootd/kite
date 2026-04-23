CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

TARGET_NAME = kite
OBJDIR      = build

CFLAGS  = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -g3 -Wall -O0 \
          -ffunction-sections -fdata-sections \
          -ffreestanding -nostdlib

LDFLAGS = -T sys/platform/linker.ld -Wl,--gc-sections -nostdlib

C_SRC   = $(shell find . -name "*.c")
ASM_SRC = $(shell find . -name "*.S")

INCLUDES = $(sort $(dir $(shell find . -name "*.h")))
CFLAGS  += $(addprefix -I,$(INCLUDES))

OBJ = $(patsubst %.c,$(OBJDIR)/%.o,$(C_SRC)) \
      $(patsubst %.S,$(OBJDIR)/%.o,$(ASM_SRC))

TARGET = $(OBJDIR)/$(TARGET_NAME).elf
BIN    = $(OBJDIR)/$(TARGET_NAME).bin
LST    = $(OBJDIR)/$(TARGET_NAME).lst

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	$(OBJCOPY) -O binary $@ $(BIN)
	$(OBJDUMP) -D $@ > $(LST)
	$(SIZE) $@

$(OBJDIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.S
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

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