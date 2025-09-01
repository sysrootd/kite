####################################
# Toolchain and flags
####################################
CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

# Base flags for C only
CFLAGS  = -mcpu=cortex-m4 -mthumb -g -Wall -O2 -ffunction-sections -fdata-sections -fno-unwind-tables -fno-asynchronous-unwind-tables

ASFLAGS = -mcpu=cortex-m4 -mthumb

# Linker script and libraries
LDFLAGS = -T linker.ld -nostartfiles -Wl,--gc-sections -lc -lgcc

####################################
# Directories
####################################
SRC_DIRS     := src system
INCLUDE_DIRS := inc system
INCDIRS      := $(addprefix -I,$(INCLUDE_DIRS))
OBJDIR       := build

####################################
# Source files
####################################
C_SRC   := $(shell find $(SRC_DIRS) -name '*.c')
ASM_SRC := $(shell find $(SRC_DIRS) -name '*.s')

# Object file names (flattened in OBJDIR)
OBJ = $(addprefix $(OBJDIR)/,$(notdir $(C_SRC:.c=.o))) \
      $(addprefix $(OBJDIR)/,$(notdir $(ASM_SRC:.s=.o)))

####################################
# Targets
####################################
TARGET = $(OBJDIR)/kernel.elf
BIN    = $(OBJDIR)/kernel.bin
LST    = $(OBJDIR)/kernel.lst

all: $(OBJDIR) $(TARGET)

# Link ELF and generate binary and listing
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	# Only include flash sections to avoid huge .bin
	$(OBJCOPY) -O binary \
        --only-section=.isr_vector \
        --only-section=.text \
        --only-section=.rodata \
        $@ $(BIN)
	$(OBJDUMP) -D $@ > $(LST)
	$(SIZE) $@

####################################
# Compile .c files
####################################
$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCDIRS) -c $< -o $@

$(OBJDIR)/%.o: system/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCDIRS) -c $< -o $@

####################################
# Assemble .s files
####################################
$(OBJDIR)/%.o: system/%.s | $(OBJDIR)
	$(AS) $(ASFLAGS) $< -o $@

####################################
# Create build directory
####################################
$(OBJDIR):
	mkdir -p $(OBJDIR)

####################################
# Flash and debug
####################################
burn: $(BIN)
	st-flash --connect-under-reset write $(BIN) 0x08000000

connect:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
	        -f /usr/share/openocd/scripts/target/stm32f4x.cfg

debug: $(TARGET)
	gdb-multiarch $(TARGET) \
	    -ex "target extended-remote localhost:3333" \
	    -ex "monitor reset halt" \
	    -ex "load" \
	    -ex "break main" \
	    -ex "continue"

####################################
# Clean
####################################
clean:
	rm -rf $(OBJDIR)

.PHONY: all clean burn connect debug
