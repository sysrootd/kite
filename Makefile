# Toolchain and flags
CC = arm-none-eabi-g++
AS = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

CFLAGS = -mcpu=cortex-m4 -mthumb -g -Wall -std=c++17 \
         -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti

ASFLAGS = -mcpu=cortex-m4 -mthumb

# Linker script and libraries
LDFLAGS = -T linker.ld -nostartfiles -nodefaultlibs -lc -lgcc

# Directories
SRC_DIRS := src system system/cmsis
INCLUDE_DIRS := include include/drivers system system/cmsis
INCDIRS := $(addprefix -I, $(INCLUDE_DIRS))

OBJDIR := build

# Source files
CPP_SRC := $(shell find $(SRC_DIRS) -name '*.cpp')
ASM_SRC := $(shell find $(SRC_DIRS) \( -name '*.s' -o -name '*.asm' \))

# Object files
CPP_OBJ := $(patsubst %.cpp,$(OBJDIR)/%.o,$(CPP_SRC))
ASM_OBJ := $(patsubst %.s,$(OBJDIR)/%.o,$(filter %.s,$(ASM_SRC)))
ASM_OBJ += $(patsubst %.asm,$(OBJDIR)/%.o,$(filter %.asm,$(ASM_SRC)))
OBJ := $(sort $(CPP_OBJ) $(ASM_OBJ))

TARGET = $(OBJDIR)/kernal.elf
BIN = $(OBJDIR)/kernal.bin
LST = $(OBJDIR)/kernal.lst

all: $(OBJDIR) $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) -o $@
	$(OBJCOPY) -O binary $@ $(BIN)
	$(OBJDUMP) -D $@ > $(LST)

# Compile .cpp files
$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCDIRS) -c $< -o $@

# Assemble .s files
$(OBJDIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) $< -o $@

# Assemble .asm files
$(OBJDIR)/%.o: %.asm
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) $< -o $@

$(OBJDIR):
	mkdir -p $(OBJDIR)

burn: $(BIN)
	sleep 1
	st-flash --connect-under-reset write $(BIN) 0x08000000

connect: $(TARGET)
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
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
