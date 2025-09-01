####################################
# Toolchain and flags
####################################
CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE    = arm-none-eabi-size

# Base flags common to C and C++
BASE_FLAGS = -mcpu=cortex-m4 -mthumb -g -Wall -O2 -ffunction-sections -fdata-sections

# C and C++ specific flags
CFLAGS  = $(BASE_FLAGS)
CXXFLAGS = $(BASE_FLAGS) -fno-exceptions -fno-rtti

ASFLAGS = -mcpu=cortex-m4 -mthumb

# Linker script and libraries
LDFLAGS = -T linker.ld -nostartfiles -Wl,--gc-sections -lc -lgcc

####################################
# Directories
####################################
SRC_DIRS     := src system
INCLUDE_DIRS := inc system
INCDIRS      := $(addprefix -I, $(INCLUDE_DIRS))
OBJDIR       := build

####################################
# Source files
####################################
C_SRC   := $(shell find $(SRC_DIRS) -name '*.c')
CPP_SRC := $(shell find $(SRC_DIRS) -name '*.cpp')
ASM_SRC := $(shell find $(SRC_DIRS) -name '*.s')

# Object file names will be placed in OBJDIR (flattened)
OBJ = $(addprefix $(OBJDIR)/,$(notdir $(C_SRC:.c=.o))) \
      $(addprefix $(OBJDIR)/,$(notdir $(CPP_SRC:.cpp=.o))) \
      $(addprefix $(OBJDIR)/,$(notdir $(ASM_SRC:.s=.o)))

####################################
# Targets
####################################
TARGET = $(OBJDIR)/kernel.elf
BIN    = $(OBJDIR)/kernel.bin
LST    = $(OBJDIR)/kernel.lst

all: $(OBJDIR) $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@
	$(OBJCOPY) -O binary $@ $(BIN)
	$(OBJDUMP) -D $@ > $(LST)
	$(SIZE) $@

# Compile .c files
$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCDIRS) -c $< -o $@

$(OBJDIR)/%.o: system/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCDIRS) -c $< -o $@

# Compile .cpp files
$(OBJDIR)/%.o: src/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(INCDIRS) -c $< -o $@

# Assemble .s files
$(OBJDIR)/%.o: system/%.s | $(OBJDIR)
	$(AS) $(ASFLAGS) $< -o $@

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

clean:
	rm -rf $(OBJDIR)

.PHONY: all clean burn connect debug
