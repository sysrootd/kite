CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

TARGET = build/kite
ELF = $(TARGET).elf
BIN = $(TARGET).bin
LST = $(TARGET).lst

SRC_DIRS = core/src app/src app sys/platform sys/driver

C_SRCS := $(sort $(shell find $(SRC_DIRS) -name '*.c'))
S_SRCS := $(sort $(shell find $(SRC_DIRS) -name '*.S'))

SRC := $(C_SRCS) $(S_SRCS)
OBJ := $(patsubst %,build/%.o,$(SRC))

CFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -g3 -Wall -O0 -ffunction-sections -fdata-sections -ffreestanding -nostdlib
INCLUDES = -I./app/ -I./app/inc/ -I./core/inc/ -I./sys/driver/gpio/ -I./sys/driver/adc/ -I./sys/driver/timer/ -I./sys/driver/uart/ -I./sys/platform/
LDFLAGS = -T sys/platform/linker.ld -Wl,--gc-sections -nostdlib

VERBOSE ?= 0
ifeq ($(VERBOSE),1)
Q =
else
Q = @
endif

all: $(ELF) $(BIN) $(LST) size

build/%.c.o: %.c
	$(Q)mkdir -p $(dir $@)
	$(Q)echo "[CC] $<"
	$(Q)$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/%.S.o: %.S
	$(Q)mkdir -p $(dir $@)
	$(Q)echo "[AS] $<"
	$(Q)$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(ELF): $(OBJ)
	$(Q)echo "[LD] $@"
	$(Q)$(CC) $(CFLAGS) $(OBJ) $(LDFLAGS) -o $@

$(BIN): $(ELF)
	$(Q)echo "[BIN] $@"
	$(Q)$(OBJCOPY) -O binary $< $@

$(LST): $(ELF)
	$(Q)echo "[LST] $@"
	$(Q)$(OBJDUMP) -D $< > $@

size: $(ELF)
	$(Q)echo ""
	$(Q)arm-none-eabi-size -A $(ELF) | \
	awk 'NR>2 && ($$1==".isr_vector" || $$1==".text" || $$1==".rodata" || $$1==".data" || $$1==".bss") { \
		printf "%-12s size: %-8d addr: 0x%08X\n", $$1, $$2, $$3; \
		if ($$1==".isr_vector" || $$1==".text" || $$1==".rodata") flash+=$$2; \
		if ($$1==".data" || $$1==".bss") ram+=$$2; \
		total+=$$2; \
	} END { \
		printf "--------------------------------\n"; \
		printf "FLASH total : %d bytes\n", flash; \
		printf "RAM total   : %d bytes\n", ram; \
		printf "TOTAL       : %d bytes\n", total; \
	}'

burn: $(BIN)
	$(Q)echo "[FLASH]"
	$(Q)st-flash --connect-under-reset write $(BIN) 0x08000000

connect:
	$(Q)echo "[OPENOCD]"
	$(Q)openocd -f /usr/share/openocd/scripts/interface/stlink.cfg \
	           -f /usr/share/openocd/scripts/target/stm32f4x.cfg

debug: $(ELF)
	$(Q)echo "[GDB]"
	$(Q)gdb-multiarch $(ELF) \
	    -ex "target extended-remote localhost:3333" \
	    -ex "monitor reset halt" \
	    -ex "load" \
	    -ex "break main" \
	    -ex "continue"

clean:
	$(Q)rm -rf build

.PHONY: burn connect debug