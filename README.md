# kite RTOS (STM32F4XX)

A small RTOS written in C for the STM32F4XX family. It provides a simple
scheduler and task APIs making it ideal for learning bare‑metal RTOS
programming on Cortex‑M4 devices.

---

## Features

- Fair & preemptive-priority scheduler 
- Task creation, delay and wake APIs
- SysTick timer for tick interrupts
- GPIO helper functions for STM32
- CMSIS‑compatible register access

---

## Requirements

- STM32 development board
- Copy the STM32CubeIDE/CubeMX-generated `linker.ld` and `startup.S` into the `sys/` directory
- Update `sys/stm32f4xx.h` for your specific STM32 MCU / board
- Tweak `sys/config.h` for system configuration(<<<<important>>>>)
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- `make`
- OpenOCD
- ST-Link hardware / tools
- VS Code with the Cortex-Debug extension (optional, for debugging)

## Platform files need to change as per your mcu

Platform-specific support files live in `sys/`:

- `sys/startup.S` — copy from STM32CubeIDE/CubeMX
- `sys/linker.ld` — copy from STM32CubeIDE/CubeMX
- `sys/stm32f4xx.h` — update for your MCU and peripheral addresses

## Tool Chain Install

### Ubuntu / Debian
```bash
sudo apt update
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi gdb-multiarch make openocd stlink-tools
```

---

## Building

```sh
# compile everything
make

# clean build products
make clean

# falsh elf to target
make burn
```

Compiled binaries appear in `build/`:
- `kite.elf` – ELF executable
- `kite.bin` – flashable binary

### TUI Debugging

```sh
#In one terminal(It will start openocd debugging service)
make connect

#In another terminal(It will start debug session)
make debug
```

### GUI Debugging

1. Launch VS Code, install Cortex‑Debug extension
2. Connect the ST‑Link to your board
3. choose **Run › Start Debugging**

---

## License

This project is licensed under the **MIT License**.

Copyright © 2025

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
