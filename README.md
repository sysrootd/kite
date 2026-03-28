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

- STM32F401 series development board
- If you don't have a similar STM32F4 board, copy the linker script from
  STM32CubeIDE or CubeMX and adjust the startup file as needed
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- `make`
- OpenOCD
- ST-Link hardware / tools
- VS Code with the Cortex-Debug extension (optional, for debugging)

## Install / Download Commands

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
```

Compiled binaries appear in `build/`:
- `kite.elf` – ELF executable
- `kite.bin` – flashable binary

### Flashing

```sh
openocd -f interface/stlink.cfg \
        -f target/stm32f4x.cfg \
        -c "program build/kite.elf verify reset exit"
```

### Debugging

1. Launch VS Code, install Cortex‑Debug extension
2. Connect the ST‑Link to your board
3. choose **Run › Start Debugging**

## VS Code Configuration Files

- `.vscode/tasks.json` – build/clean tasks
- `.vscode/launch.json` – debug configuration for your mcu
- `.vscode/settings.json` – toolchain and file association settings

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
