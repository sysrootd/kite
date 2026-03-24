# kite RTOS (STM32F401)

A small RTOS written in C for the STM32F401 family. It provides a simple
scheduler and task APIs making it ideal for learning bare‑metal RTOS
programming on Cortex‑M4 devices.

---

## Features

- Fair & preemptive-priority scheduler 
- Task creation, delay and wake APIs
- Automatic stack allocation from RAM
- SysTick timer for tick interrupts
- GPIO helper functions for STM32
- CMSIS‑compatible register access

---

## Requirements

- STM32F401 series development board
- If you don't have similar series, copy linker script from Stm32cubeide
  or CubeMx and tweak startup file
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- `make` utility
- OpenOCD and ST‑Link hardware
- Cortex‑Debug extension for VS Code for debugging

---

## Building

```sh
# compile everything
make

# clean build products
make clean
```

Compiled binaries appear in `build/`:
- `kernel.elf` – ELF executable
- `kernel.bin` – flashable binary

### Flashing

```sh
openocd -f interface/stlink.cfg \
        -f target/stm32f4x.cfg \
        -c "program build/kernel.elf verify reset exit"
```

### Debugging

1. Launch VS Code, install Cortex‑Debug extension
2. Connect the ST‑Link to your board
3. choose **Run › Start Debugging**

Pre‑configured `.vscode/launch.json` and `tasks.json` are included.

VS Code settings snippet:

```json
{
  "editor.formatOnSave": true,
  "C_Cpp.clang_format_path": "clang-format",
  "C_Cpp.clang_format_style": "LLVM",
  "C_Cpp.codeAnalysis.clangTidy.enabled": true,
  "C_Cpp.codeAnalysis.clangTidy.path": "clang-tidy"
}
```

---

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
