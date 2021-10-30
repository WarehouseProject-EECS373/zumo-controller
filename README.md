# Zumo Controller

Integrated code for deployment on Zumo robots. Let's keep component-level testing in other repositories or other branches.

## Project Architecture

The Zumo Controller is built on top of the Realtime Micro Kernel (`rmkernel`), a realtime, event-driven, run-to-completion operating systen/scheduler. `rmkernel` follows the [Active Object (AO)](https://www.state-machine.com/active-object) design pattern to eliminate typical concurrency issues like deadlock and race conditions.
With AOs, it's really easy to diagram and plan systems using UML Activity diagrams. Each AO can be viewed as a state machine where it takes a message as an input, changes state based on the message and returns. There should be ***no waiting, no blocking, no delays*** in any of the AOs. To create a delay, put the AO in a waiting state and then schedule a one-shot timed message to arrive _n_ milliseconds on the future.

To be implemented, a placeholder for the future (a rough outline)

- Drive Subsystem
  - Gets Sensor readings and updates setpoints
  - Manages PWM
- Path Tracking Subsystem
  - Reads from sensor every _n_ ms and sends data to Drive Subsystem
- Box Interface Subsystem
  - Controls electromagnet
- Communications Subsystem
  - Realtime data logging/capture
  - Wireless comms
- Central Control
  - Manages main internal state (idle, driving, picking up, etc.)
  - Tells Box Interface to engage

## Creating AOs

- File naming: `<name>_ao.c/.h`
- Add `<name>_ao.c` to `CMakeLists.txt`
- AO source file should contain:
  - All related interrupts
  - An Event Handler
  - Static declarations
  - Any peripheral init/start functions
- AO header file should only contain
  - EventHandler function
  - Peripheral Init/Start functions
- Required messages defined in `app_defs.h`
- Required defines, message ids, etc. in `app_defs.h`
- Use `ACTIVE_OBJECT_EXTERN` macro to create an extern AO declaration in `app_defs.h`
- Use `ACTIVE_OBJECT_DECL` macro in `main.c` to declare
- Use `AO_INIT` macro in `main()` to initialize

## Peripheral Mapping

[Nucleo-L4R5ZI Pinout](https://os.mbed.com/platforms/NUCLEO-L4R5ZI/)

Which peripherals are in use on which pins

### Timers

- TIM2
  - dedicated to `rmkernel` and `HAL`
- TIM3
  - Motor PWM
  - 2 output channels, PB4 and PB6

## Directory Structure

```bash
$ tree -L 2 # with modification
.
├── CMakeLists.txt # root CMake file. Source files are added here
├── L4R5ZI.ld # default linker file, taken from stm32-cmake template
├── LICENSE
├── Makefile # high level automation
├── README.md 
├── debug # output directory of make build
│   ├── _deps # stm32-cmake will generate lots of files under this directory
│   ├── zumo-controller.bin # we need to flash this
│   ├── zumo-controller.elf # also generating .elf
│   └── zumo-controller.hex
├── modules
│   └── stm32-cmake # see README/Requirements for notes on installation
├── os
│   ├── inc # rmkernel headers
│   └── src # rmkernel sources
├── src
│   ├── app_defs.h # definitions related to the entire application
│   ├── main.c
│   ├── os_port_arm_m4.c # OS port to ARM Cortex-M4
│   ├── rmk_hal_clock_cfg.c # clock configuration
│   ├── rmk_hal_clock_cfg.h 
│   ├── startup_stm32l4r5xx.s # don't touch, default template startup script
│   ├── stm32l4r5xx.h # don't touch default HAL device header
│   ├── stm32l4xx.h # don't touch, default HAL family header
│   ├── stm32l4xx_hal_conf.h # don't touch, default configuration
│   ├── system_stm32l4xx.c # don't touch, default
│   └── system_stm32l4xx.h # don't touch, default
└── st_nucleo_l4.cfg # openocd configuration for debugging
```

## Requirements

- `STLink` - Open source flashing utility
  - <https://github.com/stlink-org/stlink>
  - May also be able to flash device using `.elf` and STM32CubeIDE
- `CMake` (3.16+)
  - <https://cmake.org/install/>
- `ObKo/stm32-cmake`

  ```bash
  git submodule add https://github.com/ObKo/stm32-cmake.git modules/stm32-cmake
  ```
  
  Instead,
  ```bash
  git checkout develop
  git submodule update --init --recursive
  sudo apt install gcc-arm-none-eabi
  export STM32_TOOLCHAIN_PATH=/usr/bin/arm-none-eabi-gcc
  
  Download the latest release (V1.7.0 on the right) from here: https://github.com/stlink-org/stlink/releases/tag/v1.7.0 into Program Files (x86).
  In Command Prompt, navigate to the bin files in this directory: cd <path> (C:\Program Files (x86)\stlink-1.7.0-x86_64-w64-mingw32\bin)
  
  
  Then, run this command: st-flash.exe write \\wsl$\Ubuntu\home\aashishhari\373\zumo-controller 0x08000000
  Note the path name is based on your wsl path, use explorer.exe . and copy paste from the top search bar.
 
  ```

## Installation

1. Install CMake and STLink (see above)
2. `git clone https://github.com/WarehouseProject-EECS373/zumo-controller.git`
3. `git submodule add https://github.com/ObKo/stm32-cmake.git modules/stm32-cmake`
4. All set, `make debug` to build, `make flash` to program device

## Usage

Uses high level `Makefile` to automate build and flash.

### Building

Use `make debug` to to build and flash (includes all debug symbols). We won't be building anything for production so don't worry about `make build`.
If VSCode starts complaining about missing headers etc., you can try running `make build` to create a `compile_commands.json` file that VSCode can use.
Not exactly sure what's going on but this has worked for me in the past.

### Flashing

Use `make flash`. Not sure if configuration changes on Windows. If you're going to change anything in the Makefile, make a new target (e.g. `win-flash`) instead of just modifying the existing one.

## Examples

Lots of examples can be found in <https://github.com/rkalnins/rmkernel-dev>

- UART
- PWM
- DAC
- GPIO
- Active Objects
- Memory Pools/Blocks
