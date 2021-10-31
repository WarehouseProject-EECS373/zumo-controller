# Zumo Controller

Integrated code for deployment on Zumo robots. Let's keep component-level testing in other repositories or other branches.

## Resources and Reading

### Realtime Operating Systems (RTOS) and Active Objects

- [Modern Embedded Programming: Beyond the RTOS](https://embeddedgurus.com/state-space/category/state-machines/)
  - Very very good article on the "why" of using an event drive/state machine based RTOS
- [Economics 101: UML in Embedded Systems](https://embeddedgurus.com/state-space/2012/04/economics-101-uml-in-embedded-systems/)
- [Active Object (AO) Design Pattern for Concurrency](https://www.state-machine.com/active-object)
- [Application Note Active Objects for Embedded Systems](http://www.state-machine.com/doc/AN_Active_Objects_for_Embedded.pdf)
- [Active Objext - Embedded Artistry](https://embeddedartistry.com/fieldmanual-terms/active-object/)
  - Collection of resources (some are the same as the ones linked above)

### CMake

Some examples and reading on CMake and how it works. We're doing it a bit differently per `stm32-cmake` instructions

- [Basic project setup](https://dornerworks.com/blog/moving-embedded-projects-to-cmake/)
- [CMake Part 1 - The Dark Arts](https://blog.feabhas.com/2021/07/cmake-part-1-the-dark-arts/)
  - Read the other parts of the series. This blog is good
- [stm32-cmake](https://github.com/ObKo/stm32-cmake)
  - check out some of the examples in `stm32-cmake/examples` directory

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

- Add source (`.c`) file to `PROJECT_SOURCES` in `CMakeLists.txt`
- Clock configuration should go in `rmk_hal_clock_cfg.c`
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

## Drive Control

### Line Following

Periodic event to read from reflectance sensors and update actual position in drive subsystem.
Periodic task to recalculate control loop.

## Peripheral Mapping

[Nucleo-L4R5ZI Pinout](https://os.mbed.com/platforms/NUCLEO-L4R5ZI/)

Which peripherals are in use on which pins

### Kernel

- TIM2
  - dedicated to `rmkernel` and `HAL`

### Motor Control

- TIM3 PWM Output
  - PB4 -> D9 (right)
  - PB5 -> D10 (left)
- GPIO Direction Control
  - PA3 -> D7 (right)
  - PA4 -> D8 (left)

## Directory Structure

```bash
$ tree -L 2 # with modification
.
├── CMakeLists.txt # root CMake file. Source files are added here
├── L4R5ZI.ld # default linker file, taken from stm32-cmake template
├── LICENSE
├── Makefile # high level automation
├── README.md
├── build
├── debug # output directory of `make debug`
│   ├── zumo-controller.bin # binary file for flashing
│   ├── zumo-controller.elf # .elf for debugging
├── docs
├── gdb # debugging scripts
├── modules
│   └── stm32-cmake # HAL libraries as CMake project, see README/Requirements for notes on installation
├── os
│   ├── inc # rmkernel headers
│   └── src # rmkernel sources
├── src
│   ├── app_defs.h # definitions needed in the entire application
│   ├── main.c
│   ├── os_port_arm_m4.c # OS port to ARM Cortex-M4
│   ├── rmk_hal_clock_cfg.c # system and peripheral clock configuration
│   ├── rmk_hal_clock_cfg.h
│   ├── stm # don't touch, default from STM32 libraries
│   ├── stm32l4xx_hal_conf.h # don't touch, default from STM32 libraries
│   ├── subsys
│   ├── watchdog.c
│   └── watchdog.h
└── st_nucleo_l4.cfg # openocd configuration for debugging
```

## Requirements

- `STLink` - Open source flashing utility
  - <https://github.com/stlink-org/stlink>
  - May also be able to flash device using `.elf` and STM32CubeIDE
  - `sudo apt install stlink-tools`
  - For Windows, download the latest release (V1.7.0 on the right) from here: <https://github.com/stlink-org/stlink/releases/tag/v1.7.0> into Program Files (x86).
    - `cd <path> (C:\Program Files (x86)\stlink-1.7.0-x86_64-w64-mingw32\bin)`
    - `st-flash.exe write <path/to/bin/file> 0x08000000`
- `CMake` (3.16+)
  - <https://cmake.org/install/>
  - `sudo apt install cmake`
- `ObKo/stm32-cmake`
  After cloning `zume-controller`, run `git submodule update --init --recursive`
- GCC
  - `sudo apt install gcc-arm-none-eabi`
  - `export STM32_TOOLCHAIN_PATH=/usr/bin/arm-none-eabi-gcc`

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
