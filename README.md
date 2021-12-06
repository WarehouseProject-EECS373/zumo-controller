# Zumo Controller

Integrated code for deployment on Zumo robots.

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

A rough outline:

- Drive Subsystem
  - Gets Sensor readings and updates setpoints
  - Manages PWM timers
- Reflectance Array Subsystem
  - Reads from sensor every _n_ ms and sends data to Drive Subsystem
- Electromagnet Subsystem
  - Controls electromagnet
- Communications Subsystem
  - Realtime data logging/capture
  - Wireless comms
- State Controller
  - Manages main internal state (idle, driving, picking up, etc.)

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

PID control with integrator bounds, output bounding, and support for deadband. Conrol loop runs at 200Hz

### Line Following

200Hz periodic event to read from reflectance sensors and update actual position in drive subsystem.
This implementation is non-blocking and works well with the event-driven RTOS

[Reflectance sensor array info/pinout](https://www.pololu.com/docs/0J57/2.c) 
[Arduino sensor array code](https://pololu.github.io/zumo-32u4-arduino-library/class_q_t_r_sensors.html)

## Peripheral Mapping

[Nucleo-L4R5ZI Pinout](https://os.mbed.com/platforms/NUCLEO-L4R5ZI/)

Which peripherals are in use on which pins

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
├── tools # code formatting scripts, others
├── modules
│   ├── rmkernel # RTOS
│   ├── stcp # serial transport library, headers, footers, escape sequences
│   └── stm32-cmake # HAL libraries as CMake project, see README/Requirements for notes on installation
├── src
│   ├── app_defs.h # definitions needed in the entire application
│   ├── main.c
│   ├── os_port_arm_m4.c # RMKernel port to ARM Cortex-M4
│   ├── rmk_hal_clock_cfg.c # system and peripheral clock configuration
│   ├── rmk_hal_clock_cfg.h
│   ├── stm # default from STM32 libraries
│   ├── stm32l4xx_hal_conf.h # default from STM32 libraries
│   ├── subsys # subsystems
│   ├── cmd # commands
│   ├── trace.c # tracing callbacks
│   ├── trace.h
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
  After cloning `zumo-controller`, run `git submodule update --init --recursive`
- `arm-none-eabi-gcc`
  - `sudo apt install gcc-arm-none-eabi`
  - `export STM32_TOOLCHAIN_PATH=/usr/bin/arm-none-eabi-gcc`

## Usage

Uses high level `Makefile` to automate build flash, format.

```bash
# build for release
make build

# build with debug symnbols
make debug

# flash using st-link
make flash

# apply clang-format to source
make format
```
