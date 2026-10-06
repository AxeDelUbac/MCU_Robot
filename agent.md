# MCU Robot — Agent Instructions

Use these instructions when changing this repository. Check the linked project documentation for hardware and workflow details before making assumptions.

## 1. Project and language

- This is embedded firmware for the **ST NUCLEO-G0B1RE** (STM32G0B1, Cortex-M0+).
- Write application code in **C**. The build uses **CMake**, **west**, and **Ninja**.
- The firmware runs on **Zephyr RTOS** and uses multiple Zephyr threads.
- The configured compiler is **`arm-zephyr-eabi-gcc` from the Zephyr SDK**. Use the SDK and build configuration selected for this project; do not switch compilers or toolchains.
- The root [`README.md`](README.md) links to the guides in [`docs/`](docs/README.md). Read the relevant guide before changing hardware, build, or micro-ROS behavior.

## 2. Before editing

1. Inspect the relevant source files, the board overlay, `prj.conf`, and the existing build configuration.
2. Check whether the repository already has a helper, driver, test, or pattern for the change.
3. Make the smallest complete change that addresses the request. Preserve unrelated code and existing local changes.
4. Do not guess board wiring, pin assignments, sensor capabilities, voltage levels, or timing. Verify them in the overlay or project documentation; mark anything not verified.

## 3. C and Zephyr rules

- Match the style and naming of nearby C code. Keep functions focused and make units, valid ranges, and ownership clear.
- Check return values and handle errors explicitly. Validate input received from sensors, UART, or ROS before using it.
- Describe board hardware in Devicetree and overlays; use Kconfig and `prj.conf` for Zephyr features. Prefer Zephyr APIs and drivers over direct register access.
- Check Zephyr APIs, Devicetree bindings, and Kconfig options against the version used by this project.
- Do not block in interrupt context. Keep interrupt handlers short.
- Protect data shared between threads with the appropriate Zephyr synchronization. Preserve existing mutex and message-queue behavior when changing shared state.
- Consider thread priorities, stack sizes, timing, and memory use. Keep actuator outputs safe on startup, invalid commands, communication loss, and faults.

## 4. Build, test, and flash

Build the configured NUCLEO target from the repository root:

```sh
west build -b nucleo_g0b1re/stm32g0b1xx -d build .
```

For an already configured build directory:

```sh
west build -d build
```

- Use the project's configured Zephyr workspace, SDK, and build environment. See [`docs/build-and-architecture.md`](docs/build-and-architecture.md) and [`docs/micro-ros.md`](docs/micro-ros.md) for setup details.
- Run the smallest relevant build or test checks after code changes. First check the repository for applicable tests; do not claim tests exist or passed unless they were run.
- **Do not flash the board or command robot hardware unless the user explicitly asks.** If asked, confirm the target and physical safety conditions first.
- In your final response, state exactly which checks ran. Distinguish build results from tests, flashing, and on-device verification.

## 5. Documentation

- Write project documentation in clear, concise **English** under [`docs/`](docs/README.md).
- Keep the root README brief; add links there when creating a documentation page.
- Give each page one clear topic. Prefer short sections, lists, and steps over long paragraphs.
- Update the relevant guide when a change affects setup, hardware, build commands, interfaces, safety, or user-visible behavior.
- Keep documented commands and hardware details consistent with the code. Label assumptions and unverified behavior; never describe them as tested.
- Avoid repeating detailed instructions across pages. Link to the existing guide instead.

## 6. Completion report

Briefly summarize what changed, the checks actually performed and their results, and any remaining assumptions or unverified behavior.


## 7. Application architecture and code organization

- Keep Zephyr task entry points and thread declarations in `src/RTOS.c`, with their prototypes in `src/RTOS.h`. A reader should be able to understand the task list and each task's high-level responsibility from these files.
- Keep protocol, sensor, navigation, and control implementations in focused modules under `src/`. A task entry point may delegate to a module function; do not move an entire subsystem into `RTOS.c` just to keep the task visible.
- Separate board support and hardware-specific code from robot application behavior. Put pin, peripheral, and device configuration in Devicetree/board files and isolate hardware access behind module interfaces where practical. Application logic should not depend directly on board pins or MCU registers; this makes later porting easier.
- Give each software module one clear responsibility and a small, explicit interface. Before creating a new module or changing module boundaries, check whether an existing module should own the functionality.
- Use `snake_case` for local variables and function parameters. Preserve established public naming conventions when changing APIs; do not rename unrelated symbols as part of a localized change.
- Put declarations of public functions, shared types, and constants in the owning module's `.h` file. Put implementations in its `.c` file. Each `.c` file should include its own header and the headers for the interfaces it uses. Never include one `.c` file from another.

## 8. Architecture decisions

- For a small change that fits the existing module boundaries, follow the current architecture and proceed without interrupting the user.
- Before implementing a change that introduces a subsystem or dependency, changes module boundaries, or materially changes a public interface or task architecture, first explain the proposed structure and responsibilities and wait for the user's approval.



unused files should be added to the .gitignorre files such as build repository, log files or or ide context files.