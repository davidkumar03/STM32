# Bare-Metal Bootloader on STM32F405 (Cortex-M4)

This repository contains a **fully bare-metal bootloader and application architecture** implemented on the STM32F405 microcontroller. The project is designed to demonstrate **ARM Cortex-M startup behavior, vector table management, stack initialization, and register-level peripheral configuration** without relying on any frameworks or runtime abstractions.

---

## 1. Project Overview

* MCU: STM32F405 (ARM Cortex-M4)
* Programming model: Bare-metal (direct register access)
* Toolchain: GCC
* Abstractions: None (no frameworks)

The bootloader controls the system from reset and safely transfers execution to an independently linked application image.

---

## 2. Flash Memory Layout

| Region      | Start Address | Size  | Description                |
| ----------- | ------------- | ----- | -------------------------- |
| Bootloader  | 0x08000000    | 64 KB | Own vector table and stack |
| Application | 0x08040000    | —     | Independent vector table   |

Each image is built using a **separate linker script** to ensure memory isolation and deterministic placement.

---

## 3. Cortex-M Boot Flow

1. CPU fetches initial MSP from vector[0]
2. CPU fetches Reset_Handler from vector[1]
3. Bootloader initializes minimal system state
4. Bootloader validates and jumps to application

The application follows the same reset sequence using its own vector table.

---

## 4. Bootloader → Application Jump Sequence

The control transfer strictly follows ARM architectural requirements:

* Disable global interrupts
* Stop SysTick
* Relocate vector table using `SCB->VTOR`
* Load application MSP using MSR instruction
* Branch to application Reset_Handler

This ensures a clean and deterministic transition.

---

## 5. Stack Handling (MSP)

* MSP is owned by the currently executing image
* CONTROL register cannot modify MSP
* MSP is explicitly loaded using `MSR MSP, <value>`

Correct MSP handling is mandatory before branching to the application.

---

## 6. Peripheral Configuration (Register Level)

### USART2

* GPIO: PA2 (TX), PA3 (RX)
* Alternate Function: AF7
* Clock source: HSI (16 MHz)
* Baud rate calculated manually

All GPIO and peripheral registers are configured directly according to the reference manual.

---

## 7. Linker Scripts

Custom linker scripts are used to:

* Place vector tables correctly
* Control FLASH and SRAM regions
* Define stack location explicitly
* Prevent memory overlap

This removes dependency on default startup assumptions.

---

## 8. Repository Structure

```
/bootloader
  ├── linker.ld
  ├── startup.s
  └── main.c

/application
  ├── linker.ld
  ├── startup.s
  └── main.c
```

---

## 9. Build and Flash

* Build using GCC Makefile
* Flash bootloader and application to their respective addresses

(Exact commands may vary depending on toolchain and programmer.)

---

## 10. Learning Outcomes

* ARM Cortex-M reset and startup sequence
* Vector table relocation
* MSP ownership and stack initialization
* Safe bootloader jump mechanics
* Bare-metal UART bring-up
* Deterministic memory control using linker scripts

---

## 11. Future Enhancements

* Application validity checks
* CRC-based firmware verification
* UART-based firmware update
* Dual-image rollback support

---

## Author

Davidkumar
