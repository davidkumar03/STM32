# STM32F405 Baremetal MPU — Cortex-M4 Memory Protection from Scratch

> Zero HAL. Zero CMSIS helpers. Raw register writes, ARM architecture manual, and a debugger.

A complete baremetal implementation of the Cortex-M4 **Memory Protection Unit** on STM32F405, built as a learning project to understand how ARM processors actually enforce memory access rules at the hardware level.

---

## What this project covers

| Topic | What was built |
|---|---|
| MPU region configuration | 4 regions with correct TEX/C/B/S memory type encoding |
| Memory types | Normal WT (Flash), Normal WB+WA (SRAM), Device Shared (peripherals), Strongly-Ordered (PPB) |
| Fault handlers | MemManage, BusFault, UsageFault, HardFault — all with naked trampolines |
| CFSR decode | Every bit of MMFSR, BFSR, UFSR decoded into human-readable strings |
| Stacked frame capture | Naked trampoline → captures MSP/PSP → passes `ExceptionFrame_t*` to C handler |
| Fault address capture | MMFAR (MemManage) and BFAR (BusFault) with VALID bit checking |
| CCR traps | `DIV_0_TRP` and `UNALIGN_TRP` enable/disable with documented rationale |
| 15 test functions | One per fault sub-type — trigger any fault on demand |

---

## Memory regions configured

```
Region 0  Flash        0x08000000  1 MB    Normal WT       RO  XN=0
Region 1  SRAM1+2      0x20000000  128 KB  Normal WB+WA    RW  XN=1  S=1
Region 2  Peripherals  0x40000000  512 MB  Device Shared   RW  XN=1
Region 3  PPB          0xE0000000  1 MB    Strongly-Ordered RW XN=1
```

---

## Fault test matrix

Every fault sub-type has a dedicated test function. Uncomment one in `main()`, flash, set a breakpoint in the handler, and inspect `g_fault` in the debugger.

### MemManage faults — `MemManage_Handler_C`

| Function | CFSR | MMFAR | Trigger |
|---|---|---|---|
| `test_mm_daccviol()` | `0x00000082` | `0xDEAD0000` | Write to unmapped address |
| `test_mm_daccviol_read()` | `0x00000082` | `0xCCCCCCCC` | Read from unmapped address |
| `test_mm_write_flash()` | `0x00000082` | Flash addr | Write to read-only Flash region |

> MM-1, MM-2, MM-3 require `PRIVDEFENA=0`. See notes in `MPU_Init()`.

### BusFault faults — `BusFault_Handler_C`

| Function | CFSR | BFAR | Trigger |
|---|---|---|---|
| `test_bf_preciserr()` | `0x00008200` | `0x60000000` | Write to unconfigured FMC + DSB |
| `test_bf_impreciserr()` | `0x00000400` | invalid | Write to FMC, no DSB — buffered error |
| `test_bf_ibuserr()` | `0x00000100` | `frame->pc` | Fetch past end of Flash |
| `test_bf_noclk_periph()` | `0x00008200` | `0x50060804` | Read RNG register, clock off |

### HardFault — `HardFault_Handler`

| Function | HFSR | CFSR | Trigger |
|---|---|---|---|
| `test_hf_forced_usage()` | `0x40000000` | `0x02000000` | USGFAULTENA=0, divide by zero |
| `test_hf_forced_bus()` | `0x40000000` | `0x00008200` | BUSFAULTENA=0, FMC access |
| `test_hf_vecttbl()` | `0x00000002` | `0x00000000` | Corrupt VTOR, pend SysTick |

> `test_hf_vecttbl()` leaves the chip unrecoverable until reset. Run it last.

### UsageFault — `UsageFault_Handler_C`

| Function | CFSR | Trigger |
|---|---|---|
| `test_divbyzero()` | `0x02000000` | `SDIV` with denominator 0 |
| `test_unaligned()` | `0x01000000` | Unaligned word read (needs `UNALIGN_TRP=1`) |
| `test_nocp()` | `0x00080000` | Float instruction, FPU disabled |
| `test_invstate()` | `0x00020000` | Branch to ARM-mode address (LSB=0) |
| `test_undefinstr()` | `0x00010000` | `UDF #0` undefined opcode |

---

## Key register addresses (no CMSIS)

```c
#define SCB_BASE   0xE000ED00UL   // CPUID, ICSR, VTOR, AIRCR, CCR, SHCSR,
                                  // CFSR, HFSR, DFSR, MMFAR, BFAR, AFSR
#define MPU_BASE   0xE000ED90UL   // TYPE, CTRL, RNR, RBAR, RASR
```

---

## Bugs found and fixed during development

These were real mistakes caught while building this — documented here because each one teaches something important.

| # | Bug | Impact | Fix |
|---|---|---|---|
| 1 | `SCB_BASE = 0xE000E000` instead of `0xE000ED00` | Fault handler read garbage — CFSR and MMFAR at wrong addresses | Correct base address |
| 2 | `DSB/ISB` placed between disable and configure, not after | Race condition on MPU enable — config writes not flushed | Move barriers to after all region writes, before and after enable |
| 3 | `CFSR` split into three struct members (MMSR, BFSR, UFSR) | Wrong memory layout — each field at incorrect offset | Single `CFSR` field at `+0x28`, extract sub-fields by bit masking |
| 4 | Missing TEX/C/B/S bits in RASR | All regions Strongly-Ordered — peripheral writes stall CPU | Add `memType` parameter, encode per-region |
| 5 | `SHCSR_BUSFAULTENA` defined but never written | BusFault always escalated to HardFault — BusFault handler never ran | Add `| SHCSR_BUSFAULTENA | SHCSR_USGFAULTENA` to SHCSR write |
| 6 | `SHCRS` typo (was `SHCSR`) | Silent wrong offset in struct | Rename to `SHCSR` |

---

## How to read a fault in the debugger

1. Set breakpoint inside `MemManage_Handler_C`, `BusFault_Handler_C`, or `UsageFault_Handler_C`
2. Add `g_fault` to Live Expressions
3. Read `g_fault.type` for the fault description
4. Read `g_fault.fault_pc` — find this address in the `.map` file to identify the source line
5. Read `g_fault.mmfar` or `g_fault.bfar` for the bad memory address
6. Read `g_fault.cfsr` raw value and cross-reference the bit table below

```
CFSR[7:0]   = MMFSR  — MemManage sub-type
CFSR[15:8]  = BFSR   — BusFault sub-type
CFSR[31:16] = UFSR   — UsageFault sub-type

HFSR[30]    = FORCED  — escalated from lower fault, check CFSR for root cause
HFSR[1]     = VECTTBL — vector table read failed
```

---

## PRIVDEFENA — the most important MPU_CTRL bit

```
PRIVDEFENA = 1  (development)
  Privileged code falls back to ARM default memory map for unmapped addresses.
  MPU says YES → access reaches the bus → BusFault if bus rejects it.
  Fault handlers always reachable. Safe to develop with.

PRIVDEFENA = 0  (production / strict)
  No background map. Every accessible address must be explicitly declared.
  MPU says NO → MemManage fault before the bus is even contacted.
  If you forget Region 3 (PPB), your fault handler reads SCB → second fault → Lockup.
```

---

## References

- ARM DDI 0403 — ARMv7-M Architecture Reference Manual (B3.5 MPU, B1.5 Exceptions)
- RM0090 — STM32F405/F407/F415/F417 Reference Manual
- STM32F405 Datasheet — Table 9: alternate function mapping

---

## Project structure

```
main.c          — all source: MPU init, handlers, test functions
README.md       — this file
```

---

## Hardware

- Board: STM32F405 (tested on custom board, compatible with Nucleo-F446RE pinout)
- Toolchain: GCC ARM (`arm-none-eabi-gcc`)
- Debug: ST-Link V2 + OpenOCD / STM32CubeIDE debugger
- No HAL, no CMSIS, no vendor startup files — linker script and startup written from scratch
