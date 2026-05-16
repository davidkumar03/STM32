# STM32F405 Baremetal MPU

Cortex-M4 Memory Protection Unit implemented from scratch — no HAL, no CMSIS helper macros, raw register writes against the ARM architecture manual.

Built as a structured self-study project to understand how the Cortex-M4 enforces memory access rules at the hardware level, and how the CPU signals violations back to firmware through its exception model.

---

## What this covers

| Area | Detail |
|---|---|
| MPU region config | 4 regions — Flash, SRAM, Peripherals, PPB |
| Memory type encoding | TEX/C/B/S bits per region — Normal WT, Normal WB+WA, Device Shared, Strongly-Ordered |
| Access permissions | AP field — RO, RW privileged-only, full RW |
| Fault handlers | MemManage, BusFault, UsageFault, HardFault |
| Frame capture | Naked trampoline + EXC_RETURN decode → C handler receives `ExceptionFrame_t*` |
| CFSR decode | Every MMFSR, BFSR, UFSR bit mapped to a string in `g_fault.type` |
| CCR traps | `DIV_0_TRP` enabled, `UNALIGN_TRP` optional |
| Fault tests | 13 test functions — one per fault sub-type, triggerable from `main()` |

---

## MPU region layout

```
Region  Address       Size    Memory type         AP              XN
──────  ────────────  ──────  ──────────────────  ──────────────  ───
  0     0x08000000    1 MB    Normal Write-Through  RO / RO        0   Flash
  1     0x20000000    128 KB  Normal WB+WA S=1      RW / RW        1   SRAM
  2     0x40000000    512 MB  Device Shared          Priv-RW / --  1   Peripherals
  3     0xE0000000    1 MB    Strongly-Ordered       Priv-RW / --  1   PPB
```

**Region 3 (PPB) is mandatory when `PRIVDEFENA=0`.** Without it, fault handlers try to read `SCB->CFSR` at `0xE000ED28` — a second fault fires inside the handler — and the CPU enters Lockup.

---

## Fault test matrix

Uncomment one test function in `main()`, flash, and set a breakpoint in the handler.

### MemManage — `MemManage_Handler_C`

> MM-1, MM-2, MM-3 require `PRIVDEFENA=0` in `MPU->CTRL`. With `PRIVDEFENA=1` those addresses pass the MPU and reach the bus → BusFault instead.

| Function | CFSR | MMFAR | Sub-type |
|---|---|---|---|
| `test_mm_daccviol()` | `0x00000082` | `0xDEAD0000` | DACCVIOL — write to unmapped address |
| `test_mm_daccviol_read()` | `0x00000082` | `0xCCCCCCCC` | DACCVIOL — read from unmapped address |
| `test_mm_write_flash()` | `0x00000082` | `0x08001000` | DACCVIOL — write to read-only Flash region |
| `test_mm_iaccviol()` | `0x00000001` | `frame->pc` | IACCVIOL — execute from XN SRAM |

### BusFault — `BusFault_Handler_C`

> Requires `PRIVDEFENA=1` (default). MPU background map permits the access; the bus rejects it.

| Function | CFSR | BFAR | Sub-type |
|---|---|---|---|
| `test_bf_preciserr()` | `0x00008200` | `0x60000000` | PRECISERR — FMC write + DSB, address valid |
| `test_bf_impreciserr()` | `0x00000400` | invalid | IMPRECISERR — FMC write, no DSB, address lost |
| `test_bf_ibuserr()` | `0x00000100` | `frame->pc` | IBUSERR — fetch past end of Flash |

### HardFault — `HardFault_Handler`

| Function | HFSR | CFSR | How |
|---|---|---|---|
| `test_hf_forced_usage()` | `0x40000000` | `0x02000000` | FORCED — `USGFAULTENA=0` + divide by zero |
| `test_hf_vecttbl()` | `0x00000002` | `0x00000000` | VECTTBL — corrupt VTOR + pend SysTick |

> **`test_hf_vecttbl()` is destructive** — VTOR is left corrupt. Reset the chip immediately after.

### UsageFault — `UsageFault_Handler_C`

| Function | CFSR | Sub-type |
|---|---|---|
| `test_divbyzero()` | `0x02000000` | DIVBYZERO — `SDIV` with 0 denominator |
| `test_nocp()` | `0x00080000` | NOCP — FPU instruction, CP10/CP11 disabled |
| `test_invstate()` | `0x00020000` | INVSTATE — branch to address with Thumb bit cleared |
| `test_undefinstr()` | `0x00010000` | UNDEFINSTR — `UDF #0` undefined opcode |

---

## Reading a fault in the debugger

```
1.  Uncomment one test function in main()
2.  Set breakpoint on the while(1) inside the relevant handler _C function
3.  Add g_fault to Live Expressions (STM32CubeIDE) or Watch (GDB)
4.  Flash and run

g_fault.type     → human-readable fault description
g_fault.fault_pc → address of the instruction that caused the fault
                   look this up in the .map file to find the source line
g_fault.cfsr     → raw CFSR — cross-reference with the bit table below
g_fault.mmfar    → exact bad address (MemManage, when MMARVALID=1)
g_fault.bfar     → exact bad address (BusFault,  when BFARVALID=1)
```

### CFSR bit reference

```
Bits [7:0]   MMFSR — MemManage
  bit 0  IACCVIOL   instruction access violation     MMFAR not updated
  bit 1  DACCVIOL   data access violation            MMFAR valid when bit7=1
  bit 3  MUNSTKERR  fault during exception unstacking
  bit 4  MSTKERR    fault during exception stacking (stack overflow)
  bit 7  MMARVALID  MMFAR holds a valid address

Bits [15:8]  BFSR — BusFault
  bit 8  IBUSERR    instruction fetch bus error
  bit 9  PRECISERR  precise data bus error           BFAR valid when bit15=1
  bit10  IMPRECISERR imprecise (write buffer)         BFAR NOT valid
  bit11  UNSTKERR   bus fault during unstacking
  bit12  STKERR     bus fault during stacking
  bit15  BFARVALID  BFAR holds a valid address

Bits [31:16] UFSR — UsageFault
  bit16  UNDEFINSTR undefined instruction
  bit17  INVSTATE   invalid execution state (ARM mode attempt)
  bit18  INVPC      invalid PC on exception return
  bit19  NOCP       coprocessor / FPU not enabled
  bit24  UNALIGNED  unaligned access  (needs CCR.UNALIGN_TRP=1)
  bit25  DIVBYZERO  divide by zero    (needs CCR.DIV_0_TRP=1)

HFSR
  bit1   VECTTBL    vector table read error
  bit30  FORCED     escalated from configurable fault — check CFSR for root cause
```

---

## Key addresses (no CMSIS)

```c
#define SCB_BASE  0xE000ED00UL   // CPUID, ICSR, VTOR, AIRCR, SCR, CCR,
                                 // SHPR, SHCSR, CFSR, HFSR, DFSR,
                                 // MMFAR, BFAR, AFSR
#define MPU_BASE  0xE000ED90UL   // TYPE, CTRL, RNR, RBAR, RASR
```

---

## PRIVDEFENA — the most important MPU_CTRL bit

```
PRIVDEFENA = 1  (development — default in this project)
  Privileged code uses the ARM default memory map as a background region
  for any address not covered by your configured regions.
  MPU passes the access → bus transaction goes out.
  If the bus rejects it → BusFault.
  Fault handlers always reach SCB registers safely.

PRIVDEFENA = 0  (production / strict)
  No background map. Every address must be explicitly declared.
  MPU rejects any uncovered address → MemManage fault.
  Region 3 (PPB) is non-optional — without it, fault handlers
  cannot read SCB registers → double fault → CPU Lockup.
```

---

## Bugs caught during development

These were real register-level bugs discovered while building this project. Each one is documented because the failure mode teaches something specific about the architecture.

| Bug | Root cause | Consequence | Fix |
|---|---|---|---|
| `SCB_BASE = 0xE000E000` | SCS base, not SCB base | Every fault register (CFSR, MMFAR, HFSR) read from wrong address — garbage values | Change to `0xE000ED00` |
| `DSB/ISB` between disable and configure | Wrong sequence per ARM DDI 0403 B3.5.4 | Race: MPU enable could happen before region writes completed | Move barriers to after all region writes, before and after enable |
| CFSR split into 3 struct members | Misread of register layout | Struct offsets wrong — each member at an incorrect address | Single `CFSR` field at `+0x28`, bit-mask sub-fields in code |
| No TEX/C/B/S bits in RASR | Missing `memType` parameter | All regions Strongly-Ordered — peripheral writes stall CPU on every bus cycle | Add memory type encoding per region |
| `SHCSR_BUSFAULTENA` defined but not written | Typo — define present, register write missing bit 17 | Every BusFault escalated to HardFault, `BusFault_Handler` never ran | Add `\| SHCSR_BUSFAULTENA \| SHCSR_USGFAULTENA` to SHCSR write |
| `SHCRS` vs `SHCSR` | Struct member typo | Write to wrong struct offset | Rename to `SHCSR` |

---

## Hardware and tools

- **MCU:** STM32F405 (Cortex-M4, 168 MHz, 1 MB Flash, 192 KB SRAM)
- **Toolchain:** `arm-none-eabi-gcc`
- **Debug:** ST-Link V2 + STM32CubeIDE debugger
- **No dependencies:** no HAL, no CMSIS

---

## References

- ARM DDI 0403 — ARMv7-M Architecture Reference Manual (§B3.5 MPU, §B1.5 Exception model)
- RM0090 — STM32F405/F407/F415/F417 Reference Manual (§4 Memory mapping, §9 GPIO)
- STM32F405 Datasheet — Table 9: pin alternate function mapping
