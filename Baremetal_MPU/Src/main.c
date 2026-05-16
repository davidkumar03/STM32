/**
 ******************************************************************************
 * @file    main.c
 * @author  Baremetal STM32 — MPU Study Project
 * @brief   STM32F405 Cortex-M4 MPU — full fault isolation from raw registers
 *
 * Covers:
 *   - MPU region configuration (TEX/C/B/S memory types, AP, XN, SIZE)
 *   - MemManage, BusFault, UsageFault, HardFault handlers
 *   - Naked trampoline + stacked frame capture (MSP/PSP via EXC_RETURN)
 *   - CFSR/HFSR/MMFAR/BFAR decode into human-readable strings
 *   - CCR trap configuration (DIV_0_TRP, UNALIGN_TRP)
 *   - 12 test functions — one per fault sub-type
 *
 * Reference: ARM DDI 0403 (ARMv7-M ARM), RM0090 (STM32F4xx)
 * Tool chain: arm-none-eabi-gcc — no HAL, no CMSIS
 ******************************************************************************
 */

#include <stdint.h>

/* ===========================================================================
 * MPU register block
 * Base: 0xE000ED90  (ARM DDI 0403 table B3-8)
 * =========================================================================== */
typedef struct
{
    volatile uint32_t TYPE;   /* 0xE000ED90 — number of supported regions       */
    volatile uint32_t CTRL;   /* 0xE000ED94 — ENABLE, HFNMIENA, PRIVDEFENA      */
    volatile uint32_t RNR;    /* 0xE000ED98 — region number select               */
    volatile uint32_t RBAR;   /* 0xE000ED9C — region base address                */
    volatile uint32_t RASR;   /* 0xE000EDA0 — region attributes and size         */
} MPU_t;

/* ===========================================================================
 * System Control Block (SCB)
 * Base: 0xE000ED00  (ARM DDI 0403 table B3-4)
 *
 * NOTE: Base is 0xE000ED00, NOT 0xE000E000 (that is the SCS base).
 *       Getting this wrong means every fault register reads garbage.
 * =========================================================================== */
typedef struct
{
    volatile uint32_t CPUID;    /* +0x00  0xE000ED00 — CPU ID                   */
    volatile uint32_t ICSR;     /* +0x04  0xE000ED04 — interrupt control/state  */
    volatile uint32_t VTOR;     /* +0x08  0xE000ED08 — vector table offset       */
    volatile uint32_t AIRCR;    /* +0x0C  0xE000ED0C — app interrupt/reset ctrl  */
    volatile uint32_t SCR;      /* +0x10  0xE000ED10 — system control            */
    volatile uint32_t CCR;      /* +0x14  0xE000ED14 — config and control        */
    volatile uint32_t SHPR[3];  /* +0x18  0xE000ED18 — system handler priority   */
    volatile uint32_t SHCSR;    /* +0x24  0xE000ED24 — system handler ctrl/stat  */
    volatile uint32_t CFSR;     /* +0x28  0xE000ED28 — composite fault status
                                 *   bits[ 7: 0] = MMFSR  MemManage sub-type
                                 *   bits[15: 8] = BFSR   BusFault sub-type
                                 *   bits[31:16] = UFSR   UsageFault sub-type
                                 *   CFSR is ONE register — not three separate.  */
    volatile uint32_t HFSR;     /* +0x2C  0xE000ED2C — HardFault status          */
    volatile uint32_t DFSR;     /* +0x30  0xE000ED30 — debug fault status        */
    volatile uint32_t MMFAR;    /* +0x34  0xE000ED34 — MemManage fault address
                                 *   valid only when CFSR.MMARVALID = 1          */
    volatile uint32_t BFAR;     /* +0x38  0xE000ED38 — BusFault address
                                 *   valid only when CFSR.BFARVALID = 1          */
    volatile uint32_t AFSR;     /* +0x3C  0xE000ED3C — auxiliary fault status    */
} SCB_t;

#define MPU_BASE   0xE000ED90UL
#define SCB_BASE   0xE000ED00UL
#define MPU        ((MPU_t*)MPU_BASE)
#define SCB        ((SCB_t*)SCB_BASE)

/* ===========================================================================
 * MPU_CTRL register bits
 * =========================================================================== */
#define MPU_CTRL_ENABLE      (1UL << 0)  /* enable the MPU                     */
#define MPU_CTRL_HFNMIENA    (1UL << 1)  /* MPU active during HardFault/NMI    */
#define MPU_CTRL_PRIVDEFENA  (1UL << 2)  /* background map for privileged code  */

/* ===========================================================================
 * MPU_RASR bit positions
 * =========================================================================== */
#define MPU_RASR_ENABLE_Pos    0U   /* bit  0      — region enable             */
#define MPU_RASR_SIZE_Pos      1U   /* bits [5:1]  — region size field         */
#define MPU_RASR_SRD_Pos       8U   /* bits [15:8] — sub-region disable        */
#define MPU_RASR_MEMTYPE_Pos  16U   /* bits [21:16]— B, C, S, TEX[2:0]        */
#define MPU_RASR_AP_Pos       24U   /* bits [26:24]— access permission         */
#define MPU_RASR_XN_Pos       28U   /* bit  28     — execute never             */

/*
 * SIZE field formula:  region_bytes = 2^(SIZE+1)
 *   SIZE=16 → 2^17 = 128 KB     SIZE=19 → 2^20 = 1 MB
 *   SIZE=28 → 2^29 = 512 MB     SIZE=31 → 2^32 = 4 GB
 *
 * Alignment rule: base address low N bits must be 0, where N = SIZE+1
 */

/* ===========================================================================
 * Memory type encoding — packed into RASR[21:16]
 *
 *  Bit layout (6-bit field shifted left by MPU_RASR_MEMTYPE_Pos):
 *    bit5..3 = TEX[2:0]
 *    bit2    = S  (Shareable  — set when DMA also accesses the region)
 *    bit1    = C  (Cacheable)
 *    bit0    = B  (Bufferable)
 *
 *  Reference: ARM DDI 0403 table B3-13
 * =========================================================================== */
#define MEM_STRONGLY_ORDERED  (0x00U)  /* TEX=0 S=0 C=0 B=0 — PPB only        */
#define MEM_DEVICE_SHARED     (0x05U)  /* TEX=0 S=1 C=0 B=1 — peripherals     */
#define MEM_NORMAL_WT         (0x02U)  /* TEX=0 S=0 C=1 B=0 — Flash           */
#define MEM_NORMAL_WB_WA      (0x0FU)  /* TEX=1 S=1 C=1 B=1 — SRAM (WB+WA)   */
#define MEM_NORMAL_NC         (0x0CU)  /* TEX=1 S=1 C=0 B=0 — DMA-only bufs  */

/* ===========================================================================
 * Access permission (AP) — RASR[26:24]
 * Reference: ARM DDI 0403 table B3-15
 * =========================================================================== */
#define AP_NO_ACCESS         0U   /* Privileged: none   Unprivileged: none     */
#define AP_PRIV_RW           1U   /* Privileged: RW     Unprivileged: none     */
#define AP_PRIV_RW_USER_RO   2U   /* Privileged: RW     Unprivileged: RO       */
#define AP_FULL_RW           3U   /* Privileged: RW     Unprivileged: RW       */
#define AP_PRIV_RO           5U   /* Privileged: RO     Unprivileged: none     */
#define AP_READ_ONLY         6U   /* Privileged: RO     Unprivileged: RO       */

/* ===========================================================================
 * CFSR — MMFSR bits [7:0]  (MemManage fault sub-type)
 * =========================================================================== */
#define CFSR_IACCVIOL   (1UL << 0)  /* instruction access violation            */
#define CFSR_DACCVIOL   (1UL << 1)  /* data access violation                   */
#define CFSR_MUNSTKERR  (1UL << 3)  /* fault on exception return (unstacking)  */
#define CFSR_MSTKERR    (1UL << 4)  /* fault on exception entry (stacking)     */
#define CFSR_MLSPERR    (1UL << 5)  /* fault during FP lazy stacking           */
#define CFSR_MMARVALID  (1UL << 7)  /* MMFAR holds a valid fault address       */

/* ===========================================================================
 * CFSR — BFSR bits [15:8]  (BusFault sub-type)
 * =========================================================================== */
#define CFSR_IBUSERR     (1UL << 8)   /* instruction fetch bus error           */
#define CFSR_PRECISERR   (1UL << 9)   /* precise data bus error — BFAR valid   */
#define CFSR_IMPRECISERR (1UL << 10)  /* imprecise — write buffer, BFAR lost   */
#define CFSR_UNSTKERR    (1UL << 11)  /* bus fault during exception unstacking  */
#define CFSR_STKERR      (1UL << 12)  /* bus fault during exception stacking    */
#define CFSR_BFARVALID   (1UL << 15)  /* BFAR holds a valid fault address      */

/* ===========================================================================
 * CFSR — UFSR bits [31:16]  (UsageFault sub-type)
 * =========================================================================== */
#define CFSR_UNDEFINSTR  (1UL << 16)  /* undefined instruction opcode           */
#define CFSR_INVSTATE    (1UL << 17)  /* invalid EPSR state (ARM mode attempt)  */
#define CFSR_INVPC       (1UL << 18)  /* invalid PC on exception return         */
#define CFSR_NOCP        (1UL << 19)  /* coprocessor/FPU access, not enabled    */
#define CFSR_UNALIGNED   (1UL << 24)  /* unaligned access (needs UNALIGN_TRP)   */
#define CFSR_DIVBYZERO   (1UL << 25)  /* integer divide-by-zero (needs DIV_0_TRP)*/

/* ===========================================================================
 * HFSR bits  (HardFault status)
 * =========================================================================== */
#define HFSR_VECTTBL  (1UL << 1)   /* vector table read failed                 */
#define HFSR_FORCED   (1UL << 30)  /* escalated from configurable fault        */

/* ===========================================================================
 * SHCSR — configurable fault handler enable bits
 *
 *  When a bit is 0, that fault class cannot activate its own handler
 *  and escalates directly to HardFault (HFSR.FORCED = 1).
 * =========================================================================== */
#define SHCSR_MEMFAULTENA  (1UL << 16)  /* enable MemManage_Handler  */
#define SHCSR_BUSFAULTENA  (1UL << 17)  /* enable BusFault_Handler   */
#define SHCSR_USGFAULTENA  (1UL << 18)  /* enable UsageFault_Handler */

/* ===========================================================================
 * CCR — Configuration and Control Register  (SCB->CCR @ 0xE000ED14)
 *
 *  DIV_0_TRP   (bit 4):
 *    0 = SDIV/UDIV by zero silently returns 0  (reset default)
 *    1 = SDIV/UDIV by zero → UsageFault DIVBYZERO
 *
 *  UNALIGN_TRP (bit 3):
 *    0 = unaligned word/halfword access split transparently  (reset default)
 *    1 = any unaligned access → UsageFault UNALIGNED
 *        Enable during development; disable once all bugs are fixed.
 * =========================================================================== */
#define CCR_UNALIGN_TRP  (1UL << 3)
#define CCR_DIV_0_TRP    (1UL << 4)

/* ===========================================================================
 * GPIO / RCC — GPIOB, PB14 LED
 * =========================================================================== */
#define PERIPH_BASE   (0x40000000UL)
#define AHB1_BASE     (PERIPH_BASE + 0x20000UL)
#define GPIOB_BASE    (AHB1_BASE   + 0x400UL)
#define RCC_BASE      (AHB1_BASE   + 0x3800UL)

#define RCC_AHB1ENR   (*(volatile uint32_t*)(RCC_BASE   + 0x30UL))
#define GPIOB_MODER   (*(volatile uint32_t*)(GPIOB_BASE + 0x00UL))
#define GPIOB_ODR     (*(volatile uint32_t*)(GPIOB_BASE + 0x14UL))

#define GPIOBEN       (1UL << 1)   /* RCC_AHB1ENR bit 1 — GPIOB clock */
#define LED_PIN       (1UL << 14)  /* PB14 */

/* ===========================================================================
 * Memory barriers
 *
 *  DSB — Data Synchronisation Barrier
 *        Stalls the CPU until all pending memory writes complete.
 *        Required: after MPU region writes, before MPU enable.
 *
 *  ISB — Instruction Synchronisation Barrier
 *        Flushes the CPU pipeline — ensures no stale prefetched instructions
 *        execute after a context-changing operation (MPU enable, VTOR write).
 *        Required: after MPU enable.
 * =========================================================================== */
static inline void DSB(void) { __asm volatile ("dsb" ::: "memory"); }
static inline void ISB(void) { __asm volatile ("isb" ::: "memory"); }

/* ===========================================================================
 * MPU_ConfigRegion
 *
 *  @param region       Region number 0–7 (F405 has 8 regions)
 *  @param baseAddr     Must be aligned to region size:
 *                      base & (2^(sizeField+1) - 1) == 0
 *  @param sizeField    RASR SIZE value; region_bytes = 2^(sizeField+1)
 *  @param memType      One of MEM_* defines (packed TEX/S/C/B value)
 *  @param accessPerm   One of AP_* defines
 *  @param executeNever 1 = XN bit set (instruction fetch generates MemManage)
 * =========================================================================== */
void MPU_ConfigRegion(uint32_t region,
                      uint32_t baseAddr,
                      uint32_t sizeField,
                      uint32_t memType,
                      uint32_t accessPerm,
                      uint32_t executeNever)
{
    MPU->RNR  = region;
    MPU->RBAR = baseAddr;
    MPU->RASR = (1U            << MPU_RASR_ENABLE_Pos)
              | (sizeField     << MPU_RASR_SIZE_Pos)
              | (memType       << MPU_RASR_MEMTYPE_Pos)
              | (accessPerm    << MPU_RASR_AP_Pos)
              | (executeNever  << MPU_RASR_XN_Pos);
}

/* ===========================================================================
 * MPU_Init
 *
 *  Correct sequence per ARM DDI 0403 section B3.5.4:
 *    1.  Disable MPU
 *    2.  Configure all regions
 *    3.  Enable fault handlers in SHCSR and CCR traps
 *    4.  DSB  — flush all pending writes to MPU registers
 *    5.  ISB  — flush CPU pipeline
 *    6.  Enable MPU
 *    7.  DSB + ISB — MPU config visible to all subsequent accesses
 * =========================================================================== */
void MPU_Init(void)
{
    /* 1. Disable MPU before reconfiguring */
    MPU->CTRL = 0;

    /* 2. Configure regions ---------------------------------------------------
     *
     *  Region 0  Flash 1 MB @ 0x08000000
     *    MEM_NORMAL_WT  — Write-Through, no write-allocate (suitable for Flash)
     *    AP_READ_ONLY   — any write attempt → MemManage DACCVIOL
     *    XN=0           — allow instruction fetch (code lives here)
     *    SIZE=19        — 2^20 = 1 MB  ✓ 0x08000000 aligned                   */
    MPU_ConfigRegion(0, 0x08000000U, 19, MEM_NORMAL_WT,       AP_READ_ONLY, 0);

    /*  Region 1  SRAM 128 KB @ 0x20000000
     *    MEM_NORMAL_WB_WA — Write-Back + Write-Allocate, Shareable
     *    S=1              — DMA can also access SRAM → must be Shareable
     *    AP_FULL_RW       — privileged and unprivileged read/write
     *    XN=1             — prevent code execution from RAM (anti-ROP)
     *    SIZE=16          — 2^17 = 128 KB  ✓ 0x20000000 aligned               */
    MPU_ConfigRegion(1, 0x20000000U, 16, MEM_NORMAL_WB_WA,    AP_FULL_RW,   1);

    /*  Region 2  Peripheral space 512 MB @ 0x40000000
     *    MEM_DEVICE_SHARED — buffered, ordered, shareable
     *                        use Device (not Strongly-Ordered) for peripherals:
     *                        writes are buffered (CPU does not stall on every
     *                        register write), but ordering is preserved vs other
     *                        device accesses on the same bus.
     *    AP_PRIV_RW        — privileged only; unprivileged cannot touch hardware
     *    XN=1              — no instruction fetch from peripheral space
     *    SIZE=28           — 2^29 = 512 MB covers APB1, APB2, AHB1, AHB2      */
    MPU_ConfigRegion(2, 0x40000000U, 28, MEM_DEVICE_SHARED,   AP_PRIV_RW,   1);

    /*  Region 3  Private Peripheral Bus (PPB) 1 MB @ 0xE0000000
     *    MEM_STRONGLY_ORDERED — no buffering, no reordering (hardware requirement)
     *                           NVIC, SCB, SysTick, ITM, DWT, FPB
     *                           ARM mandates Strongly-Ordered for the PPB.
     *    AP_PRIV_RW            — privileged only; user code must not access NVIC
     *    XN=1                  — no instruction fetch
     *    SIZE=19               — 2^20 = 1 MB  ✓ 0xE0000000 aligned
     *
     *    CRITICAL: this region is mandatory when PRIVDEFENA=0.
     *    Without it, fault handlers read SCB registers → second fault → Lockup. */
    MPU_ConfigRegion(3, 0xE0000000U, 19, MEM_STRONGLY_ORDERED, AP_PRIV_RW,  1);

    /* 3a. CCR trap configuration
     *    DIV_0_TRP=1    — integer divide by zero → UsageFault instead of
     *                     silent return of 0.  Always safe to enable.
     *    UNALIGN_TRP=0  — leave off by default. Cortex-M4 handles unaligned
     *                     word/halfword accesses transparently (two bus cycles).
     *                     Enable during development to find alignment bugs,
     *                     then comment out before production.                  */
    SCB->CCR |= CCR_DIV_0_TRP;
    /* SCB->CCR |= CCR_UNALIGN_TRP; */

    /* 3b. Enable all three configurable fault handlers.
     *    If any bit is 0, that fault class escalates to HardFault (FORCED=1)
     *    and you lose the specific CFSR sub-type information.                  */
    SCB->SHCSR |= SHCSR_MEMFAULTENA
               |  SHCSR_BUSFAULTENA
               |  SHCSR_USGFAULTENA;

    /* 4+5. Flush write buffer and pipeline */
    DSB();
    ISB();

    /* 6. Enable MPU.
     *
     *    PRIVDEFENA=1 — privileged code falls back to the ARM default memory
     *    map for addresses not covered by regions 0–3. This means privileged
     *    code can still reach uncovered addresses; unprivileged code cannot.
     *    Use PRIVDEFENA=0 for strict mode: every accessible address must be
     *    explicitly declared in a region (fault handlers need Region 3 PPB).
     *
     *    HFNMIENA=0  — HardFault/NMI handlers bypass the MPU (development
     *    setting). Set HFNMIENA=1 in production once MPU config is verified
     *    and all handler stacks are covered by accessible regions.             */
    MPU->CTRL = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;

    /* 7. MPU active from this point forward */
    DSB();
    ISB();
}

/* ===========================================================================
 * Fault information structure
 *
 *  Filled by every fault handler. Declare volatile so the compiler never
 *  optimises out the writes — this variable must always be inspectable in
 *  the debugger regardless of optimisation level.
 * =========================================================================== */
typedef struct
{
    uint32_t    cfsr;       /* raw CFSR — MMFSR[7:0] BFSR[15:8] UFSR[31:16] */
    uint32_t    hfsr;       /* raw HFSR                                      */
    uint32_t    mmfar;      /* faulting address when MMARVALID=1             */
    uint32_t    bfar;       /* faulting address when BFARVALID=1             */
    uint32_t    fault_pc;   /* PC of the instruction that caused the fault   */
    uint32_t    fault_lr;   /* LR (EXC_RETURN) at point of fault             */
    const char *type;       /* human-readable fault description              */
} FaultInfo_t;

volatile FaultInfo_t g_fault;   /* global — always reachable via debugger */

/* Hardware-stacked register frame (pushed automatically on exception entry) */
typedef struct
{
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
} ExceptionFrame_t;

/* ===========================================================================
 * MemManage fault handler
 *
 *  Naked trampoline: reads EXC_RETURN bit 2 to determine which stack
 *  (MSP = Handler/privileged, PSP = Thread/task) held the frame,
 *  then passes the frame pointer to the C handler as the first argument (r0).
 * =========================================================================== */
void MemManage_Handler_C(ExceptionFrame_t *frame)
{
    uint32_t cfsr    = SCB->CFSR;
    g_fault.cfsr     = cfsr;
    g_fault.hfsr     = SCB->HFSR;
    g_fault.fault_pc = frame->pc;
    g_fault.fault_lr = frame->lr;

    if (cfsr & CFSR_IACCVIOL)
    {
        /* Instruction fetch from a region with XN=1 or AP=no-access.
         * MMFAR is NOT updated for IACCVIOL — fault_pc is the bad address. */
        g_fault.type  = "MemManage: instruction access violation (IACCVIOL)";
        g_fault.mmfar = frame->pc;
        g_fault.bfar  = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_DACCVIOL)
    {
        /* Data load/store to an address denied by MPU access permissions.
         * MMFAR is valid when MMARVALID=1 — read it before it can be
         * overwritten by a nested exception. */
        g_fault.type  = "MemManage: data access violation (DACCVIOL)";
        g_fault.mmfar = (cfsr & CFSR_MMARVALID) ? SCB->MMFAR : 0xFFFFFFFFU;
        g_fault.bfar  = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_MUNSTKERR)
    {
        /* MPU fault while restoring stacked registers on exception return.
         * The exception stack itself violates an MPU region. */
        g_fault.type  = "MemManage: unstacking error (MUNSTKERR)";
        g_fault.mmfar = 0xFFFFFFFFU;
        g_fault.bfar  = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_MSTKERR)
    {
        /* MPU fault while pushing registers on exception entry.
         * Stack pointer is in a no-access region — likely stack overflow. */
        g_fault.type  = "MemManage: stacking error (MSTKERR)";
        g_fault.mmfar = 0xFFFFFFFFU;
        g_fault.bfar  = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_MLSPERR)
    {
        /* MPU fault during FP register lazy stacking. */
        g_fault.type  = "MemManage: FP lazy stacking error (MLSPERR)";
        g_fault.mmfar = 0xFFFFFFFFU;
        g_fault.bfar  = 0xFFFFFFFFU;
    }
    else
    {
        g_fault.type  = "MemManage: unknown";
        g_fault.mmfar = 0xFFFFFFFFU;
        g_fault.bfar  = 0xFFFFFFFFU;
    }

    SCB->CFSR = cfsr;   /* write 1 to clear — otherwise fault fires again */
    while(1);           /* breakpoint here, inspect g_fault               */
}

__attribute__((naked))
void MemManage_Handler(void)
{
    __asm volatile (
        "tst   lr, #4              \n"   /* EXC_RETURN bit2: 0=used MSP, 1=used PSP */
        "ite   eq                  \n"
        "mrseq r0, msp             \n"   /* r0 → frame on MSP */
        "mrsne r0, psp             \n"   /* r0 → frame on PSP */
        "b     MemManage_Handler_C \n"
    );
}

/* ===========================================================================
 * BusFault handler
 *
 *  BusFault fires when the MPU has permitted the access but the AHB bus
 *  itself returns an error (SLVERR or DECERR from a peripheral or invalid
 *  address). PRECISERR gives a valid BFAR; IMPRECISERR does not — add a
 *  DSB() after the suspect write to force a synchronous (precise) fault.
 * =========================================================================== */
void BusFault_Handler_C(ExceptionFrame_t *frame)
{
    uint32_t cfsr    = SCB->CFSR;
    g_fault.cfsr     = cfsr;
    g_fault.hfsr     = SCB->HFSR;
    g_fault.fault_pc = frame->pc;
    g_fault.fault_lr = frame->lr;
    g_fault.mmfar    = 0xFFFFFFFFU;

    if (cfsr & CFSR_IBUSERR)
    {
        /* Instruction fetch returned a bus error.
         * fault_pc = the address the CPU tried to fetch from. */
        g_fault.type = "BusFault: instruction bus error (IBUSERR)";
        g_fault.bfar = frame->pc;
    }
    else if (cfsr & CFSR_PRECISERR)
    {
        /* Synchronous data access bus error — CPU stalled for the response.
         * BFAR holds the exact faulting address when BFARVALID=1.
         * Read BFAR before returning; a nested exception may invalidate it. */
        g_fault.type = "BusFault: precise data bus error (PRECISERR)";
        g_fault.bfar = (cfsr & CFSR_BFARVALID) ? SCB->BFAR : 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_IMPRECISERR)
    {
        /* Asynchronous write buffer error — CPU had already moved on.
         * BFAR is NOT valid; fault_pc points to a later instruction.
         * To convert to PRECISERR: add DSB() after the suspect write,
         * or map the region as Device (non-bufferable) in the MPU. */
        g_fault.type = "BusFault: imprecise data bus error — add DSB() (IMPRECISERR)";
        g_fault.bfar = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_UNSTKERR)
    {
        /* Bus error while restoring stacked registers on exception return. */
        g_fault.type = "BusFault: unstacking error (UNSTKERR)";
        g_fault.bfar = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_STKERR)
    {
        /* Bus error while pushing registers on exception entry. */
        g_fault.type = "BusFault: stacking error (STKERR)";
        g_fault.bfar = 0xFFFFFFFFU;
    }
    else
    {
        g_fault.type = "BusFault: unknown";
        g_fault.bfar = 0xFFFFFFFFU;
    }

    SCB->CFSR = cfsr;
    while(1);
}

__attribute__((naked))
void BusFault_Handler(void)
{
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "b     BusFault_Handler_C  \n"
    );
}

/* ===========================================================================
 * UsageFault handler
 *
 *  CFSR bits [31:16] = UFSR sub-register.
 *  fault_pc = exact instruction that caused the fault in all sub-types.
 *  No dedicated address register — use fault_pc to locate the bad instruction.
 * =========================================================================== */
void UsageFault_Handler_C(ExceptionFrame_t *frame)
{
    uint32_t cfsr    = SCB->CFSR;
    g_fault.cfsr     = cfsr;
    g_fault.hfsr     = SCB->HFSR;
    g_fault.fault_pc = frame->pc;
    g_fault.fault_lr = frame->lr;
    g_fault.mmfar    = 0xFFFFFFFFU;
    g_fault.bfar     = 0xFFFFFFFFU;

    if (cfsr & CFSR_DIVBYZERO)
    {
        /* SDIV or UDIV with denominator = 0.
         * Only traps when CCR.DIV_0_TRP = 1 (set in MPU_Init).
         * Without that bit, result is 0 and execution continues silently.
         * fault_pc = address of the SDIV/UDIV instruction. */
        g_fault.type = "UsageFault: divide by zero (DIVBYZERO)";
    }
    else if (cfsr & CFSR_UNALIGNED)
    {
        /* Unaligned word or halfword memory access.
         * Only traps when CCR.UNALIGN_TRP = 1.
         * By default Cortex-M4 handles unaligned accesses transparently.
         * fault_pc = the LDR/STR instruction with the misaligned address. */
        g_fault.type = "UsageFault: unaligned memory access (UNALIGNED)";
    }
    else if (cfsr & CFSR_NOCP)
    {
        /* FPU or other coprocessor instruction when CP10/CP11 not enabled.
         * Fix: SCB->CPACR |= (0xF << 20) in Reset_Handler before main().
         * fault_pc = the VLDR/VADD/VMUL/etc. FPU instruction. */
        g_fault.type = "UsageFault: coprocessor/FPU not enabled — set CPACR (NOCP)";
    }
    else if (cfsr & CFSR_INVPC)
    {
        /* Invalid EXC_RETURN value loaded into PC on exception return.
         * Indicates a corrupted or tampered LR in the exception stack frame.
         * fault_pc = the bad EXC_RETURN value attempted. */
        g_fault.type = "UsageFault: invalid PC on exception return (INVPC)";
    }
    else if (cfsr & CFSR_INVSTATE)
    {
        /* CPU tried to execute in ARM state (EPSR.T = 0).
         * Cortex-M4 is Thumb-only — EPSR.T must always be 1.
         * Most common cause: function pointer with LSB cleared (no Thumb bit).
         * fault_pc = the BX/BLX that attempted the ARM-mode switch. */
        g_fault.type = "UsageFault: invalid execution state — check Thumb bit (INVSTATE)";
    }
    else if (cfsr & CFSR_UNDEFINSTR)
    {
        /* CPU fetched an opcode it does not recognise.
         * Examples: ARMv8 instructions, compiler bug, intentional UDF.
         * fault_pc = address of the undefined opcode in Flash. */
        g_fault.type = "UsageFault: undefined instruction (UNDEFINSTR)";
    }
    else
    {
        g_fault.type = "UsageFault: unknown UFSR bits";
    }

    SCB->CFSR = cfsr;   /* write 1 to clear — prevents immediate re-entry */
    while(1);
}

__attribute__((naked))
void UsageFault_Handler(void)
{
    __asm volatile (
        "tst   lr, #4               \n"
        "ite   eq                   \n"
        "mrseq r0, msp              \n"
        "mrsne r0, psp              \n"
        "b     UsageFault_Handler_C \n"
    );
}

/* ===========================================================================
 * HardFault handler
 *
 *  HardFault fires when:
 *   a) A configurable fault escalates (its enable bit in SHCSR = 0).
 *      HFSR.FORCED = 1. Check CFSR for the root cause — the sub-type
 *      bits are still set even though HardFault ran instead.
 *   b) A fault fires inside a fault handler (double fault).
 *      HFSR.FORCED = 1. With HFNMIENA=0 (our setting), HardFault
 *      handler bypasses the MPU and can always reach SCB registers.
 *   c) The vector table itself is unreadable. HFSR.VECTTBL = 1.
 *
 *  This handler does NOT use a naked trampoline intentionally — it
 *  runs in privileged Handler mode with HFNMIENA=0, so the MPU is
 *  bypassed and direct SCB register access is always safe.
 * =========================================================================== */
void HardFault_Handler(void)
{
    volatile uint32_t hfsr = SCB->HFSR;
    volatile uint32_t cfsr = SCB->CFSR;

    g_fault.hfsr = hfsr;
    g_fault.cfsr = cfsr;
    g_fault.mmfar = (cfsr & CFSR_MMARVALID) ? SCB->MMFAR : 0xFFFFFFFFU;
    g_fault.bfar  = (cfsr & CFSR_BFARVALID) ? SCB->BFAR  : 0xFFFFFFFFU;

    if (hfsr & HFSR_FORCED)
    {
        /* A lower-priority configurable fault escalated.
         * Inspect g_fault.cfsr for the real root cause:
         *   cfsr & 0x000000FF → MemManage sub-type
         *   cfsr & 0x0000FF00 → BusFault sub-type
         *   cfsr & 0xFFFF0000 → UsageFault sub-type              */
        g_fault.type = "HardFault: escalated from configurable fault — check CFSR";
    }
    else if (hfsr & HFSR_VECTTBL)
    {
        /* Bus error occurred while reading the vector table.
         * VTOR is pointing to an invalid or uninitialized address.
         * CFSR is not updated for VECTTBL.                        */
        g_fault.type = "HardFault: vector table read error (VECTTBL)";
    }
    else
    {
        g_fault.type = "HardFault: unknown";
    }

    while(1);   /* breakpoint here — inspect g_fault, hfsr, cfsr */
}

/* ===========================================================================
 * Fault test functions
 *
 *  One function per fault sub-type.
 *  Uncomment ONE call in main() at a time.
 *  Set a breakpoint inside the corresponding handler _C function.
 *  Add g_fault to Live Expressions / Watch window in the debugger.
 *
 * ── MemManage ──────────────────────────── handler: MemManage_Handler_C
 *   MM-1, MM-2, MM-3 need PRIVDEFENA=0 in MPU->CTRL (change MPU_Init).
 *   With PRIVDEFENA=1 those addresses pass the MPU and reach the bus.
 *
 * ── BusFault ───────────────────────────── handler: BusFault_Handler_C
 *   Needs PRIVDEFENA=1 (default) so MPU background map permits the access.
 *
 * ── HardFault ──────────────────────────── handler: HardFault_Handler
 *   HF-1: disable specific handler → fault escalates to HardFault.
 *   HF-2: corrupt VTOR → vector table read fails → VECTTBL.
 *         WARNING: chip is unrecoverable until reset after HF-2.
 *
 * ── UsageFault ─────────────────────────── handler: UsageFault_Handler_C
 * =========================================================================== */

/* --- MemManage tests --- */

/* MM-1: DACCVIOL — write to unmapped address (PRIVDEFENA=0 required)
 * CFSR=0x00000082  MMFAR=0xDEAD0000 */
void test_mm_daccviol(void)
{
    volatile uint32_t *bad = (volatile uint32_t*)0xDEAD0000;
    *bad = 0xBEEF;
}

/* MM-2: DACCVIOL — read from unmapped address (PRIVDEFENA=0 required)
 * CFSR=0x00000082  MMFAR=0xCCCCCCCC */
void test_mm_daccviol_read(void)
{
    volatile uint32_t *bad = (volatile uint32_t*)0xCCCCCCCC;
    volatile uint32_t v = *bad;
    (void)v;
}

/* MM-3: DACCVIOL — write to read-only Flash region (any PRIVDEFENA)
 * CFSR=0x00000082  MMFAR=0x08001000 */
void test_mm_write_flash(void)
{
    volatile uint32_t *flash = (volatile uint32_t*)0x08001000;
    *flash = 0x12345678;
}

/* MM-4: IACCVIOL — execute from SRAM (XN=1 on Region 1, any PRIVDEFENA)
 * CFSR=0x00000001  fault_pc=RAM address  MMARVALID=0 (no MMFAR for IACCVIOL) */
void test_mm_iaccviol(void)
{
    static uint16_t ram_code[2];
    ram_code[0] = 0xBF00;   /* NOP   */
    ram_code[1] = 0x4770;   /* BX LR */
    DSB();
    void (*fn)(void) = (void(*)(void))(((uint32_t)ram_code) | 1U);
    fn();   /* fetch from XN SRAM → IACCVIOL */
}

/* --- BusFault tests --- */

/* BF-1: PRECISERR — write to unconfigured FMC external RAM + DSB
 * CFSR=0x00008200  BFAR=0x60000000 */
void test_bf_preciserr(void)
{
    (*(volatile uint32_t*)(0x60000000)) = 5;
    DSB();   /* drain write buffer → synchronous bus error → PRECISERR */
}

/* BF-2: IMPRECISERR — same write without DSB (write buffer masks address)
 * CFSR=0x00000400  BFAR=invalid  fault_pc=a later instruction */
void test_bf_impreciserr(void)
{
    (*(volatile uint32_t*)(0x60000000)) = 5;
    /* no DSB — error returns asynchronously → IMPRECISERR */
}

/* BF-3: IBUSERR — fetch instruction from beyond end of Flash
 * CFSR=0x00000100  fault_pc=0x08100000 */
void test_bf_ibuserr(void)
{
    void (*bad)(void) = (void(*)(void))0x08100001;   /* past 1 MB Flash + Thumb bit */
    bad();
}

/* --- HardFault tests --- */

/* HF-1: FORCED — disable UsageFault handler, trigger divide-by-zero
 * HFSR=0x40000000  CFSR=0x02000000 (root cause still in CFSR) */
void test_hf_forced_usage(void)
{
    SCB->SHCSR &= ~SHCSR_USGFAULTENA;
    DSB(); ISB();
    volatile int32_t a = 1, b = 0;
    volatile int32_t r = a / b;
    (void)r;
}

/* HF-2: VECTTBL — point VTOR to invalid RAM, pend SysTick
 * HFSR=0x00000002  CFSR=0x00000000
 * WARNING: chip unrecoverable after this — reset required */
void test_hf_vecttbl(void)
{
    SCB->VTOR  = 0x2001F000;
    DSB(); ISB();
    SCB->ICSR |= (1UL << 26);   /* PENDSTSET — pend SysTick */
    DSB(); ISB();
}

/* --- UsageFault tests --- */

/* UF-1: DIVBYZERO — requires CCR.DIV_0_TRP=1 (set in MPU_Init)
 * CFSR=0x02000000 */
void test_divbyzero(void)
{
    volatile int32_t a = 42, b = 0;
    volatile int32_t r = a / b;
    (void)r;
}

/* UF-2: NOCP — FPU instruction with CP10/CP11 disabled
 * CFSR=0x00080000 */
void test_nocp(void)
{
    *((volatile uint32_t*)0xE000ED88) &= ~(0xFUL << 20);   /* disable FPU */
    DSB(); ISB();
    volatile float a = 1.5f, b = 2.5f;
    volatile float c = a + b;   /* VADD.F32 → NOCP */
    (void)c;
}

/* UF-3: INVSTATE — branch to address with Thumb bit cleared (ARM mode)
 * CFSR=0x00020000 */
void test_invstate(void)
{
    uint32_t addr = (uint32_t)test_divbyzero & ~1U;   /* strip Thumb bit */
    void (*bad)(void) = (void(*)(void))addr;
    bad();
}

/* UF-4: UNDEFINSTR — official permanently-undefined Thumb opcode
 * CFSR=0x00010000 */
void test_undefinstr(void)
{
    __asm volatile ("udf #0");
}

/* ===========================================================================
 * main
 * =========================================================================== */
int main(void)
{
    MPU_Init();

    RCC_AHB1ENR  |= GPIOBEN;
    GPIOB_MODER  |=  (1U << 28);   /* PB14 output */
    GPIOB_MODER  &= ~(1U << 29);

    /* =========================================================================
     * Test menu — ONE call at a time.
     *
     * Debugger work flow:
     *   1. Set breakpoint inside the relevant handler _C function
     *   2. Add g_fault to Live Expressions / Watch
     *   3. Flash and run
     *   4. Read g_fault.type, g_fault.fault_pc, g_fault.mmfar / g_fault.bfar
     *   5. Cross-reference fault_pc in the .map file to find the source line
     *
     * MemManage  (PRIVDEFENA=0 for MM-1/2/3) ─── MemManage_Handler_C
     *   test_mm_daccviol()       CFSR=0x00000082  MMFAR=0xDEAD0000
     *   test_mm_daccviol_read()  CFSR=0x00000082  MMFAR=0xCCCCCCCC
     *   test_mm_write_flash()    CFSR=0x00000082  MMFAR=0x08001000
     *   test_mm_iaccviol()       CFSR=0x00000001  fault_pc=RAM addr
     *
     * BusFault   (PRIVDEFENA=1) ─────────────── BusFault_Handler_C
     *   test_bf_preciserr()      CFSR=0x00008200  BFAR=0x60000000
     *   test_bf_impreciserr()    CFSR=0x00000400  BFAR=invalid
     *   test_bf_ibuserr()        CFSR=0x00000100  fault_pc=0x08100000
     *
     * HardFault ──────────────────────────────── HardFault_Handler
     *   test_hf_forced_usage()   HFSR=0x40000000  CFSR=0x02000000
     *   test_hf_vecttbl()        HFSR=0x00000002  CFSR=0x00000000  ← reset after
     *
     * UsageFault ─────────────────────────────── UsageFault_Handler_C
     *   test_divbyzero()         CFSR=0x02000000
     *   test_nocp()              CFSR=0x00080000
     *   test_invstate()          CFSR=0x00020000
     *   test_undefinstr()        CFSR=0x00010000
     * ========================================================================= */

    /* --- active test --- */
    test_divbyzero();

    /* MemManage */
    /* test_mm_daccviol();        */
    /* test_mm_daccviol_read();   */
    /* test_mm_write_flash();     */
    /* test_mm_iaccviol();        */

    /* BusFault */
    /* test_bf_preciserr();       */
    /* test_bf_impreciserr();     */
    /* test_bf_ibuserr();         */

    /* HardFault */
    /* test_hf_forced_usage();    */
    /* test_hf_vecttbl();         */

    /* UsageFault */
    /* test_nocp();               */
    /* test_invstate();           */
    /* test_undefinstr();         */

    for (;;)
    {
        GPIOB_ODR ^= LED_PIN;
        for (volatile int i = 0; i < 100000; i++);
    }

    return 0;
}
