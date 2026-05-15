/**
 ******************************************************************************
 * @file    main.c
 * @brief   STM32F405 Baremetal — MPU + MemManage + BusFault + UsageFault
 *
 * UsageFault additions in this file:
 *  [UF-1] CFSR_UFSR bit masks — UNDEFINSTR, INVSTATE, INVPC,
 *          NOCP, UNALIGNED, DIVBYZERO
 *  [UF-2] CCR register — DIV_0_TRP and UNALIGN_TRP enable bits
 *  [UF-3] UsageFault_Handler naked trampoline + C handler
 *  [UF-4] SCB->CCR configured in MPU_Init() before enabling MPU
 *  [UF-5] Five test functions — one per UsageFault cause
 ******************************************************************************
 */

#include <stdint.h>

/* ============================================================
 * MPU register block — base 0xE000ED90
 * ============================================================ */
typedef struct
{
    volatile uint32_t TYPE;
    volatile uint32_t CTRL;
    volatile uint32_t RNR;
    volatile uint32_t RBAR;
    volatile uint32_t RASR;
} MPU_t;

/* ============================================================
 * SCB register block — base 0xE000ED00
 * ============================================================ */
typedef struct
{
    volatile uint32_t CPUID;    /* +0x00 */
    volatile uint32_t ICSR;     /* +0x04 */
    volatile uint32_t VTOR;     /* +0x08 */
    volatile uint32_t AIRCR;    /* +0x0C */
    volatile uint32_t SCR;      /* +0x10 */
    volatile uint32_t CCR;      /* +0x14 — Configuration and Control Register */
    volatile uint32_t SHPR[3];  /* +0x18 */
    volatile uint32_t SHCSR;    /* +0x24 */
    volatile uint32_t CFSR;     /* +0x28 — MMFSR[7:0] BFSR[15:8] UFSR[31:16] */
    volatile uint32_t HFSR;     /* +0x2C */
    volatile uint32_t DFSR;     /* +0x30 */
    volatile uint32_t MMFAR;    /* +0x34 */
    volatile uint32_t BFAR;     /* +0x38 */
    volatile uint32_t AFSR;     /* +0x3C */
} SCB_t;

#define MPU_BASE  0xE000ED90UL
#define SCB_BASE  0xE000ED00UL
#define MPU  ((MPU_t*)MPU_BASE)
#define SCB  ((SCB_t*)SCB_BASE)

/* ============================================================
 * MPU_CTRL bits
 * ============================================================ */
#define MPU_CTRL_ENABLE           (1UL << 0)
#define MPU_CTRL_HFNMIENA         (1UL << 1)
#define MPU_CTRL_PRIVDEFENA       (1UL << 2)

/* ============================================================
 * MPU_RASR bit positions
 * ============================================================ */
#define MPU_RASR_ENABLE_Pos       0U
#define MPU_RASR_SIZE_Pos         1U
#define MPU_RASR_SRD_Pos          8U
#define MPU_RASR_MEMTYPE_Pos     16U
#define MPU_RASR_AP_Pos          24U
#define MPU_RASR_XN_Pos          28U

/* ============================================================
 * Memory type encoding — RASR[21:16]
 * ============================================================ */
#define MEM_STRONGLY_ORDERED   (0x00U)
#define MEM_DEVICE_SHARED      (0x05U)
#define MEM_NORMAL_WT          (0x02U)
#define MEM_NORMAL_WB_WA       (0x0FU)
#define MEM_NORMAL_NC          (0x0CU)

/* ============================================================
 * Access permission — RASR[26:24]
 * ============================================================ */
#define AP_NO_ACCESS             0U
#define AP_PRIV_RW               1U
#define AP_PRIV_RW_USER_RO       2U
#define AP_FULL_RW               3U
#define AP_PRIV_RO               5U
#define AP_READ_ONLY             6U

/* ============================================================
 * CFSR — MMFSR bits [7:0]
 * ============================================================ */
#define CFSR_IACCVIOL    (1UL << 0)
#define CFSR_DACCVIOL    (1UL << 1)
#define CFSR_MUNSTKERR   (1UL << 3)
#define CFSR_MSTKERR     (1UL << 4)
#define CFSR_MLSPERR     (1UL << 5)
#define CFSR_MMARVALID   (1UL << 7)

/* ============================================================
 * CFSR — BFSR bits [15:8]
 * ============================================================ */
#define CFSR_IBUSERR     (1UL << 8)
#define CFSR_PRECISERR   (1UL << 9)
#define CFSR_IMPRECISERR (1UL << 10)
#define CFSR_UNSTKERR    (1UL << 11)
#define CFSR_STKERR      (1UL << 12)
#define CFSR_BFARVALID   (1UL << 15)

/* ============================================================
 * [UF-1] CFSR — UFSR bits [31:16]
 *
 *  UNDEFINSTR — CPU executed an opcode it does not recognise
 *  INVSTATE   — EPSR.T=0 (ARM mode attempted) or invalid IT field
 *               Most common: function pointer LSB cleared (no Thumb bit)
 *  INVPC      — Bad EXC_RETURN value on exception return (LR corrupted)
 *  NOCP       — FPU / coprocessor instruction when CP10/CP11 not enabled
 *               Fix: SCB->CPACR |= (0xF << 20) before using float
 *  UNALIGNED  — Unaligned word/halfword access — only traps when
 *               CCR.UNALIGN_TRP = 1 (default = 0, silent fixup)
 *  DIVBYZERO  — Integer divide by zero — only traps when
 *               CCR.DIV_0_TRP = 1 (default = 0, returns 0 silently)
 * ============================================================ */
#define CFSR_UNDEFINSTR  (1UL << 16)
#define CFSR_INVSTATE    (1UL << 17)
#define CFSR_INVPC       (1UL << 18)
#define CFSR_NOCP        (1UL << 19)
#define CFSR_UNALIGNED   (1UL << 24)
#define CFSR_DIVBYZERO   (1UL << 25)

/* ============================================================
 * HFSR bits
 * ============================================================ */
#define HFSR_FORCED      (1UL << 30)
#define HFSR_VECTTBL     (1UL << 1)

/* ============================================================
 * SHCSR — fault handler enable bits
 * ============================================================ */
#define SHCSR_MEMFAULTENA (1UL << 16)
#define SHCSR_BUSFAULTENA (1UL << 17)
#define SHCSR_USGFAULTENA (1UL << 18)

/* ============================================================
 * [UF-2] CCR — Configuration and Control Register
 *
 *  DIV_0_TRP   bit 4:
 *    0 = SDIV/UDIV by zero returns 0, no fault  (reset default)
 *    1 = SDIV/UDIV by zero → UsageFault DIVBYZERO
 *
 *  UNALIGN_TRP bit 3:
 *    0 = unaligned word/halfword access handled transparently (reset default)
 *    1 = any unaligned access → UsageFault UNALIGNED
 *        Note: unaligned halfword in IT block always faults regardless.
 * ============================================================ */
#define CCR_UNALIGN_TRP  (1UL << 3)
#define CCR_DIV_0_TRP    (1UL << 4)

/* ============================================================
 * GPIO / RCC
 * ============================================================ */
#define BASE              (0x40000000UL)
#define AHB_BUS           (BASE + 0x20000UL)
#define GPIOB             (AHB_BUS + 0x400UL)
#define RCC               (AHB_BUS + 0x3800UL)
#define AHB_ENABLE        (*(volatile uint32_t*)(RCC   + 0x30UL))
#define GPIOB_MODER       (*(volatile uint32_t*)(GPIOB + 0x00UL))
#define GPIOB_ODR         (*(volatile uint32_t*)(GPIOB + 0x14UL))
#define GPIOBEN           (1U << 1)
#define LED_PIN           (1U << 14)

/* ============================================================
 * Barriers
 * ============================================================ */
static inline void DSB(void) { __asm volatile ("dsb" ::: "memory"); }
static inline void ISB(void) { __asm volatile ("isb" ::: "memory"); }

/* ============================================================
 * MPU_ConfigRegion
 * ============================================================ */
void MPU_ConfigRegion(uint32_t region,
                      uint32_t baseAddr,
                      uint32_t sizeField,
                      uint32_t memType,
                      uint32_t accessPerm,
                      uint32_t executeNever)
{
    MPU->RNR  = region;
    MPU->RBAR = baseAddr;
    MPU->RASR = (1U           << MPU_RASR_ENABLE_Pos)
              | (sizeField    << MPU_RASR_SIZE_Pos)
              | (memType      << MPU_RASR_MEMTYPE_Pos)
              | (accessPerm   << MPU_RASR_AP_Pos)
              | (executeNever << MPU_RASR_XN_Pos);
}

/* ============================================================
 * MPU_Init
 * ============================================================ */
void MPU_Init(void)
{
    MPU->CTRL = 0;

    MPU_ConfigRegion(0, 0x08000000U, 19, MEM_NORMAL_WT,        AP_READ_ONLY, 0);
    MPU_ConfigRegion(1, 0x20000000U, 16, MEM_NORMAL_WB_WA,     AP_FULL_RW,   1);
    MPU_ConfigRegion(2, 0x40000000U, 28, MEM_DEVICE_SHARED,    AP_PRIV_RW,   1);
    MPU_ConfigRegion(3, 0xE0000000U, 19, MEM_STRONGLY_ORDERED, AP_PRIV_RW,   1);

    /* [UF-4] Enable fault traps in CCR.
     *
     *  DIV_0_TRP=1   — integer divide by zero → UsageFault (not silent 0).
     *                  Always safe to enable. No code should divide by zero.
     *
     *  UNALIGN_TRP=0 — left off by default. Cortex-M4 handles unaligned
     *                  accesses transparently at a small performance cost.
     *                  Enable during development to find alignment bugs:
     *                  uncomment the line below, fix all UNALIGNED faults,
     *                  then comment it back out for production.            */
    SCB->CCR |= CCR_DIV_0_TRP;
    /* SCB->CCR |= CCR_UNALIGN_TRP; */   /* uncomment to trap unaligned  */

    /* [UF-4] Enable all three configurable fault handlers in SHCSR.
     *  If any one is 0, that fault class escalates to HardFault.          */
    SCB->SHCSR |= SHCSR_MEMFAULTENA
               |  SHCSR_BUSFAULTENA
               |  SHCSR_USGFAULTENA;

    DSB();
    ISB();

    MPU->CTRL = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;

    DSB();
    ISB();
}

/* ============================================================
 * Fault info — filled by all three handlers
 * ============================================================ */
typedef struct
{
    uint32_t    cfsr;
    uint32_t    hfsr;
    uint32_t    mmfar;
    uint32_t    bfar;
    uint32_t    fault_pc;
    uint32_t    fault_lr;
    const char *type;
} FaultInfo_t;

volatile FaultInfo_t g_fault;

typedef struct
{
    uint32_t r0, r1, r2, r3, r12, lr, pc, xpsr;
} ExceptionFrame_t;

/* ============================================================
 * MemManage handler
 * ============================================================ */
void MemManage_Handler_C(ExceptionFrame_t *frame)
{
    uint32_t cfsr    = SCB->CFSR;
    g_fault.cfsr     = cfsr;
    g_fault.hfsr     = SCB->HFSR;
    g_fault.fault_pc = frame->pc;
    g_fault.fault_lr = frame->lr;

    if (cfsr & CFSR_IACCVIOL)
    {
        g_fault.type  = "MemManage: instruction access violation";
        g_fault.mmfar = frame->pc;
    }
    else if (cfsr & CFSR_DACCVIOL)
    {
        g_fault.type  = "MemManage: data access violation";
        g_fault.mmfar = (cfsr & CFSR_MMARVALID) ? SCB->MMFAR : 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_MUNSTKERR)
    {
        g_fault.type  = "MemManage: unstacking error";
        g_fault.mmfar = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_MSTKERR)
    {
        g_fault.type  = "MemManage: stacking error";
        g_fault.mmfar = 0xFFFFFFFFU;
    }
    else
    {
        g_fault.type  = "MemManage: unknown";
        g_fault.mmfar = 0xFFFFFFFFU;
    }

    SCB->CFSR = cfsr;
    while(1);
}

__attribute__((naked))
void MemManage_Handler(void)
{
    __asm volatile (
        "tst   lr, #4              \n"
        "ite   eq                  \n"
        "mrseq r0, msp             \n"
        "mrsne r0, psp             \n"
        "b     MemManage_Handler_C \n"
    );
}

/* ============================================================
 * BusFault handler
 * ============================================================ */
void BusFault_Handler_C(ExceptionFrame_t *frame)
{
    uint32_t cfsr    = SCB->CFSR;
    g_fault.cfsr     = cfsr;
    g_fault.hfsr     = SCB->HFSR;
    g_fault.fault_pc = frame->pc;
    g_fault.fault_lr = frame->lr;

    if (cfsr & CFSR_IBUSERR)
    {
        g_fault.type = "BusFault: instruction bus error";
        g_fault.bfar = frame->pc;
    }
    else if (cfsr & CFSR_PRECISERR)
    {
        g_fault.type = "BusFault: precise data bus error";
        g_fault.bfar = (cfsr & CFSR_BFARVALID) ? SCB->BFAR : 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_IMPRECISERR)
    {
        g_fault.type = "BusFault: imprecise — add DSB or use Device region";
        g_fault.bfar = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_UNSTKERR)
    {
        g_fault.type = "BusFault: unstacking error";
        g_fault.bfar = 0xFFFFFFFFU;
    }
    else if (cfsr & CFSR_STKERR)
    {
        g_fault.type = "BusFault: stacking error";
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

/* ============================================================
 * [UF-3] UsageFault handler
 *
 *  CFSR bits [31:16] = UFSR sub-register.
 *  fault_pc = the exact instruction that caused the fault.
 *  There is NO address register for UsageFault (no MMFAR/BFAR
 *  equivalent) — the faulting address is always fault_pc itself.
 * ============================================================ */
void UsageFault_Handler_C(ExceptionFrame_t *frame)
{
    uint32_t cfsr    = SCB->CFSR;
    g_fault.cfsr     = cfsr;
    g_fault.hfsr     = SCB->HFSR;
    g_fault.fault_pc = frame->pc;   /* instruction that caused the fault */
    g_fault.fault_lr = frame->lr;
    g_fault.mmfar    = 0xFFFFFFFFU; /* not applicable for UsageFault     */
    g_fault.bfar     = 0xFFFFFFFFU;

    if (cfsr & CFSR_DIVBYZERO)
    {
        /* SDIV or UDIV instruction with denominator = 0.
         * Requires CCR.DIV_0_TRP = 1 to trap (set in MPU_Init).
         * fault_pc = address of the SDIV/UDIV instruction.
         * CFSR raw = 0x02000000 (bit 25).                          */
        g_fault.type = "UsageFault: divide by zero (DIVBYZERO)";
    }
    else if (cfsr & CFSR_UNALIGNED)
    {
        /* Unaligned word/halfword memory access.
         * Requires CCR.UNALIGN_TRP = 1 to trap.
         * fault_pc = the LDR/STR with the misaligned address.
         * CFSR raw = 0x01000000 (bit 24).                          */
        g_fault.type = "UsageFault: unaligned memory access (UNALIGNED)";
    }
    else if (cfsr & CFSR_NOCP)
    {
        /* FPU instruction when CP10/CP11 not enabled in CPACR.
         * Fix: SCB->CPACR |= (0xF << 20) in Reset_Handler.
         * fault_pc = the VLDR/VADD/VMUL/etc instruction.
         * CFSR raw = 0x00080000 (bit 19).                          */
        g_fault.type = "UsageFault: FPU disabled — set CPACR CP10/CP11 (NOCP)";
    }
    else if (cfsr & CFSR_INVPC)
    {
        /* Bad EXC_RETURN value on exception return.
         * Happens when stacked LR is corrupted or tampered with.
         * fault_pc = the bad EXC_RETURN value that was loaded into PC.
         * CFSR raw = 0x00040000 (bit 18).                          */
        g_fault.type = "UsageFault: invalid PC on exception return (INVPC)";
    }
    else if (cfsr & CFSR_INVSTATE)
    {
        /* CPU tried to execute in ARM state (Thumb bit = 0).
         * Cortex-M4 only supports Thumb. EPSR.T must always be 1.
         * Most common cause: function pointer with LSB=0 (missing Thumb bit).
         * fault_pc = the BX/BLX that attempted the ARM-mode switch.
         * CFSR raw = 0x00020000 (bit 17).                          */
        g_fault.type = "UsageFault: invalid execution state (INVSTATE)";
    }
    else if (cfsr & CFSR_UNDEFINSTR)
    {
        /* CPU fetched an opcode it does not understand.
         * Examples: ARMv8 instructions, intentional UDF, bad compiler output.
         * fault_pc = address of the undefined opcode in Flash.
         * CFSR raw = 0x00010000 (bit 16).                          */
        g_fault.type = "UsageFault: undefined instruction (UNDEFINSTR)";
    }
    else
    {
        g_fault.type = "UsageFault: unknown UFSR bits";
    }

    /* Clear UFSR bits by writing 1 to each set bit.
     * Without clearing, the fault fires again on any attempt to return. */
    SCB->CFSR = cfsr;

    /* Breakpoint here — watch g_fault in debugger:
     *   g_fault.type     — which UsageFault sub-type fired
     *   g_fault.fault_pc — exact address of the faulting instruction
     *   g_fault.cfsr     — raw CFSR value for manual bit inspection   */
    while(1);
}

__attribute__((naked))
void UsageFault_Handler(void)
{
    __asm volatile (
        "tst   lr, #4               \n"  /* EXC_RETURN bit2: 0=MSP 1=PSP */
        "ite   eq                   \n"
        "mrseq r0, msp              \n"  /* frame pointer = MSP           */
        "mrsne r0, psp              \n"  /* frame pointer = PSP           */
        "b     UsageFault_Handler_C \n"
    );
}

/* ============================================================
 * HardFault handler
 * ============================================================ */
void HardFault_Handler(void)
{
    volatile uint32_t hfsr = SCB->HFSR;
    volatile uint32_t cfsr = SCB->CFSR;
    g_fault.hfsr = hfsr;
    g_fault.cfsr = cfsr;

    if      (hfsr & HFSR_FORCED)  { g_fault.type = "HardFault: escalated — check CFSR bits"; }
    else if (hfsr & HFSR_VECTTBL) { g_fault.type = "HardFault: vector table read error"; }
    else                           { g_fault.type = "HardFault: unknown"; }

    while(1);
}

/* ============================================================
 * [UF-5] UsageFault test functions
 *
 *  Call ONE at a time from main(). Each targets one CFSR bit.
 * ============================================================ */

/* Test 1 — DIVBYZERO (bit 25)
 * What:  integer signed divide by zero
 * Needs: CCR.DIV_0_TRP = 1  ← already set in MPU_Init()
 * CFSR:  0x02000000
 * PC:    points to the SDIV instruction inside this function    */
void test_divbyzero(void)
{
    volatile int32_t a = 42;
    volatile int32_t b = 0;
    volatile int32_t r = a / b;   /* SDIV r, a, b → UsageFault  */
    (void)r;
}

/* Test 2 — UNALIGNED (bit 24)
 * What:  32-bit load from a non-4-byte-aligned address
 * Needs: CCR.UNALIGN_TRP = 1  ← commented out in MPU_Init by default
 *        Enable it to make this test fault.
 * CFSR:  0x01000000
 * PC:    points to the LDR instruction                          */
void test_unaligned(void)
{
    static uint8_t buf[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
    uint32_t *p = (uint32_t*)(buf + 1);  /* 1-byte offset → misaligned  */
    volatile uint32_t v = *p;            /* LDR from unaligned addr      */
    (void)v;
}

/* Test 3 — NOCP (bit 19)
 * What:  FPU instruction with FPU disabled in CPACR
 * Needs: FPU must be OFF — we disable it here explicitly
 * CFSR:  0x00080000
 * PC:    points to the VADD/VLDR etc. instruction               */
void test_nocp(void)
{
    /* Disable FPU by clearing CP10 and CP11 in CPACR */
    *((volatile uint32_t*)0xE000ED88) &= ~(0xFUL << 20);
    DSB(); ISB();

    /* Any float instruction now → NOCP */
    volatile float a = 1.5f;
    volatile float b = 2.5f;
    volatile float c = a + b;   /* VADD.F32 → NOCP UsageFault   */
    (void)c;
}

/* Test 4 — INVSTATE (bit 17)
 * What:  branch to a function address with LSB=0 (ARM mode)
 * Cortex-M4 is Thumb-only. A function pointer must have LSB=1.
 * Clearing LSB makes the CPU think it should switch to ARM state.
 * CFSR:  0x00020000
 * PC:    the BLX instruction that loaded the bad address         */
void test_invstate(void)
{
    /* Strip Thumb bit (LSB) from a valid function address */
    uint32_t addr = (uint32_t)test_divbyzero & ~1U;  /* LSB = 0 → ARM mode */
    void (*bad)(void) = (void(*)(void))addr;
    bad();   /* BLX → CPU tries ARM state → INVSTATE UsageFault  */
}

/* Test 5 — UNDEFINSTR (bit 16)
 * What:  execute an undefined Thumb opcode
 * UDF #0 (0xDE00) is the official permanently-undefined instruction.
 * CFSR:  0x00010000
 * PC:    address of the UDF instruction in Flash                 */
void test_undefinstr(void)
{
    __asm volatile ("udf #0");   /* Thumb undefined instruction → UsageFault */
}

/* ============================================================
 * MemManage fault test functions
 *
 *  Requires PRIVDEFENA=0 in MPU->CTRL for MM-1 and MM-2.
 *  With PRIVDEFENA=1 those accesses reach the bus → BusFault instead.
 *  MM-3 (XN) fires regardless of PRIVDEFENA.
 *
 *  All land in MemManage_Handler_C → while(1).
 * ============================================================ */

/* MM-1 — DACCVIOL (bit 1): data access to unmapped address
 * What:  write to 0xDEAD0000 — not in any MPU region
 * Needs: PRIVDEFENA=0 (change MPU->CTRL in MPU_Init)
 * CFSR:  0x00000082 (DACCVIOL bit1=1, MMARVALID bit7=1)
 * MMFAR: 0xDEAD0000 — exact bad address
 * PC:    the STR instruction inside this function               */
void test_mm_daccviol(void)
{
    volatile uint32_t *bad = (volatile uint32_t*)0xDEAD0000;
    *bad = 0xBEEF;   /* write to unmapped address → DACCVIOL    */
}

/* MM-2 — DACCVIOL (bit 1): read from unmapped address
 * What:  read from 0xCCCCCCCC — not in any MPU region
 * Needs: PRIVDEFENA=0
 * CFSR:  0x00000082
 * MMFAR: 0xCCCCCCCC
 * PC:    the LDR instruction inside this function               */
void test_mm_daccviol_read(void)
{
    volatile uint32_t *bad = (volatile uint32_t*)0xCCCCCCCC;
    volatile uint32_t v = *bad;   /* read from unmapped → DACCVIOL */
    (void)v;
}

/* MM-3 — DACCVIOL (bit 1): write to Flash (read-only region)
 * What:  write to Flash address — Region 0 is AP_READ_ONLY
 * CFSR:  0x00000082
 * MMFAR: address inside Flash that was written
 * PC:    the STR instruction                                     */
void test_mm_write_flash(void)
{
    /* Address inside our Flash region (region 0, AP=RO).
     * MPU denies the write → DACCVIOL.                           */
    volatile uint32_t *flash = (volatile uint32_t*)0x08001000;
    *flash = 0x12345678;   /* write to RO Flash → DACCVIOL        */
}

/* ============================================================
 * BusFault test functions
 *
 *  Requires PRIVDEFENA=1 (default in MPU_Init) so the MPU lets
 *  the access through to the bus. With PRIVDEFENA=0, unmapped
 *  addresses would get a MemManage fault from the MPU instead.
 *
 *  All land in BusFault_Handler_C → while(1).
 * ============================================================ */

/* BF-1 — PRECISERR (bit 9): write to unconfigured external RAM
 * What:  write to FMC External RAM region — no FMC configured
 *        DSB() forces the write buffer to drain synchronously
 * CFSR:  0x00008200 (PRECISERR bit9=1, BFARVALID bit15=1)
 * BFAR:  0x60000000 — exact bad address
 * PC:    the STR instruction (precise — thanks to DSB)           */
void test_bf_preciserr(void)
{
    (*(volatile uint32_t*)(0x60000000)) = 5;
    DSB();   /* drain write buffer → synchronous error → PRECISERR */
}

/* BF-2 — IMPRECISERR (bit 10): buffered write, no DSB
 * What:  same FMC address but WITHOUT DSB — write goes to buffer
 *        CPU races ahead, error returns asynchronously
 * CFSR:  0x00000400 (IMPRECISERR bit10=1, BFARVALID=0)
 * BFAR:  NOT valid — address is lost in the buffer
 * PC:    a LATER instruction (NOT the bad write) — confusing!
 *
 * Lesson: always add DSB after suspicious writes during debug.   */
void test_bf_impreciserr(void)
{
    (*(volatile uint32_t*)(0x60000000)) = 5;
    /* NO DSB — write buffered → imprecise fault at an unknown PC */
}

/* BF-3 — IBUSERR (bit 8): fetch from beyond end of Flash
 * What:  jump to Flash address past the 1 MB boundary
 *        The Flash interface returns a bus error on the fetch
 * CFSR:  0x00000100 (IBUSERR bit8=1)
 * BFAR:  not updated — use fault_pc instead (= bad fetch address)
 * PC:    0x08100000 — the address the CPU tried to fetch from    */
void test_bf_ibuserr(void)
{
    /* F405 has 1 MB Flash ending at 0x080FFFFF.
     * 0x08100001 = just past the end (Thumb bit set).            */
    void (*bad)(void) = (void(*)(void))0x08100001;
    bad();   /* fetch from beyond Flash → IBUSERR BusFault        */
}

/* BF-4 — PRECISERR: read from AHB2 peripheral with no clock
 * What:  read RNG data register — RNG clock disabled in RCC
 *        AHB bus returns SLVERR → precise BusFault
 * CFSR:  0x00008200
 * BFAR:  0x50060800 (RNG->DR address)
 * PC:    the LDR instruction                                     */
void test_bf_noclk_periph(void)
{
    /* RNG base = 0x50060800. RNG clock is off by default.
     * Reading without enabling the clock → bus SLVERR.           */
    volatile uint32_t rng_dr = *(volatile uint32_t*)0x50060804;
    DSB();   /* force precise fault                               */
    (void)rng_dr;
}

/* ============================================================
 * HardFault test functions
 *
 *  HardFault fires when:
 *   a) A configurable fault escalates (handler disabled in SHCSR)
 *   b) A fault fires inside a fault handler (double fault)
 *   c) The vector table itself is unreadable (VECTTBL)
 *
 *  All land in HardFault_Handler → while(1).
 *  Check g_fault.hfsr first — FORCED or VECTTBL tells you the path.
 *  Then check g_fault.cfsr for the root cause.
 * ============================================================ */

/* HF-1 — FORCED: UsageFault escalates when USGFAULTENA=0
 * What:  disable UsageFault handler, then trigger divide-by-zero
 *        UsageFault cannot activate → escalates to HardFault
 * HFSR:  0x40000000 (FORCED bit30=1)
 * CFSR:  0x02000000 (DIVBYZERO — root cause still recorded)
 * Handler: HardFault_Handler                                     */
void test_hf_forced_usage(void)
{
    /* Disable UsageFault handler temporarily */
    SCB->SHCSR &= ~SHCSR_USGFAULTENA;  /* bit 18 = 0             */
    DSB(); ISB();

    volatile int32_t a = 1, b = 0;
    volatile int32_t r = a / b;   /* DIVBYZERO → no UsageFault handler
                                   * → escalates to HardFault    */
    (void)r;
}

/* HF-2 — FORCED: BusFault escalates when BUSFAULTENA=0
 * What:  disable BusFault handler, then access unmapped address
 *        BusFault cannot activate → escalates to HardFault
 * HFSR:  0x40000000 (FORCED)
 * CFSR:  0x00008200 (PRECISERR — root cause still visible)
 * Handler: HardFault_Handler                                     */
void test_hf_forced_bus(void)
{
    /* Disable BusFault handler temporarily */
    SCB->SHCSR &= ~SHCSR_BUSFAULTENA;  /* bit 17 = 0             */
    DSB(); ISB();

    (*(volatile uint32_t*)(0x60000000)) = 99;
    DSB();   /* PRECISERR → no BusFault handler → HardFault       */
}

/* HF-3 — VECTTBL: vector table read failure
 * What:  point VTOR to an invalid address, then trigger any IRQ
 *        CPU reads (invalid_base + IRQ_number*4) → bus error
 * HFSR:  0x00000002 (VECTTBL bit1=1)
 * CFSR:  0x00000000 — CFSR not set for VECTTBL
 * Handler: HardFault_Handler
 *
 * WARNING: after this test the chip is in an unrecoverable state
 *          until reset — VTOR is corrupt, no IRQ can be handled. */
void test_hf_vecttbl(void)
{
    /* Point VTOR to end of SRAM — no valid vector table there    */
    SCB->VTOR = 0x2001F000;    /* near top of SRAM2, no table    */
    DSB(); ISB();

    /* Trigger SysTick — CPU reads 0x2001F03C for handler address
     * That SRAM is uninitialized rubbish or zero → bus error     */
    /* Force SysTick exception pending via ICSR */
    SCB->ICSR |= (1UL << 26);  /* PENDSTSET — pend SysTick       */
    DSB(); ISB();
    /* SysTick fires → CPU reads vector table → VECTTBL HardFault */
}

/* ============================================================
 * main
 * ============================================================ */
int main(void)
{
    MPU_Init();

    AHB_ENABLE  |= GPIOBEN;
    GPIOB_MODER |=  (1U << 28);
    GPIOB_MODER &= ~(1U << 29);

    /* ════════════════════════════════════════════════════════════
     * Test menu — uncomment ONE function call at a time.
     * Put a breakpoint in the matching handler _C function.
     * Watch g_fault in Live Expressions / Expressions window.
     *
     * ── MemManage tests ─────────────────  handler: MemManage_Handler_C
     *  Needs PRIVDEFENA=0 in MPU->CTRL for MM-1, MM-2, MM-3
     *  (change: MPU->CTRL = MPU_CTRL_ENABLE; in MPU_Init)
     *
     *  test_mm_daccviol()        CFSR=0x00000082  MMFAR=0xDEAD0000
     *  test_mm_daccviol_read()   CFSR=0x00000082  MMFAR=0xCCCCCCCC
     *  test_mm_write_flash()     CFSR=0x00000082  MMFAR=Flash address
     *
     * ── BusFault tests ──────────────────  handler: BusFault_Handler_C
     *  Needs PRIVDEFENA=1 (default) so MPU passes the access to bus
     *
     *  test_bf_preciserr()       CFSR=0x00008200  BFAR=0x60000000
     *  test_bf_impreciserr()     CFSR=0x00000400  BFAR=invalid
     *  test_bf_ibuserr()         CFSR=0x00000100  PC=0x08100000
     *  test_bf_noclk_periph()    CFSR=0x00008200  BFAR=0x50060804
     *
     * ── HardFault tests ─────────────────  handler: HardFault_Handler
     *
     *  test_hf_forced_usage()    HFSR=0x40000000  CFSR=0x02000000
     *  test_hf_forced_bus()      HFSR=0x40000000  CFSR=0x00008200
     *  test_hf_vecttbl()         HFSR=0x00000002  CFSR=0x00000000
     *
     * ── UsageFault tests ────────────────  handler: UsageFault_Handler_C
     *
     *  test_divbyzero()          CFSR=0x02000000
     *  test_unaligned()          CFSR=0x01000000  (enable UNALIGN_TRP)
     *  test_nocp()               CFSR=0x00080000
     *  test_invstate()           CFSR=0x00020000
     *  test_undefinstr()         CFSR=0x00010000
     * ════════════════════════════════════════════════════════════ */

    /* ── Check one by one ── */


    /* MemManage */
    /* test_mm_daccviol();       */
    /* test_mm_daccviol_read();  */
    /* test_mm_write_flash();    */

    /* BusFault */
    /* test_bf_preciserr();      */
    /* test_bf_impreciserr();    */
    /* test_bf_ibuserr();        */
    /* test_bf_noclk_periph();   */

    /* HardFault */
    /* test_hf_forced_usage();   */
    /* test_hf_forced_bus();     */
    /* test_hf_vecttbl();        */

    /* UsageFault */
    //   test_divbyzero();
    /* test_unaligned();         */
    /* test_nocp();              */
    /* test_invstate();          */
    /* test_undefinstr();        */

    for (;;)
    {
        GPIOB_ODR ^= LED_PIN;
        for (int i = 0; i < 100000; i++);
    }

    return 0;
}
