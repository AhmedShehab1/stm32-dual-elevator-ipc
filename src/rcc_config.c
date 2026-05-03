/**
 * @file    rcc_config.c
 * @brief   STM32F4 Clock Tree & Peripheral RCC Configuration — 84 MHz
 *
 * @details Implements two isolated, sequentially-called functions:
 *
 *          1. SystemClock_Config_84MHz()
 *             Brings up the HSI → PLL chain and switches SYSCLK to 84 MHz.
 *             Every register write is commented with the bit-field name and
 *             the RM0368 section that mandates the ordering.
 *
 *          2. Peripheral_Clocks_Enable()
 *             Single-pass enable of every peripheral clock gate needed by
 *             the Dual-Elevator project.  Uses read-modify-write (|=) to
 *             avoid clobbering bits owned by other subsystems.
 *
 * @note    Direct register access via CMSIS device-header structs only.
 *          No HAL, no LL, no cube-generated code.
 *
 * @note    Register-field bit positions used throughout:
 *
 *          RCC_CR:
 *            HSION   [0]    HSI oscillator enable
 *            HSIRDY  [1]    HSI oscillator ready
 *            PLLON   [24]   Main PLL enable
 *            PLLRDY  [25]   Main PLL ready
 *
 *          RCC_PLLCFGR:
 *            PLLM    [5:0]  Division factor for VCO input
 *            PLLN    [14:6] Multiplication factor for VCO
 *            PLLP    [17:16] Division for main system clock (00=2,01=4,10=6,11=8)
 *            PLLSRC  [22]   0=HSI, 1=HSE
 *            PLLQ    [27:24] Division for USB/SDIO/RNG
 *
 *          RCC_CFGR:
 *            SW      [1:0]  System clock switch (10 = PLL)
 *            SWS     [3:2]  System clock switch status
 *            HPRE    [7:4]  AHB prescaler  (0xxx = /1)
 *            PPRE1   [12:10] APB1 prescaler (100=2, 101=4, 110=8, 111=16)
 *            PPRE2   [15:13] APB2 prescaler (0xx = /1)
 *
 *          FLASH_ACR:
 *            LATENCY [2:0]  Wait states
 *            PRFTEN  [8]    Prefetch enable
 *            ICEN    [9]    Instruction cache enable
 *            DCEN    [10]   Data cache enable
 *
 * @author  Elite Staff Embedded Engineer
 * @version 1.0.0
 */

#include "rcc_config.h"
#include "stm32f4xx.h"   /* CMSIS: RCC, FLASH register structs + bit definitions */

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERNAL: PRIVATE BIT-FIELD CONSTANTS
 *
 * Define only what is not already provided by the CMSIS header, or provide
 * named aliases that make the intent of each write self-documenting.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── RCC_PLLCFGR field positions ── */
#define RCC_PLLCFGR_PLLM_POS        (0U)
#define RCC_PLLCFGR_PLLN_POS        (6U)
#define RCC_PLLCFGR_PLLP_POS        (16U)
#define RCC_PLLCFGR_PLLSRC_HSI      (0U)          /* Bit 22 = 0 → HSI source */
#define RCC_PLLCFGR_PLLQ_POS        (24U)

/*
 * PLLP encoding (RM0368 Table 27):
 *   00 → /2,  01 → /4,  10 → /6,  11 → /8
 * PLL_P = 4 → encoded value = 1 (01b).
 */
#define PLLP_ENCODED                (1U)           /* Encodes PLL_P = 4 (/4) */

/* ── RCC_CFGR field values ── */
#define RCC_CFGR_SW_PLL             (0x2U)         /* SW[1:0] = 10 → PLL as SYSCLK */
#define RCC_CFGR_SWS_PLL            (0x2U << 2U)   /* SWS[3:2] = 10 → PLL active   */
#define RCC_CFGR_SWS_MASK           (0x3U << 2U)

#define RCC_CFGR_HPRE_DIV1          (0x0U << 4U)   /* AHB  prescaler /1 */
#define RCC_CFGR_PPRE1_DIV2         (0x4U << 10U)  /* APB1 prescaler /2 (100b) */
#define RCC_CFGR_PPRE2_DIV1         (0x0U << 13U)  /* APB2 prescaler /1 (0xxb) */

/* ── FLASH_ACR field bits ── */
#define FLASH_ACR_LATENCY_2WS       (0x2U)         /* 2 wait states for 84 MHz */
#define FLASH_ACR_LATENCY_MASK      (0xFU)
#define FLASH_ACR_PRFTEN_BIT        (1U << 8U)     /* Prefetch enable */
#define FLASH_ACR_ICEN_BIT          (1U << 9U)     /* Instruction cache enable */
#define FLASH_ACR_DCEN_BIT          (1U << 10U)    /* Data cache enable */

/* ═══════════════════════════════════════════════════════════════════════════
 * INTERNAL: FAIL-SAFE HALT
 *
 * Called when a hardware ready-flag never asserts within RCC_LOCK_TIMEOUT
 * iterations.  The system cannot be trusted at this point — halt safely
 * rather than execute application code on an unknown clock.
 *
 * In a production system this would trigger a watchdog-reset or log a fault
 * code.  For this project it is a visible, debuggable infinite loop.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void clock_fault_halt(void)
{
    /* Disable all maskable interrupts to prevent re-entrant behaviour. */
    __disable_irq();

    /* An infinite loop is detectable in a debugger via the PC register. */
    for (;;)
    {
        __NOP();   /* Prevent the loop from being optimised away. */
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SystemClock_Config_84MHz
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Configure HSI → PLL → 84 MHz SYSCLK.
 *
 * MANDATORY ORDERING (RM0368 §6.3.2, §3.4):
 *
 *   Step 1 — Enable & verify HSI before using it as PLL source.
 *   Step 2 — Set Flash wait states BEFORE raising the clock speed.
 *             Failure to do this causes a HardFault on the first
 *             instruction fetch after the clock switch.
 *   Step 3 — Configure PLL (only legal while PLLON = 0).
 *   Step 4 — Configure bus prescalers (can be done any time before step 6).
 *   Step 5 — Enable PLL and wait for lock (PLLRDY).
 *   Step 6 — Switch SYSCLK to PLL and wait for hardware confirmation (SWS).
 *   Step 7 — Enable Flash accelerator caches (optional but recommended).
 */
void SystemClock_Config_84MHz(void)
{
    uint32_t timeout;

    /* ── Step 1: Ensure HSI is ON and stable ──────────────────────────────
     *
     * After power-on / reset the HSI is already the default clock source,
     * but we explicitly enable and verify it for robustness — e.g. if this
     * function is ever called from a software-reset context where the clock
     * source has been changed.
     *
     * RCC_CR.HSION [bit 0] = 1 → enable HSI oscillator.
     * RCC_CR.HSIRDY [bit 1] = polled until 1 (hardware sets when stable).
     */
    RCC->CR |= RCC_CR_HSION;

    timeout = RCC_LOCK_TIMEOUT;
    while (((RCC->CR & RCC_CR_HSIRDY) == 0U) && (timeout > 0U))
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        clock_fault_halt();   /* HSI never stabilised — cannot proceed. */
    }

    /* ── Step 2: Configure Flash latency for 84 MHz BEFORE the clock switch
     *
     * At 84 MHz / 2.7–3.6 V supply the Flash subsystem requires 2 wait
     * states (PM0059 Table 6).  Writing this BEFORE raising the frequency
     * is mandatory — if the CPU executes a Flash fetch at 84 MHz with 0 WS
     * configured, the instruction read returns stale data and the core
     * immediately HardFaults.
     *
     * We also enable:
     *   PRFTEN [8]  — Prefetch buffer: hides single-cycle latency on
     *                 sequential instruction fetches (recommended for 84 MHz).
     *   ICEN   [9]  — Instruction cache: 64 cache lines × 128-bit, reduces
     *                 Flash stalls for loops and repeated code.
     *   DCEN   [10] — Data cache: reduces stall on literal-pool loads.
     *
     * Read-modify-write: preserve any reserved bits or OTP bits already set.
     */
    {
        uint32_t acr = FLASH->ACR;
        acr &= ~FLASH_ACR_LATENCY_MASK;                      /* Clear LATENCY field */
        acr |=  FLASH_ACR_LATENCY_2WS                        /* Set 2 wait states   */
             |  FLASH_ACR_PRFTEN_BIT
             |  FLASH_ACR_ICEN_BIT
             |  FLASH_ACR_DCEN_BIT;
        FLASH->ACR = acr;
    }

    /* Verify the latency register actually accepted the write before going
     * further.  FLASH_ACR is in the AHB clock domain so the read-back
     * should be immediate, but the defensive check costs nothing.
     */
    if ((FLASH->ACR & FLASH_ACR_LATENCY_MASK) != FLASH_ACR_LATENCY_2WS)
    {
        clock_fault_halt();
    }

    /* ── Step 3: Configure RCC_PLLCFGR (PLL must be OFF while writing) ────
     *
     * At reset PLLON = 0, so no explicit disable is needed on first boot.
     * The write below is a full assignment (not |=) to guarantee a clean
     * known state — all fields are set explicitly.
     *
     * Bit layout written:
     *   PLLM   [5:0]  = 16   → VCO input = 16/16 = 1 MHz
     *   PLLN   [14:6] = 336  → VCO output = 336 MHz
     *   PLLP   [17:16]= 01   → SYSCLK output = 336/4 = 84 MHz
     *   PLLSRC [22]   = 0    → HSI as PLL source
     *   PLLQ   [27:24]= 7    → USB/SDIO/RNG = 336/7 = 48 MHz
     *
     * PLLN must be in the range [50, 432] for STM32F401 (RM0368 §6.3.2).
     * VCO input must be in [1, 2] MHz (we use exactly 1 MHz — optimal).
     * VCO output must be in [100, 432] MHz (336 MHz — within spec).
     */
    RCC->PLLCFGR =
          ((uint32_t)PLL_M     << RCC_PLLCFGR_PLLM_POS)   /* PLLM = 16         */
        | ((uint32_t)PLL_N     << RCC_PLLCFGR_PLLN_POS)   /* PLLN = 336        */
        | ((uint32_t)PLLP_ENCODED << RCC_PLLCFGR_PLLP_POS)/* PLLP encoding = 1 (/4) */
        | ((uint32_t)RCC_PLLCFGR_PLLSRC_HSI << 22U)       /* HSI source        */
        | ((uint32_t)PLL_Q     << RCC_PLLCFGR_PLLQ_POS);  /* PLLQ = 7          */

    /* ── Step 4: Configure AHB, APB1, APB2 prescalers in RCC_CFGR ─────────
     *
     * These can be set before or after enabling the PLL, but must be correct
     * before SYSCLK is switched so peripherals see the right frequencies
     * from the first cycle.
     *
     * We clear only the prescaler fields and write them fresh; the SW field
     * is left as 00 (HSI) for now and will be switched in step 6.
     *
     * HPRE  [7:4]  = 0000 → AHB /1  → HCLK  = 84 MHz
     * PPRE1 [12:10]= 100  → APB1 /2 → PCLK1 = 42 MHz  (max 42 MHz on F401)
     * PPRE2 [15:13]= 000  → APB2 /1 → PCLK2 = 84 MHz  (max 84 MHz on F401)
     */
    {
        uint32_t cfgr = RCC->CFGR;

        /* Clear the three prescaler fields only. */
        cfgr &= ~(  RCC_CFGR_HPRE                /* bits  [7:4]  */
                  | RCC_CFGR_PPRE1               /* bits  [12:10]*/
                  | RCC_CFGR_PPRE2 );            /* bits  [15:13]*/

        /* Write prescalers. SW is still 00 (HSI) at this point. */
        cfgr |= RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_PPRE2_DIV1;

        RCC->CFGR = cfgr;
    }

    /* ── Step 5: Enable PLL and wait for lock (PLLRDY) ─────────────────────
     *
     * After setting PLLON = 1, hardware locks the VCO.  PLLRDY is set by
     * hardware when the output is stable.  We poll with a timeout to avoid
     * an infinite hang on defective silicon or a broken crystal configuration.
     *
     * Typical lock time: ~50–100 µs at HSI startup frequencies.
     */
    RCC->CR |= RCC_CR_PLLON;

    timeout = RCC_LOCK_TIMEOUT;
    while (((RCC->CR & RCC_CR_PLLRDY) == 0U) && (timeout > 0U))
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        clock_fault_halt();   /* PLL never locked — do not switch SYSCLK. */
    }

    /* ── Step 6: Switch SYSCLK source to PLL ───────────────────────────────
     *
     * RCC_CFGR.SW[1:0] = 10 → select PLL as SYSCLK source.
     * Hardware then updates SWS[3:2] to 10 once the switch is complete
     * (takes a small number of synchronisation cycles).
     * We must wait for SWS confirmation — reading the old SYSCLK speed
     * before the switch completes could corrupt a peripheral configuration.
     */
    {
        uint32_t cfgr = RCC->CFGR;
        cfgr &= ~RCC_CFGR_SW;                /* Clear SW[1:0] */
        cfgr |=  RCC_CFGR_SW_PLL;            /* Set  SW = 10 (PLL) */
        RCC->CFGR = cfgr;
    }

    /* Wait for hardware to confirm the switch via SWS[3:2]. */
    timeout = RCC_LOCK_TIMEOUT;
    while (((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) && (timeout > 0U))
    {
        timeout--;
    }
    if (timeout == 0U)
    {
        clock_fault_halt();   /* Clock switch never completed. */
    }

    /*
     * At this point:
     *   SYSCLK = 84 MHz (PLL_P output)
     *   HCLK   = 84 MHz (AHB /1)
     *   PCLK1  = 42 MHz (APB1 /2)
     *   PCLK2  = 84 MHz (APB2 /1)
     *   Flash  = 2 WS with prefetch + I/D caches enabled
     *
     * The core is now running at full speed.  All subsequent initialisation
     * functions can rely on SYSCLK_FREQ and the PCLK constants in rcc_config.h.
     */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Peripheral_Clocks_Enable
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Enable RCC bus clocks for all Dual-Elevator project peripherals.
 *
 * Peripheral → bus → enable register mapping:
 *
 *   AHB1ENR  (0x40023830)
 *   ├─ GPIOA   bit  0   (RCC_AHB1ENR_GPIOAEN)
 *   ├─ GPIOB   bit  1   (RCC_AHB1ENR_GPIOBEN)
 *   ├─ GPIOC   bit  2   (RCC_AHB1ENR_GPIOCEN)
 *   └─ DMA1    bit 21   (RCC_AHB1ENR_DMA1EN)
 *
 *   APB1ENR  (0x40023840)
 *   ├─ TIM2    bit  0   (RCC_APB1ENR_TIM2EN)
 *   ├─ TIM3    bit  1   (RCC_APB1ENR_TIM3EN)
 *   └─ USART2  bit 17   (RCC_APB1ENR_USART2EN)
 *
 *   APB2ENR  (0x40023844)
 *   ├─ TIM1    bit  0   (RCC_APB2ENR_TIM1EN)
 *   ├─ USART1  —        (not used in this project)
 *   ├─ SPI1    bit 12   (RCC_APB2ENR_SPI1EN)
 *   └─ SYSCFG  bit 14   (RCC_APB2ENR_SYSCFGEN)
 *
 * EXTI lines are managed by SYSCFG (APB2) and the EXTI controller itself
 * (RCC_APB2ENR_SYSCFGEN covers both).  There is no separate "EXTI enable"
 * in the RCC — the EXTI block is always clocked once SYSCFG is enabled.
 *
 * After each register write we perform a mandatory dummy read.
 * RM0368 §6.3.11 note: "After the enable bit is set, there is a 2 clock
 * cycle delay before the clock is active."  The dummy read ensures the
 * write has propagated through the AHB bus before the caller attempts to
 * configure a peripheral register.
 */
void Peripheral_Clocks_Enable(void)
{
    volatile uint32_t dummy;   /* Declared volatile to prevent read elimination. */

    /* ── AHB1: GPIO A/B/C + DMA1 ──────────────────────────────────────────
     *
     * Single |= to enable all four clocks in one write cycle.
     * The bits are non-overlapping, so there is no risk of accidental
     * enable/disable of other AHB1 peripherals not owned by this project.
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN    /* GPIOA: SPI1, UART2, motor LED */
                  | RCC_AHB1ENR_GPIOBEN    /* GPIOB: floor buttons, sensors, CS */
                  | RCC_AHB1ENR_GPIOCEN    /* GPIOC: hallway calls, E-stop     */
                  | RCC_AHB1ENR_DMA1EN;    /* DMA1:  UART2 TX (Stream6 Ch4)    */

    dummy = RCC->AHB1ENR;                  /* Mandatory read-back delay        */
    (void)dummy;

    /* ── APB1: TIM2, TIM3, USART2 ─────────────────────────────────────────
     *
     * APB1 is clocked at PCLK1 = 42 MHz.
     * Timer input clocks are 84 MHz (×2 multiplier — see rcc_config.h).
     *
     * TIM2   → 50 ms SPI sync tick (Master) / Slave pre-load tick
     * TIM3   → 500 ms UART telemetry tick → triggers DMA transfer
     * USART2 → 500 ms non-blocking debug output (DMA-driven)
     */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN     /* 50 ms SPI IPC dispatch timer   */
                  | RCC_APB1ENR_TIM3EN     /* 500 ms UART telemetry timer     */
                  | RCC_APB1ENR_USART2EN;  /* UART debug (PA2=TX, PA3=RX)    */

    dummy = RCC->APB1ENR;                  /* Mandatory read-back delay       */
    (void)dummy;

    /* ── APB2: TIM1, SPI1, SYSCFG ─────────────────────────────────────────
     *
     * APB2 is clocked at PCLK2 = 84 MHz (same as SYSCLK).
     *
     * TIM1   → 10 kHz PWM for motor-simulation LED (CH1 on PA8)
     * SPI1   → Full-duplex IPC link (PA5=SCK, PA6=MISO, PA7=MOSI, PB6=NSS)
     * SYSCFG → EXTICR register access for routing GPIO lines to EXTI.
     *          Also gates the EXTI block itself — no separate EXTI enable bit.
     *
     * Note: SYSCFG MUST be enabled before writing SYSCFG_EXTICR registers
     * in the GPIO/EXTI initialisation stage, otherwise those writes will
     * be silently ignored (the peripheral is clock-gated).
     */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN     /* PWM motor simulation (10 kHz)   */
                  | RCC_APB2ENR_SPI1EN     /* SPI IPC link @ APB2 speed       */
                  | RCC_APB2ENR_SYSCFGEN;  /* SYSCFG + EXTI controller        */

    dummy = RCC->APB2ENR;                  /* Mandatory read-back delay       */
    (void)dummy;

    /*
     * All 10 peripheral clocks are now active:
     *   AHB1 : GPIOA, GPIOB, GPIOC, DMA1
     *   APB1 : TIM2, TIM3, USART2
     *   APB2 : TIM1, SPI1, SYSCFG (covers EXTI)
     *
     * Callers may now safely write to GPIO MODER, TIM PSC/ARR, SPI CR1, etc.
     */
}
