/**
 * @file    rcc_config.h
 * @brief   STM32F4 Clock Tree Configuration — 84 MHz SYSCLK
 *
 * @details Provides compile-time frequency constants for all clock domains
 *          and declares the two clock-initialisation functions.
 *
 *          Clock tree summary (HSI → PLL → SYSCLK):
 *
 *          HSI (16 MHz)
 *           └─► PLL  /M=16 → 1 MHz VCO-input
 *                    ×N=336 → 336 MHz VCO-output
 *                    /P=4   → 84 MHz  PLL_P  ──► SYSCLK = 84 MHz
 *                    /Q=7   → 48 MHz  PLL_Q  ──► USB / SDIO / RNG
 *
 *          AHB  prescaler = /1  → HCLK   = 84 MHz
 *          APB1 prescaler = /2  → PCLK1  = 42 MHz  (TIM2, TIM3, USART2)
 *          APB2 prescaler = /1  → PCLK2  = 84 MHz  (TIM1, SPI1, SYSCFG)
 *
 *          Timer clock note (RM0368 §6.2):
 *          If APBx prescaler ≠ 1, the timer input clock is 2× PCLK.
 *          ├─ TIM1 input clock = 84 MHz  (APB2 prescaler = 1 → ×1 = 84 MHz)
 *          └─ TIM2/TIM3 input clock = 84 MHz (APB1 prescaler = 2 → ×2 = 84 MHz)
 *
 * @note    All frequency macros are plain integer literals so they can be used
 *          in preprocessor #if comparisons and array-size expressions without
 *          triggering VLA or floating-point warnings.
 *
 * @author  Elite Staff Embedded Engineer
 * @version 1.0.0
 */

#ifndef RCC_CONFIG_H
#define RCC_CONFIG_H

#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * PLL SOURCE & CONFIGURATION CONSTANTS
 *
 * These must match the values written to RCC_PLLCFGR exactly.
 * Changing any one value invalidates the derived frequency constants below.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define PLL_SOURCE_HSI_HZ   (16000000U)   /**< HSI oscillator frequency (Hz). */
#define PLL_M               (16U)         /**< VCO input divider  : f_VCO_in  = 16/16 = 1 MHz. */
#define PLL_N               (336U)        /**< VCO multiplier     : f_VCO_out = 1×336 = 336 MHz. */
#define PLL_P               (4U)          /**< Main output divider: f_PLL_P   = 336/4 = 84 MHz. */
#define PLL_Q               (7U)          /**< USB/SDIO/RNG div   : f_PLL_Q   = 336/7 = 48 MHz. */

/* ═══════════════════════════════════════════════════════════════════════════
 * DERIVED SYSTEM CLOCK FREQUENCIES (Hz)
 *
 * All values are computed from the PLL constants above.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** System clock: PLL_P output selected as SYSCLK. */
#define SYSCLK_FREQ         (84000000U)

/** AHB bus clock (HCLK): AHB prescaler = /1. */
#define HCLK_FREQ           (84000000U)

/** APB1 peripheral clock (PCLK1): APB1 prescaler = /2. Max for STM32F401 = 42 MHz. */
#define PCLK1_FREQ          (42000000U)

/** APB2 peripheral clock (PCLK2): APB2 prescaler = /1. Max for STM32F401 = 84 MHz. */
#define PCLK2_FREQ          (84000000U)

/**
 * @brief Timer 1 input clock (APB2 prescaler = 1 → multiplier = ×1).
 *
 * RM0368 §6.2: "If the APBx prescaler is 1, the timer clock frequencies are
 * set to the same frequency as that of the APBx domain."
 */
#define TIM1_CLK_FREQ       (84000000U)

/**
 * @brief Timer 2 / Timer 3 input clock (APB1 prescaler = 2 → multiplier = ×2).
 *
 * RM0368 §6.2: "Otherwise, they are set to twice (×2) the frequency of the
 * APBx domain."  PCLK1 = 42 MHz → TIM2/TIM3 clock = 84 MHz.
 */
#define TIM2_CLK_FREQ       (84000000U)
#define TIM3_CLK_FREQ       (84000000U)

/** SPI1 input clock = PCLK2 = 84 MHz. */
#define SPI1_CLK_FREQ       PCLK2_FREQ

/** USART2 input clock = PCLK1 = 42 MHz. */
#define USART2_CLK_FREQ     PCLK1_FREQ

/* ═══════════════════════════════════════════════════════════════════════════
 * FLASH WAIT-STATE CONSTANT
 *
 * STM32F401 Flash programming manual (PM0059) §3.4, Table 6:
 *   2.7 V – 3.6 V supply:
 *     0 WS  → HCLK ≤ 30 MHz
 *     1 WS  → HCLK ≤ 60 MHz
 *     2 WS  → HCLK ≤ 84 MHz   ← we are here
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Flash wait states required for HCLK = 84 MHz at 2.7–3.6 V. */
#define FLASH_WAIT_STATES   (2U)

/* ═══════════════════════════════════════════════════════════════════════════
 * STARTUP TIMEOUT
 *
 * PLL lock is typically achieved within ~100 µs after enable.
 * At reset the MCU runs on HSI/6 ≈ 2.66 MHz; a counter of 20 000 ticks
 * provides a safe margin (~7.5 ms) without being excessively long.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Maximum iterations to wait for PLL ready / HSI ready flags. */
#define RCC_LOCK_TIMEOUT    (20000U)

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNCTION DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Configure the STM32F4 clock tree for 84 MHz SYSCLK.
 *
 * @details Sequence:
 *          1. Enable HSI and wait for it to stabilise.
 *          2. Set Flash ACR wait states to 2 WS (mandatory before PLL switch).
 *          3. Configure RCC_PLLCFGR (M=16, N=336, P=4, Q=7, source=HSI).
 *          4. Set AHB, APB1 (/2), APB2 (/1) prescalers in RCC_CFGR.
 *          5. Enable PLL and wait for PLLRDY with a safe timeout.
 *          6. Switch SYSCLK source to PLL and wait for SWS confirmation.
 *          7. Enable instruction cache, data cache, and prefetch buffer.
 *
 * @note   Must be the FIRST function called in main(), before any peripheral
 *         initialisation that depends on a specific clock frequency.
 *
 * @return Nothing. On PLL-lock timeout the function enters a safe infinite
 *         loop (fail-safe halt) rather than proceeding with an incorrect clock.
 */
void SystemClock_Config_84MHz(void);

/**
 * @brief  Enable RCC peripheral bus clocks for all project peripherals.
 *
 * @details Enables the following clocks via atomic read-modify-write on the
 *          three RCC enable registers:
 *
 *          AHB1ENR  : GPIOA, GPIOB, GPIOC, DMA1
 *          APB1ENR  : TIM2, TIM3, USART2
 *          APB2ENR  : TIM1, SPI1, SYSCFG
 *
 * @note   Call AFTER SystemClock_Config_84MHz() and BEFORE any peripheral
 *         driver init (GPIO MODER writes, timer PSC/ARR writes, etc.).
 *
 * @note   A mandatory dummy read of each enable register is performed after
 *         the write to ensure the clock gate has propagated before the caller
 *         proceeds to configure peripheral registers (RM0368 §6.3.11 note).
 */
void Peripheral_Clocks_Enable(void);

#endif /* RCC_CONFIG_H */
