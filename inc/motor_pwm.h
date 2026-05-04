/**
 * @file    motor_pwm.h
 * @brief   TIM1-CH1 PWM Motor Simulation Driver — STM32F4 @ 84 MHz
 *
 * @details Generates a 10 kHz PWM signal on PA8 (TIM1_CH1) to simulate
 *          motor speed via LED brightness.  Three discrete speed states are
 *          provided through the MotorSpeed_t enum; the duty cycle is updated
 *          at run-time through a single non-blocking register write to CCR1.
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CLOCK TREE (relevant path only)                                        │
 * │                                                                         │
 * │  SYSCLK 84 MHz ──► APB2 prescaler /1 ──► PCLK2 = 84 MHz               │
 * │                                           │                             │
 * │  APB2 prescaler = 1 → TIM1 multiplier ×1  │                             │
 * │  (RM0368 §6.2: if APBx prescaler = 1,     │                             │
 * │   timer clock = PCLK, NOT 2×PCLK)         │                             │
 * │                                           ▼                             │
 * │                                      TIM1_CLK = 84 MHz                 │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  PWM MATH                                                               │
 * │                                                                         │
 * │  f_tick  = f_TIM  / (PSC + 1)                                          │
 * │          = 84,000,000 / (83 + 1)  =  1,000,000 Hz  (1 µs per tick)    │
 * │                                                                         │
 * │  f_PWM   = f_tick / (ARR + 1)                                          │
 * │          = 1,000,000 / (99 + 1)   = 10,000 Hz  ✓                      │
 * │                                                                         │
 * │  Duty %  = CCR1 / (ARR + 1)  × 100                                    │
 * │          CCR1 =  0 →   0 %  (motor stopped)                            │
 * │          CCR1 = 20 →  20 %  (motor slow)                               │
 * │          CCR1 = 100 → 100 % (motor full — force-high via CCR1 > ARR)   │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * @note    Motor_PWM_Init() assumes RCC clocks for GPIOA and TIM1 have
 *          already been enabled (by Peripheral_Clocks_Enable() in rcc_config.c).
 *
 * @note    The 100 % state is achieved by writing CCR1 = ARR + 1 = 100,
 *          which forces the output permanently high in PWM mode 1.
 *          This is the correct, hardware-documented approach (RM0368 §18.3.9).
 *
 * @author  Elite Staff Embedded Engineer
 * @version 1.0.0
 */

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * CLOCK & TIMER CONSTANTS  (derived from rcc_config.h values)
 *
 * Defined here independently so motor_pwm.c is a self-contained module
 * that documents its own assumptions.  Must stay in sync with rcc_config.h.
 * ═══════════════════════════════════════════════════════════════════════════ */

/** TIM1 input clock frequency (Hz).  APB2 prescaler = /1 → TIM1_CLK = PCLK2. */
#define MOTOR_TIM_CLK_HZ        (84000000UL)

/** Target PWM output frequency (Hz). */
#define MOTOR_PWM_FREQ_HZ       (10000UL)

/* ═══════════════════════════════════════════════════════════════════════════
 * PSC / ARR / CCR1 REGISTER VALUES
 *
 * Derived from the formula:
 *   PSC  = (f_TIM / (f_tick))       - 1   where f_tick is chosen as 1 MHz
 *   ARR  = (f_tick / f_PWM)         - 1
 *   CCR1 = (duty_percent / 100) × (ARR + 1)
 *
 * These constants are used in Motor_PWM_Init() and serve as documentation
 * for any reader who wants to verify the math without opening the .c file.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Prescaler register value.
 *
 * f_tick = 84,000,000 / (83 + 1) = 1,000,000 Hz  (1 µs/tick).
 * Written to TIM1->PSC.  Hardware adds 1 internally, so the divisor is 84.
 */
#define MOTOR_TIM_PSC           (83U)

/**
 * @brief Auto-reload register value (defines the PWM period).
 *
 * f_PWM = 1,000,000 / (99 + 1) = 10,000 Hz.
 * The counter counts 0..99 (100 ticks per period = 100 µs).
 * Written to TIM1->ARR.
 */
#define MOTOR_TIM_ARR           (99U)

/**
 * @brief CCR1 value for 0 % duty cycle (motor stopped).
 *
 * In PWM mode 1: output is HIGH while CNT < CCR1.
 * CCR1 = 0 → CNT is never < 0 → output is always LOW → 0 % duty.
 */
#define MOTOR_CCR1_STOP         (0U)

/**
 * @brief CCR1 value for 20 % duty cycle (motor slow).
 *
 * CNT < 20 → HIGH (20 ticks out of 100) → 20 % duty.
 * Represents low-speed / creep motion.
 */
#define MOTOR_CCR1_SLOW         (20U)

/**
 * @brief CCR1 value for 100 % duty cycle (motor full speed).
 *
 * CCR1 = ARR + 1 = 100.
 * RM0368 §18.3.9: When CCR1 > ARR in PWM mode 1, the output is
 * permanently forced HIGH → 100 % duty cycle without counter wrap artefacts.
 */
#define MOTOR_CCR1_FULL         (MOTOR_TIM_ARR + 1U)   /* = 100 */

/* ═══════════════════════════════════════════════════════════════════════════
 * REGISTER BIT CONSTANTS  (named aliases for every magic number)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── GPIOA MODER: PA8 alternate-function mode (MODER bits[17:16] = 10) ── */
#define PA8_MODER_BIT_POS       (16U)          /* MODER8[1:0] starts at bit 16 */
#define PA8_MODER_AF_MODE       (0x2U)         /* 10b = Alternate Function mode */
#define PA8_MODER_MASK          (0x3U << PA8_MODER_BIT_POS)

/* ── GPIOA OSPEEDR: PA8 high speed (OSPEEDR8[1:0] = 10) ── */
#define PA8_OSPEEDR_BIT_POS     (16U)
#define PA8_OSPEEDR_HIGH        (0x2U)         /* 10b = high speed */
#define PA8_OSPEEDR_MASK        (0x3U << PA8_OSPEEDR_BIT_POS)

/* ── GPIOA PUPDR: PA8 no pull (PUPDR8[1:0] = 00) ── */
#define PA8_PUPDR_BIT_POS       (16U)
#define PA8_PUPDR_NONE          (0x0U)
#define PA8_PUPDR_MASK          (0x3U << PA8_PUPDR_BIT_POS)

/**
 * @brief Alternate Function 1 (TIM1_CH1) selector for PA8 in AFRH.
 *
 * PA8 is in the HIGH alternate-function register (AFRH = AFR[1]).
 * AF1 = 0x1 occupies bits [3:0] of AFRH (i.e. AFRH[8-8] = pin 8 offset 0).
 * AFRH pin offset: pin_number - 8.  PA8 → offset = 0 → bits [3:0].
 */
#define PA8_AFRH_BIT_POS        (0U)           /* (8 - 8) × 4 = 0 */
#define PA8_AFRH_AF1            (0x1U)         /* AF1 = TIM1/TIM2 */
#define PA8_AFRH_MASK           (0xFU << PA8_AFRH_BIT_POS)

/* ── TIM1 CCMR1: OC1M[2:0] = 110b = PWM mode 1, at bits [6:4] ── */
#define TIM1_CCMR1_OC1M_PWM1   (6U << 4U)     /* PWM mode 1: active while CNT < CCR1 */
#define TIM1_CCMR1_OC1PE        (1U << 3U)     /* OC1 preload enable (recommended) */
#define TIM1_CCMR1_OC1M_MASK   (7U << 4U)     /* OC1M field mask */

/* ── TIM1 CCER: CC1E [bit 0] = Capture/Compare 1 output enable ── */
#define TIM1_CCER_CC1E          (1U << 0U)

/* ── TIM1 BDTR: MOE [bit 15] = Main Output Enable (mandatory for advanced timers) ── */
#define TIM1_BDTR_MOE           (1U << 15U)

/* ── TIM1 CR1: ARPE [bit 7] = Auto-reload preload enable ── */
#define TIM1_CR1_ARPE           (1U << 7U)

/* ── TIM1 CR1: CEN [bit 0] = Counter enable ── */
#define TIM1_CR1_CEN            (1U << 0U)

/* ── TIM1 EGR: UG [bit 0] = Update generation (force PSC/ARR reload) ── */
#define TIM1_EGR_UG             (1U << 0U)

/* ═══════════════════════════════════════════════════════════════════════════
 * MOTOR SPEED ENUM
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Discrete motor speed states for the elevator simulation.
 *
 * These map directly to the three CCR1 duty-cycle values defined above.
 * The numeric values are NOT written to registers — Motor_SetSpeed() uses
 * a switch-case to translate them to the correct CCR1 constant.
 *
 * Usage:
 * @code
 *   Motor_SetSpeed(MOTOR_STOP);   // LED off  — elevator stationary
 *   Motor_SetSpeed(MOTOR_SLOW);   // LED dim  — elevator approaching floor
 *   Motor_SetSpeed(MOTOR_FULL);   // LED full — elevator in transit
 * @endcode
 */
typedef enum
{
    MOTOR_STOP  = 0,    /**< 0 %  duty — motor off, elevator stationary.    */
    MOTOR_SLOW  = 1,    /**< 20 % duty — low speed (deceleration / startup). */
    MOTOR_FULL  = 2     /**< 100 % duty — full speed, elevator in transit.   */
} MotorSpeed_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNCTION DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise TIM1-CH1 for 10 kHz PWM on PA8.
 *
 * @details Performs, in order:
 *          1. Configure PA8: AF mode, AF1 selection, high speed, no pull.
 *          2. Set TIM1->PSC = 83  (f_tick = 1 MHz).
 *          3. Set TIM1->ARR = 99  (f_PWM  = 10 kHz).
 *          4. Set TIM1->CCR1 = 0  (start stopped).
 *          5. Configure CCMR1: PWM mode 1, OC1 preload enable.
 *          6. Configure CCER: enable CC1 output.
 *          7. Configure BDTR: set MOE (Main Output Enable — mandatory for TIM1).
 *          8. Configure CR1:  enable ARPE, generate UG update event, start counter.
 *
 * @pre    RCC clocks for GPIOA and TIM1 must be enabled before calling
 *         (handled by Peripheral_Clocks_Enable() in rcc_config.c).
 *
 * @note   This function does NOT enable or configure any NVIC interrupt.
 *         PWM generation is fully hardware-driven once the counter is running.
 *
 * @note   After this call the motor is in MOTOR_STOP state (CCR1 = 0).
 *         Call Motor_SetSpeed() to change the duty cycle.
 */
void Motor_PWM_Init(void);

/**
 * @brief  Set the motor simulation speed by updating TIM1->CCR1.
 *
 * @details This is a non-blocking, single-register write.  The new duty
 *          cycle takes effect at the next counter update event (end of the
 *          current 100 µs PWM period) because OC1 preload is enabled.
 *
 * @param[in] speed  Desired motor speed state from the MotorSpeed_t enum.
 *
 * @note   Safe to call from ISR context — the write to CCR1 is atomic on
 *         the 32-bit AHB bus and the timer preload mechanism prevents
 *         mid-period glitches.
 */
void Motor_SetSpeed(MotorSpeed_t speed);

#endif /* MOTOR_PWM_H */
