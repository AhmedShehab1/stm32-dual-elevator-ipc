/**
 * @file    motor_pwm.c
 * @brief   TIM1-CH1 PWM Motor Simulation Driver — Implementation
 *
 * @details Two functions implement the complete PWM motor simulation:
 *
 *          Motor_PWM_Init()
 *          ─────────────────
 *          Configures PA8 as TIM1_CH1 (AF1) and programs TIM1 for
 *          10 kHz PWM output.  Register writes are performed in the exact
 *          order mandated by RM0368 §18.3.8 ("PWM output mode").
 *
 *          Mandatory initialisation order (RM0368 §18.3.8):
 *            1. GPIO alternate-function setup (before timer starts).
 *            2. Set PSC and ARR (timing backbone — set while counter off).
 *            3. Set CCR1 (initial duty = 0 %).
 *            4. Set CCMR1.OC1M (output compare mode).
 *            5. Set CCMR1.OC1PE (output-compare preload — mandatory for glitch-free
 *               run-time updates; RM0368 §18.3.9 states preload must be enabled).
 *            6. Set CCER.CC1E (enable channel output).
 *            7. Set BDTR.MOE  (Main Output Enable — TIM1-specific; output stays low
 *               without this bit regardless of CCER; RM0368 §18.4.18).
 *            8. Set CR1.ARPE  (auto-reload preload).
 *            9. Write EGR.UG  (force update event — latches PSC/ARR/CCR1
 *               into their shadow registers; RM0368 §18.4.6).
 *           10. Set CR1.CEN   (start counter — last step, after all config).
 *
 *          Motor_SetSpeed()
 *          ──────────────────
 *          Single CCR1 write, translated via a switch-case from the
 *          MotorSpeed_t enum.  The OC1 preload mechanism ensures the new
 *          duty cycle is loaded at the next update event (UEV), preventing
 *          a mid-period glitch if the write happens during a PWM cycle.
 *
 * @author  Elite Staff Embedded Engineer
 * @version 1.0.0
 */

#include "motor_pwm.h"
#include "stm32f4xx.h"   /* CMSIS: GPIOA, TIM1 register structs */

/* ═══════════════════════════════════════════════════════════════════════════
 * STATIC ASSERTIONS — catch configuration errors at compile time
 *
 * These fire a compile error if the PSC/ARR constants produce a frequency
 * other than 10 kHz, or if CCR1 values exceed ARR+1.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Verify PSC yields exactly 1 MHz tick rate from 84 MHz input clock. */
_Static_assert(
    (MOTOR_TIM_CLK_HZ / (MOTOR_TIM_PSC + 1U)) == 1000000UL,
    "PSC constant does not produce a 1 MHz tick frequency from 84 MHz TIM_CLK"
);

/* Verify ARR yields exactly 10 kHz PWM from the 1 MHz tick rate. */
_Static_assert(
    (MOTOR_TIM_CLK_HZ / (MOTOR_TIM_PSC + 1U)) / (MOTOR_TIM_ARR + 1U) == MOTOR_PWM_FREQ_HZ,
    "ARR constant does not produce 10 kHz PWM from 1 MHz tick frequency"
);

/* Verify duty-cycle CCR1 values are within the valid range [0, ARR+1]. */
_Static_assert(MOTOR_CCR1_STOP <= (MOTOR_TIM_ARR + 1U),
    "MOTOR_CCR1_STOP exceeds ARR+1");
_Static_assert(MOTOR_CCR1_SLOW <= (MOTOR_TIM_ARR + 1U),
    "MOTOR_CCR1_SLOW exceeds ARR+1");
_Static_assert(MOTOR_CCR1_FULL <= (MOTOR_TIM_ARR + 1U),
    "MOTOR_CCR1_FULL exceeds ARR+1");

/* ═══════════════════════════════════════════════════════════════════════════
 * Motor_PWM_Init
 * ═══════════════════════════════════════════════════════════════════════════ */

void Motor_PWM_Init(void)
{
    /* ── 1a. Configure PA8: set MODER8[1:0] = 10 (Alternate Function) ─────
     *
     * GPIOA->MODER is a 32-bit register with 2 bits per pin:
     *   00 = Input, 01 = Output, 10 = Alternate Function, 11 = Analog
     *
     * PA8 occupies MODER[17:16].  We clear both bits with the mask, then
     * OR in the AF mode value (0x2 << 16).
     *
     * Read-modify-write preserves the existing state of all other GPIO pins.
     */
    GPIOA->MODER &= ~PA8_MODER_MASK;
    GPIOA->MODER |=  (PA8_MODER_AF_MODE << PA8_MODER_BIT_POS);

    /* ── 1b. Configure PA8: output speed = high ───────────────────────────
     *
     * OSPEEDR8[1:0] = 10 → high speed (suitable for 10 kHz PWM slew rate).
     * PWM at 10 kHz has a period of 100 µs; even the low-speed setting
     * (OSPEEDR = 00, ~2 MHz) would be adequate, but high speed is preferred
     * for a clean square-wave edge on a simulation LED.
     */
    GPIOA->OSPEEDR &= ~PA8_OSPEEDR_MASK;
    GPIOA->OSPEEDR |=  (PA8_OSPEEDR_HIGH << PA8_OSPEEDR_BIT_POS);

    /* ── 1c. Configure PA8: no pull-up / pull-down ────────────────────────
     *
     * PUPDR8[1:0] = 00 → floating.  The TIM1 peripheral drives the line
     * directly; an external pull would interfere with the duty cycle.
     */
    GPIOA->PUPDR &= ~PA8_PUPDR_MASK;
    /* PA8_PUPDR_NONE = 0x0 → no bits need to be set; mask-clear is enough. */

    /* ── 1d. Configure PA8: Alternate Function 1 (TIM1_CH1) via AFRH ─────
     *
     * STM32F4 GPIO has two AFR registers:
     *   AFRL (AFR[0]) → pins 0–7
     *   AFRH (AFR[1]) → pins 8–15
     *
     * PA8 is in AFRH.  Each pin uses 4 bits; PA8 occupies AFRH[3:0]
     * (bit position = (pin_number - 8) × 4 = (8-8)×4 = 0).
     *
     * AF1 = 0x1 selects TIM1_CH1 / TIM2_CH1 (RM0368 Table 9).
     * AF1 is the ONLY valid alternate function for TIM1_CH1 on PA8.
     *
     * MUST be configured BEFORE the timer counter starts, otherwise the
     * GPIO input/output model during the first PWM cycles is undefined.
     */
    GPIOA->AFR[1] &= ~PA8_AFRH_MASK;
    GPIOA->AFR[1] |=  (PA8_AFRH_AF1 << PA8_AFRH_BIT_POS);

    /* ── 2. Program PSC: prescaler sets tick frequency to 1 MHz ──────────
     *
     * TIM1->PSC = 83 → counter tick rate = 84,000,000 / (83+1) = 1,000,000 Hz
     *
     * PSC is buffered: the new value is loaded at the next update event (UG).
     * The actual reload happens in step 9 (EGR.UG write).
     */
    TIM1->PSC = MOTOR_TIM_PSC;       /* = 83 */

    /* ── 3. Program ARR: auto-reload sets PWM period to 100 µs (10 kHz) ──
     *
     * TIM1->ARR = 99 → counter cycles 0..99 = 100 ticks × 1 µs = 100 µs period
     *           → f_PWM = 1,000,000 / 100 = 10,000 Hz ✓
     *
     * ARR is also buffered (shadow register).  The value is used after UG.
     */
    TIM1->ARR = MOTOR_TIM_ARR;       /* = 99 */

    /* ── 4. Program CCR1: initial duty cycle = 0 % (motor stopped) ────────
     *
     * In PWM mode 1: output is HIGH while CNT < CCR1.
     * CCR1 = 0 → CNT is never < 0 → output always LOW → 0 % duty.
     *
     * CCR1 has a preload shadow register when OC1PE is set (see step 5b).
     * The preloaded value is latched after UG.
     */
    TIM1->CCR1 = MOTOR_CCR1_STOP;    /* = 0 */

    /* ── 5a. Configure CCMR1: set OC1M[2:0] = 110 (PWM mode 1) ───────────
     *
     * CCMR1 register layout (output compare mode for CH1):
     *   bits [1:0]  CC1S  = 00   → CH1 configured as output (required for PWM)
     *   bit  [2]    OC1FE = 0    → fast enable off (not needed for LED PWM)
     *   bit  [3]    OC1PE = set in step 5b
     *   bits [6:4]  OC1M  = 110  → PWM mode 1
     *   bit  [7]    OC1CE = 0    → clear disabled
     *
     * CC1S must be 00 (output) for OC1M to control the output pin.
     * We clear the entire OC1M field first, then OR in the mode value.
     * The CC1S bits start as 00 at reset and are not touched here.
     *
     * PWM mode 1 (OC1M = 110):
     *   Channel is active (HIGH) while CNT < CCR1 (upcounting mode).
     *   Channel is inactive (LOW)  while CNT ≥ CCR1.
     */
    TIM1->CCMR1 &= ~TIM1_CCMR1_OC1M_MASK;       /* Clear OC1M[2:0]       */
    TIM1->CCMR1 |=  TIM1_CCMR1_OC1M_PWM1;       /* Set  OC1M = 110b      */

    /* ── 5b. Enable OC1 preload (OC1PE bit 3) ─────────────────────────────
     *
     * RM0368 §18.3.9 (PWM mode — note): "As the preload registers are
     * transferred to the shadow registers only when an update event occurs,
     * before starting the counter, you have to initialize all the registers
     * by setting the UG bit in the TIMx_EGR register."
     *
     * With OC1PE = 1:
     *   Writes to CCR1 go into the preload register.
     *   The preload value is transferred to the active CCR1 shadow at UEV.
     *   This guarantees that a Motor_SetSpeed() call mid-period does not
     *   cause a partial-period glitch — the duty change only takes effect
     *   at the next clean period boundary.
     */
    TIM1->CCMR1 |= TIM1_CCMR1_OC1PE;

    /* ── 6. Enable Capture/Compare channel 1 output (CCER.CC1E) ──────────
     *
     * TIM1->CCER bit 0 = CC1E:
     *   0 → CC1 output is disabled (pin driven by GPIO logic).
     *   1 → CC1 output is enabled  (pin driven by timer compare output).
     *
     * CC1P (bit 1) = 0 (default) → active HIGH polarity (output HIGH when
     * CNT < CCR1).  Not modified — reset default is correct.
     */
    TIM1->CCER |= TIM1_CCER_CC1E;

    /* ── 7. Enable Main Output (BDTR.MOE — Advanced Timer TIM1 specific) ──
     *
     * TIM1 is an "advanced-control timer."  It has a Break and Dead-Time
     * Register (BDTR) that includes a global output gate called MOE
     * (Main Output Enable), bit 15.
     *
     * Without MOE = 1, ALL TIM1 outputs remain at their idle/reset level
     * regardless of CCER.CC1E, CCMR1, or the running counter.  This is
     * a safety feature for motor-drive applications with dead-time control.
     *
     * For this simulation we have no break input, so MOE is set
     * unconditionally.  General-purpose timers (TIM2–TIM5) do NOT have
     * this register and do NOT require this step.
     */
    TIM1->BDTR |= TIM1_BDTR_MOE;

    /* ── 8. Enable Auto-Reload Preload (CR1.ARPE) ─────────────────────────
     *
     * With ARPE = 1, writes to TIM1->ARR go into the ARR shadow register
     * and are applied at the next UEV.  This prevents a mid-period counter
     * overflow anomaly if ARR is ever updated at run-time.
     *
     * For this project ARR is never changed, but enabling ARPE is best
     * practice for all PWM applications (RM0368 §18.3.1).
     */
    TIM1->CR1 |= TIM1_CR1_ARPE;

    /* ── 9. Generate Update Event (EGR.UG) — latch all shadow registers ───
     *
     * Writing UG = 1 forces an immediate update event.  This causes:
     *   • PSC shadow register  ← TIM1->PSC  (83)
     *   • ARR shadow register  ← TIM1->ARR  (99)
     *   • CCR1 shadow register ← TIM1->CCR1 (0)
     *
     * Without this step, the shadow registers retain their reset values
     * (PSC=0, ARR=0xFFFF) until the first natural overflow, which could
     * produce an incorrect PWM period or duty cycle for the first few
     * hundred milliseconds of operation.
     *
     * UG is cleared by hardware immediately after the update event.
     * The UIF interrupt flag (SR bit 0) is also set by UG — but because
     * no interrupt is enabled for TIM1 Update in this project's NVIC
     * configuration, the flag is harmless.  We clear it explicitly to avoid
     * a spurious interrupt if the NVIC is later enabled for other TIM1 events.
     */
    TIM1->EGR  |= TIM1_EGR_UG;
    TIM1->SR   &= ~(1U << 0U);   /* Clear UIF flag set by the UG write */

    /* ── 10. Start the counter (CR1.CEN) ──────────────────────────────────
     *
     * CEN is set LAST — after all configuration is complete and shadow
     * registers have been initialised by the UG event.
     *
     * From this point the counter runs continuously at 1 MHz, overflowing
     * every 100 µs.  PA8 outputs a 10 kHz PWM signal at 0 % duty cycle.
     * Motor_SetSpeed() changes the duty in real time.
     */
    TIM1->CR1 |= TIM1_CR1_CEN;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Motor_SetSpeed
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Set motor speed by updating the TIM1 capture/compare register.
 *
 * @details The switch-case translates the human-readable enum into the
 *          exact CCR1 value.  Writing to CCR1 when OC1PE = 1 updates the
 *          preload register; the new duty cycle is applied at the next UEV
 *          (i.e. when the 100 µs counter period ends), preventing glitches.
 *
 *          This function is ISR-safe: the single 32-bit write to CCR1 is
 *          atomic on the AHB bus, and the timer preload eliminates the race
 *          condition between a CCR1 write and the counter position.
 *
 * @param speed  MOTOR_STOP → CCR1 = 0  (0 % duty)
 *               MOTOR_SLOW → CCR1 = 20 (20 % duty)
 *               MOTOR_FULL → CCR1 = 100 (100 % duty, forced-high)
 */
void Motor_SetSpeed(MotorSpeed_t speed)
{
    uint32_t ccr1_value;

    switch (speed)
    {
        case MOTOR_STOP:
            /* 0 % duty — elevator stationary, LED fully off.
             * CNT is never < 0, so output stays LOW for the full period. */
            ccr1_value = MOTOR_CCR1_STOP;   /* = 0 */
            break;

        case MOTOR_SLOW:
            /* 20 % duty — elevator decelerating or door-zone creep.
             * Output HIGH for ticks 0–19 (20 ticks), LOW for ticks 20–99. */
            ccr1_value = MOTOR_CCR1_SLOW;   /* = 20 */
            break;

        case MOTOR_FULL:
            /* 100 % duty — elevator in full-speed transit.
             * CCR1 = ARR + 1 = 100.  In PWM mode 1, when CCR1 > ARR the
             * output is forced permanently HIGH for the entire period.
             * RM0368 §18.3.9: "If CCR1 ≥ ARR+1, the output is always active." */
            ccr1_value = MOTOR_CCR1_FULL;   /* = 100 */
            break;

        default:
            /* Defensive: unknown state → stop motor immediately.
             * This branch is unreachable with a well-typed caller but guards
             * against bit corruption of the speed argument in a fault scenario. */
            ccr1_value = MOTOR_CCR1_STOP;   /* = 0 */
            break;
    }

    /* Single 32-bit write — atomic on the AHB bus.
     * The OC1 preload mechanism ensures this is applied at the next UEV,
     * not mid-period.  No critical section is needed. */
    TIM1->CCR1 = ccr1_value;
}
