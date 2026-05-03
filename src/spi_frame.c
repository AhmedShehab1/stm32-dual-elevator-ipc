/**
 * @file    spi_frame.c
 * @brief   SPI IPC Frame Serialisation, Deserialisation & Checksum
 *
 * @details Implements the three helper functions declared in spi_frame.h.
 *
 *          Design principles enforced here:
 *
 *          1. ZERO STRUCT PADDING RISK
 *             Fields are written byte-by-byte into a uint8_t[8] buffer.
 *             No memcpy of structs, no union overlays — the raw buffer is
 *             always built field-by-field so compiler ABI differences are
 *             irrelevant.
 *
 *          2. NO BLOCKING / NO POLLING
 *             These functions perform only arithmetic and memory writes.
 *             They are safe to call from TIM2_IRQHandler (50 ms tick) or
 *             from the main-loop dispatcher.
 *
 *          3. CALLER OWNS ATOMICITY
 *             spi_frame_serialise() trusts that the caller has already
 *             snapshotted volatile variables under Enter_Critical().
 *             This keeps the function ISR-agnostic and testable on a host.
 *
 *          4. FAST CHECKSUM
 *             The inline macro SPI_CALC_CHECKSUM() in spi_frame.h is the
 *             primary path.  spi_calc_checksum_fn() is the out-of-line
 *             fallback (e.g. for variable-length future extensions).
 *
 * @author  Elite Staff Embedded Engineer
 * @version 1.0.0
 */

#include "spi_frame.h"

/* ─────────────────────────────────────────────────────────────────────────
 * INTERNAL HELPERS
 * ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Clamp a floor value to the valid range [1, SPI_FLOOR_MAX].
 *
 * Defensive guard — prevents nibble overflow if a bug sets floor > 4.
 * Runs as a single comparison + cmov; no branching penalty on Cortex-M4.
 *
 * @param  floor_val  Raw floor value.
 * @return            Clamped value in [1, SPI_FLOOR_MAX], or 0 if 0.
 */
static inline uint8_t clamp_floor(uint8_t floor_val)
{
    /* 0 is a legal sentinel ("no floor"); clamp positive values to [1..4]. */
    if ((floor_val > 0U) && (floor_val > SPI_FLOOR_MAX))
    {
        return (uint8_t)SPI_FLOOR_MAX;
    }
    return floor_val;
}

/**
 * @brief Sanitise an FSM state value to a valid wire encoding.
 *
 * If the state is out of range (e.g. uninitialised memory), default to
 * ELEV_FAULT so the receiver knows something is wrong rather than silently
 * acting on garbage.
 *
 * @param  s  Raw state value.
 * @return    Sanitised state cast to uint8_t.
 */
static inline uint8_t sanitise_state(ElevState_t s)
{
    if ((uint8_t)s >= (uint8_t)ELEV_STATE_COUNT)
    {
        return (uint8_t)ELEV_FAULT;
    }
    return (uint8_t)s;
}

/* ─────────────────────────────────────────────────────────────────────────
 * PUBLIC API — SERIALISE
 * ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Serialise a state snapshot into a raw 8-byte transmit buffer.
 *
 * Call pattern from TIM2_IRQHandler (Slave pre-load, 50 ms cadence):
 *
 * @code
 *   SpiStateSnapshot_t snap;
 *
 *   Enter_Critical();
 *   snap.fsm_state     = elev_ctx.state;
 *   snap.current_floor = elev_ctx.floor;
 *   snap.target_floor  = elev_ctx.target_floor;
 *   snap.cabin_queue   = elev_ctx.cabin_queue & SPI_QUEUE_MASK_VALID;
 *   snap.hall_calls    = 0x00U;                       // Slave has none
 *   snap.flags         = flags_byte & SPI_FLAGS_MASK_VALID;
 *   Exit_Critical();
 *
 *   spi_frame_serialise(spi_tx_buf, &snap);
 *
 *   // Pre-load first byte into hardware DR immediately
 *   SPI1->DR = spi_tx_buf[SPI_IDX_HEADER];
 *   SPI1->CR2 |= SPI_CR2_RXNEIE;
 * @endcode
 *
 * @param[out] tx_buf   Raw 8-byte output buffer.  Written entirely.
 * @param[in]  snap     Non-null pointer to a pre-snapshotted state.
 */
void spi_frame_serialise(SpiFrameRaw_t tx_buf, const SpiStateSnapshot_t *snap)
{
    /* ── Byte 0: Header / sync word ─────────────────────────────────────── */
    tx_buf[SPI_IDX_HEADER] = SPI_FRAME_HEADER;

    /* ── Byte 1: FSM state (0–5, sanitised) ─────────────────────────────── */
    tx_buf[SPI_IDX_FSM_STATE] = sanitise_state(snap->fsm_state);

    /* ── Byte 2: FLOOR_POS — upper nibble = target, lower nibble = current ─
     *
     *   [7:4] target_floor  (0 = no target; 1-4 = valid floor)
     *   [3:0] current_floor (1-4)
     *
     *   Both values are clamped to [0..4] so no nibble can bleed into the
     *   other nibble or produce a > 0xFF byte.
     */
    {
        uint8_t cur = clamp_floor(snap->current_floor);
        uint8_t tgt = clamp_floor(snap->target_floor);
        tx_buf[SPI_IDX_FLOOR_POS] = SPI_FLOOR_PACK(cur, tgt);
    }

    /* ── Byte 3: QUEUE_LO — cabin call bitmask ──────────────────────────── */
    /*   Mask off reserved upper nibble defensively. */
    tx_buf[SPI_IDX_QUEUE_LO] = (snap->cabin_queue & SPI_QUEUE_MASK_VALID);

    /* ── Byte 4: HALL_CALLS — hallway call assignments ──────────────────── */
    /*   Master fills bits 0–5; Slave always sends 0x00.                    */
    /*   Mask off undefined bits 6–7.                                       */
    tx_buf[SPI_IDX_HALL_CALLS] = (snap->hall_calls & SPI_HALL_MASK_VALID);

    /* ── Byte 5: FLAGS ───────────────────────────────────────────────────── */
    /*   Bit0=E-STOP, Bit1=DOOR_OPEN, Bit2=FAULT.  Bits 3–7 must be 0.     */
    tx_buf[SPI_IDX_FLAGS] = (snap->flags & SPI_FLAGS_MASK_VALID);

    /* ── Byte 6: RESERVED — always 0x00 ─────────────────────────────────── */
    tx_buf[SPI_IDX_RESERVED] = 0x00U;

    /* ── Byte 7: CHECKSUM — XOR of Bytes 0–6 ────────────────────────────── */
    /*   Inline macro: no loop overhead, constant execution time.           */
    tx_buf[SPI_IDX_CHECKSUM] = SPI_CALC_CHECKSUM(tx_buf);
}

/* ─────────────────────────────────────────────────────────────────────────
 * PUBLIC API — DESERIALISE
 * ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Deserialise a raw received SPI buffer into a decoded frame struct.
 *
 * Validates:
 *   1. Header byte == SPI_FRAME_HEADER (0xA5).
 *   2. XOR of all 8 bytes == 0x00 (checksum integrity).
 *
 * Both conditions must hold for @c out->checksum_ok to be true.
 * The caller MUST discard or handle frames where checksum_ok == false.
 *
 * Call pattern from SPI1_IRQHandler after all 8 bytes received:
 *
 * @code
 *   // In SPI1_IRQHandler, after byte_idx reaches SPI_FRAME_LEN:
 *   spi_rx_ready = 1U;   // volatile flag — consumed by main loop
 *
 *   // In main loop / TIM2 post-processing:
 *   if (spi_rx_ready) {
 *       spi_rx_ready = 0U;
 *       spi_frame_deserialise(&slave_decoded, spi_rx_buf);
 *       if (!slave_decoded.checksum_ok) {
 *           spi_crc_error_ctr++;   // telemetry counter
 *           return;                // discard corrupt frame
 *       }
 *       // Use slave_decoded fields for dispatch logic …
 *   }
 * @endcode
 *
 * @param[out] out      Pointer to destination decoded struct.
 * @param[in]  rx_buf   Source 8-byte received frame buffer.
 */
void spi_frame_deserialise(SpiFrameDecoded_t *out, const SpiFrameRaw_t rx_buf)
{
    /* ── Validate header ─────────────────────────────────────────────────── */
    bool header_ok   = (rx_buf[SPI_IDX_HEADER] == SPI_FRAME_HEADER);

    /* ── Validate checksum: XOR of all 8 bytes must be 0x00 ─────────────── */
    bool checksum_ok = SPI_VERIFY_FRAME(rx_buf);

    out->checksum_ok = (header_ok && checksum_ok);

    /* Always unpack — caller decides whether to use results based on flag.  */
    /* Unpacking invalid data is harmless if the caller checks checksum_ok.  */

    /* ── Byte 1: FSM state ───────────────────────────────────────────────── */
    {
        uint8_t raw_state = rx_buf[SPI_IDX_FSM_STATE];
        /* Bounds-check before casting to enum. */
        if (raw_state < (uint8_t)ELEV_STATE_COUNT)
        {
            out->fsm_state = (ElevState_t)raw_state;
        }
        else
        {
            out->fsm_state = ELEV_FAULT;   /* Unknown value → treat as fault. */
        }
    }

    /* ── Byte 2: FLOOR_POS — unpack nibbles ─────────────────────────────── */
    out->current_floor = SPI_FLOOR_CURRENT(rx_buf[SPI_IDX_FLOOR_POS]);
    out->target_floor  = SPI_FLOOR_TARGET (rx_buf[SPI_IDX_FLOOR_POS]);

    /* ── Byte 3: QUEUE_LO ────────────────────────────────────────────────── */
    out->cabin_queue   = (rx_buf[SPI_IDX_QUEUE_LO]   & SPI_QUEUE_MASK_VALID);

    /* ── Byte 4: HALL_CALLS ──────────────────────────────────────────────── */
    out->hall_calls    = (rx_buf[SPI_IDX_HALL_CALLS]  & SPI_HALL_MASK_VALID);

    /* ── Byte 5: FLAGS ───────────────────────────────────────────────────── */
    out->flags         = (rx_buf[SPI_IDX_FLAGS]        & SPI_FLAGS_MASK_VALID);

    /* Byte 6 (RESERVED) and Byte 7 (CHECKSUM) are not exposed in the
     * decoded struct — CHECKSUM has been consumed above, RESERVED is ignored. */
}

/* ─────────────────────────────────────────────────────────────────────────
 * PUBLIC API — CHECKSUM (out-of-line fallback)
 * ───────────────────────────────────────────────────────────────────────── */

/**
 * @brief Compute XOR checksum over @p len bytes of @p buf.
 *
 * This is the out-of-line equivalent of the @ref SPI_CALC_CHECKSUM macro.
 * Prefer the macro for the fixed 7-byte case.  Use this function when the
 * length is variable (e.g. a future extended frame variant).
 *
 * Execution time: O(len), no heap, no branches — safe from ISR context.
 *
 * @param[in]  buf  Non-null pointer to byte array.
 * @param[in]  len  Number of bytes to XOR together.
 * @return     XOR of all bytes.  Returns 0x00 if len == 0.
 */
uint8_t spi_calc_checksum_fn(const uint8_t *buf, uint8_t len)
{
    uint8_t csum = 0x00U;
    uint8_t i;

    for (i = 0U; i < len; i++)
    {
        csum ^= buf[i];
    }

    return csum;
}
