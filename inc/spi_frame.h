/**
 * @file    spi_frame.h
 * @brief   8-Byte SPI IPC Frame Definition — Dual-Elevator System
 *
 * @details Defines the canonical over-the-wire layout for the Full-Duplex
 *          SPI link between the Master MCU (Dispatcher + Elevator A) and the
 *          Slave MCU (Elevator B).
 *
 *          Wire layout (byte index, 0-based):
 *          ┌──────┬───────────┬───────────┬──────────┬────────────┬───────┬──────────┬──────────┐
 *          │  B0  │    B1     │    B2     │    B3    │     B4     │  B5   │    B6    │    B7    │
 *          │ 0xA5 │ FSM_STATE │ FLOOR_POS │ QUEUE_LO │ HALL_CALLS │ FLAGS │ RESERVED │ CHECKSUM │
 *          └──────┴───────────┴───────────┴──────────┴────────────┴───────┴──────────┴──────────┘
 *
 *          CHECKSUM = XOR of B0..B6.
 *          Receiver verifies: (B0 ^ B1 ^ B2 ^ B3 ^ B4 ^ B5 ^ B6 ^ B7) == 0x00.
 *
 * @note    All fields are uint8_t to guarantee no struct padding and trivial
 *          serialisation — the raw buffer IS the frame.
 *
 * @note    Slave TX pre-load rule:
 *          The Slave's TIM2 ISR (50 ms) builds the frame from the latest
 *          volatile state variables (guarded by Enter_Critical / Exit_Critical)
 *          and pre-loads spi_tx_buf[0] into SPI1->DR BEFORE the Master
 *          asserts CS.  The remaining bytes are shifted out byte-by-byte in
 *          the SPI1_IRQHandler driven by RXNE.
 *
 * @author  Elite Staff Embedded Engineer
 * @version 1.0.0
 */

#ifndef SPI_FRAME_H
#define SPI_FRAME_H

#include <stdint.h>
#include <stdbool.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * COMPILE-TIME CONSTANTS
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Total number of bytes in one SPI IPC frame. */
#define SPI_FRAME_LEN           ((uint8_t)8U)

/** Synchronisation / magic header — Byte 0 of every frame. */
#define SPI_FRAME_HEADER        ((uint8_t)0xA5U)

/** Byte index constants — use these instead of raw literals everywhere. */
#define SPI_IDX_HEADER          (0U)
#define SPI_IDX_FSM_STATE       (1U)
#define SPI_IDX_FLOOR_POS       (2U)
#define SPI_IDX_QUEUE_LO        (3U)
#define SPI_IDX_HALL_CALLS      (4U)
#define SPI_IDX_FLAGS           (5U)
#define SPI_IDX_RESERVED        (6U)
#define SPI_IDX_CHECKSUM        (7U)

/* ═══════════════════════════════════════════════════════════════════════════
 * FSM STATE ENUMERATION  (Byte 1 — FSM_STATE)
 *
 * Stored as uint8_t over the wire.  Values 0-5 are protocol-stable; do NOT
 * reorder without bumping a protocol version byte (currently RESERVED B6).
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Elevator Finite State Machine states.
 *
 * The numeric values are serialised directly into Byte 1 of the SPI frame.
 * They must fit in a uint8_t and must not be changed without updating both
 * MCU firmware images simultaneously.
 */
typedef enum
{
    ELEV_IDLE           = 0U,   /**< Stationary, door closed, no pending requests. */
    ELEV_MOVING_UP      = 1U,   /**< Ascending — motor PWM active. */
    ELEV_MOVING_DOWN    = 2U,   /**< Descending — motor PWM active. */
    ELEV_DOORS_OPEN     = 3U,   /**< At target floor, door-open timer running (TIM4). */
    ELEV_EMSTOP         = 4U,   /**< Emergency-stopped. Highest-priority latch. */
    ELEV_FAULT          = 5U,   /**< SPI comms lost; running in independent/safe mode. */

    /* Sentinel — not transmitted; used for range checks only. */
    ELEV_STATE_COUNT    = 6U
} ElevState_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * BYTE 2 — FLOOR_POS  (nibble-packed)
 *
 *   [7:4]  target_floor  (1-4, or 0 = none)
 *   [3:0]  current_floor (1-4)
 *
 * Valid floor range: 1..SPI_FLOOR_MAX.  0 in either nibble means "unknown".
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SPI_FLOOR_MAX               (4U)    /**< Building has 4 floors. */

/** Extract current floor from a packed FLOOR_POS byte. */
#define SPI_FLOOR_CURRENT(byte)     ((uint8_t)((byte) & 0x0FU))

/** Extract target floor from a packed FLOOR_POS byte. */
#define SPI_FLOOR_TARGET(byte)      ((uint8_t)(((byte) >> 4U) & 0x0FU))

/**
 * @brief Pack current and target floor into a single FLOOR_POS byte.
 *
 * @param cur   Current floor (1–4).
 * @param tgt   Target floor  (1–4, or 0 if none).
 * @return      Packed byte: upper nibble = tgt, lower nibble = cur.
 */
#define SPI_FLOOR_PACK(cur, tgt)    ((uint8_t)(((uint8_t)(tgt) << 4U) | ((uint8_t)(cur) & 0x0FU)))

/* ═══════════════════════════════════════════════════════════════════════════
 * BYTE 3 — QUEUE_LO  (cabin call queue bitmask)
 *
 *   Bit 0 → Floor 1 pending
 *   Bit 1 → Floor 2 pending
 *   Bit 2 → Floor 3 pending
 *   Bit 3 → Floor 4 pending
 *   Bits 4–7 → Reserved (must be 0)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SPI_QUEUE_FLOOR1            ((uint8_t)(1U << 0U))   /**< Cabin call: Floor 1. */
#define SPI_QUEUE_FLOOR2            ((uint8_t)(1U << 1U))   /**< Cabin call: Floor 2. */
#define SPI_QUEUE_FLOOR3            ((uint8_t)(1U << 2U))   /**< Cabin call: Floor 3. */
#define SPI_QUEUE_FLOOR4            ((uint8_t)(1U << 3U))   /**< Cabin call: Floor 4. */
#define SPI_QUEUE_MASK_VALID        ((uint8_t)0x0FU)        /**< Only lower nibble is valid. */

/** Encode a floor number (1-based) into its queue bit position. */
#define SPI_QUEUE_BIT(floor_1based) ((uint8_t)(1U << ((uint8_t)(floor_1based) - 1U)))

/** Test whether a specific floor is set in a queue byte. */
#define SPI_QUEUE_IS_SET(queue_byte, floor_1based) \
    (((queue_byte) & SPI_QUEUE_BIT(floor_1based)) != 0U)

/* ═══════════════════════════════════════════════════════════════════════════
 * BYTE 4 — HALL_CALLS  (hallway call bitmask — Master transmits live state)
 *
 *   Bit 0 → U1  (Up   call at Floor 1)
 *   Bit 1 → D2  (Down call at Floor 2)
 *   Bit 2 → U2  (Up   call at Floor 2)
 *   Bit 3 → D3  (Down call at Floor 3)
 *   Bit 4 → U3  (Up   call at Floor 3)
 *   Bit 5 → D4  (Down call at Floor 4)
 *   Bits 6–7 → Reserved (must be 0)
 *
 * When the Master assigns a hallway call to Elevator B (Slave), it sets the
 * corresponding bit here.  The Slave clears the bit after it acknowledges the
 * assignment in the next frame.
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SPI_HALL_U1                 ((uint8_t)(1U << 0U))   /**< Up   at Floor 1. */
#define SPI_HALL_D2                 ((uint8_t)(1U << 1U))   /**< Down at Floor 2. */
#define SPI_HALL_U2                 ((uint8_t)(1U << 2U))   /**< Up   at Floor 2. */
#define SPI_HALL_D3                 ((uint8_t)(1U << 3U))   /**< Down at Floor 3. */
#define SPI_HALL_U3                 ((uint8_t)(1U << 4U))   /**< Up   at Floor 3. */
#define SPI_HALL_D4                 ((uint8_t)(1U << 5U))   /**< Down at Floor 4. */
#define SPI_HALL_MASK_VALID         ((uint8_t)0x3FU)        /**< Bits 0–5 are valid. */

/* ═══════════════════════════════════════════════════════════════════════════
 * BYTE 5 — FLAGS  (system status bitfield)
 *
 *   Bit 0 → E_STOP   : Emergency stop is active (latched until cleared by operator).
 *   Bit 1 → DOOR     : Door is physically open (sensor feedback or door-timer active).
 *   Bit 2 → FAULT    : SPI communication fault declared (sender is in independent mode).
 *   Bits 3–7 → Reserved (must be 0; receiver must ignore).
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SPI_FLAG_ESTOP              ((uint8_t)(1U << 0U))   /**< Emergency stop active. */
#define SPI_FLAG_DOOR_OPEN          ((uint8_t)(1U << 1U))   /**< Door is open. */
#define SPI_FLAG_FAULT              ((uint8_t)(1U << 2U))   /**< Comm/system fault. */
#define SPI_FLAGS_MASK_VALID        ((uint8_t)0x07U)        /**< Only bits 0–2 defined. */

/** Test individual flag bits cleanly. */
#define SPI_FLAG_IS_SET(flags_byte, flag_mask)  (((flags_byte) & (flag_mask)) != 0U)

/* ═══════════════════════════════════════════════════════════════════════════
 * RAW FRAME BUFFER TYPE
 *
 * A plain uint8_t[8] is the canonical "on-the-wire" representation.
 * No struct padding can occur — each byte maps directly to the SPI DR.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Typedef for the raw 8-byte SPI frame buffer.
 *
 * This is what is loaded into / read from the SPI data register.
 * Treat this as an opaque byte array; use the accessor macros above and
 * the serialisation functions in spi_frame.c to read/write fields.
 */
typedef uint8_t SpiFrameRaw_t[SPI_FRAME_LEN];

/* ═══════════════════════════════════════════════════════════════════════════
 * DECODED FRAME VIEW  (logical, unpacked representation)
 *
 * Used by application logic after deserialisation.  Never transmitted as-is.
 * Kept separate from the raw buffer to make padding behaviour irrelevant.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Logical, unpacked view of a received or pending SPI frame.
 *
 * Fields are populated by @ref spi_frame_deserialise() and consumed by the
 * Master dispatcher.  The struct may have internal padding — do not memcpy
 * it directly to/from a SPI buffer.
 */
typedef struct
{
    ElevState_t fsm_state;      /**< Sender's current FSM state (0–5). */
    uint8_t     current_floor;  /**< Current floor (1–4). */
    uint8_t     target_floor;   /**< Target floor  (1–4, or 0 = none). */
    uint8_t     cabin_queue;    /**< Pending cabin calls bitmask (bits 0–3). */
    uint8_t     hall_calls;     /**< Active/assigned hallway calls (bits 0–5). */
    uint8_t     flags;          /**< Status flags: E-STOP, DOOR, FAULT. */
    bool        checksum_ok;    /**< true if received checksum validated. */
} SpiFrameDecoded_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * SYSTEM STATE SNAPSHOT  (input to serialiser)
 *
 * Aggregates all volatile ISR-updated variables that the serialiser needs.
 * The caller must snapshot these under Enter_Critical() before passing here.
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Caller-supplied snapshot of the current elevator state.
 *
 * The serialiser (@ref spi_frame_serialise) takes a pointer to this struct.
 * The caller is responsible for atomically copying volatile variables into
 * this struct under Enter_Critical() / Exit_Critical() before calling.
 */
typedef struct
{
    ElevState_t fsm_state;      /**< Copy of volatile elev_ctx.state. */
    uint8_t     current_floor;  /**< Copy of volatile elev_ctx.floor. */
    uint8_t     target_floor;   /**< Copy of volatile elev_ctx.target_floor. */
    uint8_t     cabin_queue;    /**< Copy of volatile elev_ctx.cabin_queue. */
    uint8_t     hall_calls;     /**< Hall calls bitmask (Master: live; Slave: 0x00). */
    uint8_t     flags;          /**< Assembled flags byte (E-STOP | DOOR | FAULT). */
} SpiStateSnapshot_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * FAST CHECKSUM MACRO  (inline XOR, zero function-call overhead)
 *
 * Usage:
 *   uint8_t cs = SPI_CALC_CHECKSUM(buf);
 *
 * Verifier idiom (XOR all 8 bytes including checksum must equal 0x00):
 *   bool ok = SPI_VERIFY_FRAME(buf);
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Compute XOR checksum over Bytes 0–6 of a raw frame buffer.
 *
 * Expands to a single expression — no loop, no function call.
 * Safe to call from ISR context.
 *
 * @param _buf  uint8_t array of at least SPI_FRAME_LEN bytes.
 * @return      uint8_t XOR of bytes [0]..[6].
 */
#define SPI_CALC_CHECKSUM(_buf)                          \
    ((uint8_t)(  (_buf)[SPI_IDX_HEADER]                  \
               ^ (_buf)[SPI_IDX_FSM_STATE]               \
               ^ (_buf)[SPI_IDX_FLOOR_POS]               \
               ^ (_buf)[SPI_IDX_QUEUE_LO]                \
               ^ (_buf)[SPI_IDX_HALL_CALLS]              \
               ^ (_buf)[SPI_IDX_FLAGS]                   \
               ^ (_buf)[SPI_IDX_RESERVED] ))

/**
 * @brief Verify a complete received frame (all 8 bytes XOR to 0x00).
 *
 * @param _buf  uint8_t array of SPI_FRAME_LEN bytes (including checksum).
 * @return      Non-zero (true) if frame is intact; 0 (false) if corrupt.
 */
#define SPI_VERIFY_FRAME(_buf)                           \
    ((SPI_CALC_CHECKSUM(_buf) ^ (_buf)[SPI_IDX_CHECKSUM]) == 0x00U)

/* ═══════════════════════════════════════════════════════════════════════════
 * FUNCTION DECLARATIONS
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Serialise a state snapshot into a raw 8-byte transmit buffer.
 *
 * @param[out] tx_buf   Destination raw frame buffer (must be SPI_FRAME_LEN bytes).
 * @param[in]  snap     Pointer to a pre-atomically-read state snapshot.
 *
 * @note  This function does NOT enter a critical section itself.  The caller
 *        must snapshot volatile variables under Enter_Critical() first.
 */
void spi_frame_serialise(SpiFrameRaw_t tx_buf, const SpiStateSnapshot_t *snap);

/**
 * @brief Deserialise a raw received buffer into a decoded logical frame.
 *
 * Validates the header and checksum.  Sets @c out->checksum_ok accordingly.
 * The caller should discard the frame if @c checksum_ok is false.
 *
 * @param[out] out      Destination decoded frame struct.
 * @param[in]  rx_buf   Source raw frame buffer (SPI_FRAME_LEN bytes).
 */
void spi_frame_deserialise(SpiFrameDecoded_t *out, const SpiFrameRaw_t rx_buf);

/**
 * @brief Compute XOR checksum over bytes 0–(len-1) of an arbitrary buffer.
 *
 * Provided as an out-of-line fallback for callers that prefer a function
 * call.  For the fixed-length 7-byte case, prefer the @ref SPI_CALC_CHECKSUM
 * macro which generates zero-overhead inline code.
 *
 * @param[in]  buf  Pointer to byte array.
 * @param[in]  len  Number of bytes to XOR.
 * @return     XOR result.
 */
uint8_t spi_calc_checksum_fn(const uint8_t *buf, uint8_t len);

#endif /* SPI_FRAME_H */
