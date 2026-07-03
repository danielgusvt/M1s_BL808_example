/**
 * @file rfm95.h
 * @brief RFM95(W) LoRa transceiver driver — platform-independent header.
 *
 * Ported from STM32 HAL to a callback-based HAL so the same driver
 * works on BL808 (or any other platform) by simply wiring the
 * function pointers in rfm95_handle_t.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/*  Compile-time tunables                                              */
/* ------------------------------------------------------------------ */
#ifndef RFM95_SPI_TIMEOUT
#define RFM95_SPI_TIMEOUT 10  /* ms */
#endif

#ifndef RFM95_SEND_TIMEOUT
#define RFM95_SEND_TIMEOUT 2000  /* ms */
#endif

#ifndef RFM95_RECEIVE_TIMEOUT
#define RFM95_RECEIVE_TIMEOUT 2000  /* ms */
#endif

#define RFM9x_VER 0x12

/* ------------------------------------------------------------------ */
/*  Register map                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    RFM95_REGISTER_FIFO_ACCESS         = 0x00,
    RFM95_REGISTER_OP_MODE             = 0x01,
    RFM95_REGISTER_FR_MSB              = 0x06,
    RFM95_REGISTER_FR_MID              = 0x07,
    RFM95_REGISTER_FR_LSB              = 0x08,
    RFM95_REGISTER_PA_CONFIG           = 0x09,
    RFM95_REGISTER_LNA                 = 0x0C,
    RFM95_REGISTER_FIFO_ADDR_PTR      = 0x0D,
    RFM95_REGISTER_FIFO_TX_BASE_ADDR  = 0x0E,
    RFM95_REGISTER_FIFO_RX_BASE_ADDR  = 0x0F,
    RFM95_REGISTER_FIFO_RX_CURRENT_ADDR = 0x10,
    RFM95_REGISTER_IRQ_FLAGS           = 0x12,
    RFM95_REGISTER_FIFO_RX_BYTES_NB   = 0x13,
    RFM95_REGISTER_PACKET_SNR          = 0x19,
    RFM95_REGISTER_MODEM_CONFIG_1      = 0x1D,
    RFM95_REGISTER_MODEM_CONFIG_2      = 0x1E,
    RFM95_REGISTER_SYMB_TIMEOUT_LSB   = 0x1F,
    RFM95_REGISTER_PREAMBLE_MSB        = 0x20,
    RFM95_REGISTER_PREAMBLE_LSB        = 0x21,
    RFM95_REGISTER_PAYLOAD_LENGTH      = 0x22,
    RFM95_REGISTER_MAX_PAYLOAD_LENGTH  = 0x23,
    RFM95_REGISTER_MODEM_CONFIG_3      = 0x26,
    RFM95_REGISTER_INVERT_IQ_1         = 0x33,
    RFM95_REGISTER_SYNC_WORD           = 0x39,
    RFM95_REGISTER_INVERT_IQ_2         = 0x3B,
    RFM95_REGISTER_DIO_MAPPING_1       = 0x40,
    RFM95_REGISTER_VERSION             = 0x42,
    RFM95_REGISTER_PA_DAC              = 0x4D
} rfm95_register_t;

/* ------------------------------------------------------------------ */
/*  PA config bitfield (identical to the original)                     */
/* ------------------------------------------------------------------ */
typedef struct {
    union {
        struct {
            uint8_t output_power : 4;
            uint8_t max_power    : 3;
            uint8_t pa_select    : 1;
        };
        uint8_t buffer;
    };
} rfm95_register_pa_config_t;

/* ------------------------------------------------------------------ */
/*  Op-mode constants                                                  */
/* ------------------------------------------------------------------ */
#define RFM95_REGISTER_OP_MODE_SLEEP                  0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP             0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY           0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                0x83
#define RFM95_REGISTER_OP_MODE_LORA_RX_CONT           0x85
#define RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE         0x86

#define RFM95_REGISTER_PA_DAC_LOW_POWER               0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER              0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE   0x40
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE   0x00

#define RFM95_REGISTER_INVERT_IQ_1_TX                 0x27
#define RFM95_REGISTER_INVERT_IQ_2_TX                 0x1d
#define RFM95_REGISTER_INVERT_IQ_1_RX                 0x67
#define RFM95_REGISTER_INVERT_IQ_2_RX                 0x19

/* ------------------------------------------------------------------ */
/*  Modem configuration helpers                                        */
/* ------------------------------------------------------------------ */
/* Signal bandwidth (shifted left 4 in modem config 1) */
#define RFM95_BW_78    0
#define RFM95_BW_104   1
#define RFM95_BW_156   2
#define RFM95_BW_208   3
#define RFM95_BW_3125  4
#define RFM95_BW_417   5
#define RFM95_BW_625   6
#define RFM95_BW_125   7
#define RFM95_BW_250   8
#define RFM95_BW_500   9

/* Coding rate (shifted left 1 in modem config 1) */
#define RFM95_CR_45  1
#define RFM95_CR_46  2
#define RFM95_CR_47  3
#define RFM95_CR_48  4

/* Implicit header */
#define RFM95_IMP_HEA  1

/* Spreading factor (shifted left 4 in modem config 2) */
#define RFM95_SF_6   6
#define RFM95_SF_7   7
#define RFM95_SF_8   8
#define RFM95_SF_9   9
#define RFM95_SF_10  10
#define RFM95_SF_11  11
#define RFM95_SF_12  12

/* Tx continuous mode (shifted left 3) */
#define RFM95_TX_CONT   1
/* Rx payload CRC on (shifted left 2) */
#define RFM95_CRC_ON    1

/* Mobile/static node (shifted left 3) */
#define RFM95_STATIC_ND  0
#define RFM95_MOBILE_ND  1
/* AGC auto on (shifted left 2) */
#define RFM95_AGC_AUTO   1

/* ------------------------------------------------------------------ */
/*  HAL callback signatures                                            */
/* ------------------------------------------------------------------ */

/** SPI transfer: send tx_data while receiving into rx_data (full-duplex).
 *  Either pointer may be NULL for send-only / receive-only.
 *  Returns true on success. */
typedef bool (*rfm95_spi_transfer_fn)(const uint8_t *tx_data,
                                      uint8_t *rx_data,
                                      size_t length);

/** Write a GPIO output pin (nss / nrst). value: 0=low, 1=high. */
typedef void (*rfm95_gpio_write_fn)(uint8_t value);

/** Read the DIO0 pin state.  Returns true when DIO0 is HIGH (interrupt asserted).
 *  Used for polling when hardware GPIO interrupts are not available (e.g. BL808 D0). */
typedef bool (*rfm95_poll_dio0_fn)(void);

/** Blocking delay in milliseconds. */
typedef void (*rfm95_delay_ms_fn)(uint32_t ms);

/** Return a monotonic microsecond timestamp (wraps are OK). */
typedef uint64_t (*rfm95_get_tick_us_fn)(void);

/** Optional callback after interrupts are configured. */
typedef void (*rfm95_on_after_interrupts_configured_fn)(void);

/* ------------------------------------------------------------------ */
/*  Interrupt IDs                                                      */
/* ------------------------------------------------------------------ */
typedef enum {
    RFM95_INTERRUPT_DIO0 = 0
} rfm95_interrupt_t;

#define RFM95_INTERRUPT_COUNT 3

/* ------------------------------------------------------------------ */
/*  Handle                                                             */
/* ------------------------------------------------------------------ */
typedef struct {
    /* --- Platform callbacks (must be set before rfm95_init) --- */
    rfm95_spi_transfer_fn                    spi_transfer;
    rfm95_gpio_write_fn                      gpio_write_nss;
    rfm95_gpio_write_fn                      gpio_write_nrst;
    rfm95_delay_ms_fn                        delay_ms;
    rfm95_get_tick_us_fn                     get_tick_us;
    rfm95_on_after_interrupts_configured_fn  on_after_interrupts_configured;

    /** Poll DIO0 pin — set this when hardware GPIO interrupts are not
     *  available (e.g. BL808 D0/c906 core).  If non-NULL, wait_for_irq()
     *  will poll this instead of checking interrupt_times[]. */
    rfm95_poll_dio0_fn                       poll_dio0;

    /* --- Internal state (set by driver / ISR) --- */
    volatile uint32_t interrupt_times[RFM95_INTERRUPT_COUNT];
} rfm95_handle_t;

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */
bool rfm95_init(rfm95_handle_t *handle);
bool rfm95_set_power(rfm95_handle_t *handle, int8_t power);
bool rfm95_send_package(rfm95_handle_t *handle, const uint8_t *data, size_t length);
bool rfm95_receive_package(rfm95_handle_t *handle, uint8_t *data, size_t *length, int8_t *snr);

/**
 * Call this from the DIO0 external interrupt handler.
 */
void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt);
