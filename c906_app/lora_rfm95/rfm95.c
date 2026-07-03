/**
 * @file rfm95.c
 * @brief RFM95(W) LoRa transceiver driver — platform-independent implementation.
 *
 * Ported from STM32 HAL.  All platform interaction goes through the
 * function pointers in rfm95_handle_t.
 */
#include "rfm95.h"

#include <assert.h>
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Low-level SPI helpers                                              */
/* ------------------------------------------------------------------ */

static bool read_register(rfm95_handle_t *handle, rfm95_register_t reg,
                          uint8_t *buffer, size_t length)
{
    /* Single full-duplex transfer: clock out [addr, 0x00, 0x00, ...] while
       the module returns [<undefined>, reg[0], reg[1], ...] on MISO.
       The register data therefore lands in rx[1 .. length]. */
    uint8_t tx[1 + 252] = {0};   /* addr + up to one max-length FIFO read */
    uint8_t rx[1 + 252];

    if (length + 1 > sizeof(tx)) {
        return false;
    }

    tx[0] = (uint8_t)reg & 0x7Fu;   /* MSB=0 -> read */

    handle->gpio_write_nss(0);
    bool ok = handle->spi_transfer(tx, rx, length + 1);
    handle->gpio_write_nss(1);

    if (ok) {
        memcpy(buffer, &rx[1], length);
    }
    return ok;
}

static bool write_register(rfm95_handle_t *handle, rfm95_register_t reg,
                           uint8_t value)
{
    handle->gpio_write_nss(0);

    uint8_t tx[2] = { (uint8_t)reg | 0x80u, value };
    bool ok = handle->spi_transfer(tx, NULL, 2);

    handle->gpio_write_nss(1);
    return ok;
}

/* ------------------------------------------------------------------ */
/*  Reset                                                              */
/* ------------------------------------------------------------------ */

static void reset(rfm95_handle_t *handle)
{
    handle->gpio_write_nrst(0);
    handle->delay_ms(3);
    handle->gpio_write_nrst(1);
    handle->delay_ms(6);
}

/* ------------------------------------------------------------------ */
/*  Frequency configuration                                            */
/* ------------------------------------------------------------------ */

static bool configure_frequency(rfm95_handle_t *handle, uint32_t frequency)
{
    /* FQ = (FRF * 32 MHz) / (2^19) */
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    if (!write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
    if (!write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8)))  return false;
    if (!write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0)))  return false;

    return true;
}

/* ------------------------------------------------------------------ */
/*  Wait for DIO interrupt with timeout                                */
/* ------------------------------------------------------------------ */

static bool wait_for_irq(rfm95_handle_t *handle, rfm95_interrupt_t interrupt,
                         uint32_t timeout_ms)
{
    uint64_t deadline_us = handle->get_tick_us() + (uint64_t)timeout_ms * 1000u;

    if (handle->poll_dio0 != NULL) {
        /* Polling mode — used when hardware GPIO interrupts are unavailable
           (e.g. BL808 D0/c906 core has no GPIO_INT0_IRQn). */
        while (!handle->poll_dio0()) {
            if (handle->get_tick_us() >= deadline_us) {
                return false;
            }
        }
    } else {
        /* ISR mode — interrupt handler sets interrupt_times[] via
           rfm95_on_interrupt(). */
        while (handle->interrupt_times[interrupt] == 0) {
            if (handle->get_tick_us() >= deadline_us) {
                return false;
            }
        }
    }

    return true;
}


/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power)
{
    assert((power >= 2 && power <= 17) || power == 20);

    rfm95_register_pa_config_t pa_config = {0};
    uint8_t pa_dac_config = 0;

    if (power >= 2 && power <= 17) {
        pa_config.max_power   = 7;
        pa_config.pa_select   = 1;
        pa_config.output_power = (power - 2);
        pa_dac_config = RFM95_REGISTER_PA_DAC_LOW_POWER;

    } else if (power == 20) {
        pa_config.max_power   = 7;
        pa_config.pa_select   = 1;
        pa_config.output_power = 15;
        pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
    }

    if (!write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
    if (!write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config))       return false;

    return true;
}

bool rfm95_init(rfm95_handle_t *handle)
{
    assert(handle->spi_transfer      != NULL);
    assert(handle->gpio_write_nss    != NULL);
    assert(handle->gpio_write_nrst   != NULL);
    assert(handle->delay_ms          != NULL);
    assert(handle->get_tick_us       != NULL);

    reset(handle);

    /* Check for correct version. */
    uint8_t version;

    if (!read_register(handle, RFM95_REGISTER_VERSION, &version, 1)) return false;
    if (version != RFM9x_VER) return false;

    /* Module must be placed in sleep mode before switching to LoRa. */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_SLEEP))      return false;
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

    /* Set standard frequency 868.0 MHz */
    configure_frequency(handle, 868000000);

    /* Default interrupt configuration — prevents DIO5 clock interrupts at 1 MHz */
    if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1,
                        RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;

    if (handle->on_after_interrupts_configured != NULL) {
        handle->on_after_interrupts_configured();
    }
    /* Set module power to 20 dBm */
    if (!rfm95_set_power(handle, 20)) return false;

    /* Set LNA to the highest gain with 150% boost */
    if (!write_register(handle, RFM95_REGISTER_LNA, 0x23)) return false;
    /* Preamble set to 8 */
    if (!write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;
    if (!write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x08)) return false;
    /* Set standard sync word 0x12 */
    if (!write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x12)) return false;
    /* Set up TX and RX FIFO base addresses. */
    if (!write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x80)) return false;
    if (!write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;

    /* Maximum payload length */
    if (!write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 252)) return false;

    /* Configure modem: 125 kHz BW, 4/8 CR, SF8, CRC on, AGC auto */
    if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1,
                        (RFM95_BW_125 << 4) | (RFM95_CR_48 << 1))) return false;
    if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2,
                        (RFM95_SF_8 << 4) | (RFM95_CRC_ON << 2))) return false;
    if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3,
                        (RFM95_AGC_AUTO << 2))) return false;

    /* Let module sleep after initialisation. */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

    return true;
}

bool rfm95_send_package(rfm95_handle_t *handle, const uint8_t *send_data, size_t send_data_length)
{
    if (send_data_length > 248) return false;

    uint8_t payload_buf[252];

    /* RadioHead format header */
    payload_buf[0] = 0xff;  /* Destination */
    payload_buf[1] = 0xff;  /* Node */
    payload_buf[2] = 0x00;  /* Identifier */
    payload_buf[3] = 0x00;  /* Flags */

    memcpy(payload_buf + 4, send_data, send_data_length);
    size_t payload_len = send_data_length + 4;

    /* Set IQ registers for standard node-to-node. */
    if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
    if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

    /* Set the payload length. */
    if (!write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, payload_len)) return false;

    /* Enable tx-done interrupt, clear flags and previous interrupt time. */
    if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1,
                        RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return false;
    if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
    handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;

    /* Move modem to LoRa standby. */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;

    /* Set pointer to start of TX section in FIFO. */
    if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0x80)) return false;

    /* Write payload to FIFO. */
    for (size_t i = 0; i < payload_len; i++) {
        write_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf[i]);
    }

    /* Set modem to tx mode. */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_TX)) return false;

    /* Wait for the transfer complete interrupt. */
    if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, RFM95_SEND_TIMEOUT)) return false;

    /* Return modem to sleep. */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

    return true;
}

bool rfm95_receive_package(rfm95_handle_t *handle, uint8_t *receive_data,
                           size_t *receive_data_length, int8_t *snr)
{
    *receive_data_length = 0;

    /* Set IQ registers for standard node-to-node. */
    if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
    if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

    /* Clear flags and previous interrupt time, configure mapping for RX done. */
    if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1,
                        RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;
    if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
    handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;

    /* Enter RX Continuous mode */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_RX_CONT)) return false;

    /* Wait for RX done interrupt */
    if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, RFM95_RECEIVE_TIMEOUT)) {
        write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
        return false;
    }

    uint8_t irq_flags;
    read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irq_flags, 1);

    /* Check if there was a CRC error. */
    if (irq_flags & 0x20) {
        write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
        return false;
    }

    int8_t packet_snr;
    if (!read_register(handle, RFM95_REGISTER_PACKET_SNR, (uint8_t *)&packet_snr, 1)) return false;
    if (snr) *snr = (int8_t)(packet_snr / 4);

    /* Read received payload length. */
    uint8_t payload_len_internal;
    if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &payload_len_internal, 1)) return false;

    /* Read received payload. */
    uint8_t rx_current_addr;
    if (!read_register(handle, RFM95_REGISTER_FIFO_RX_CURRENT_ADDR, &rx_current_addr, 1)) return false;
    if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, rx_current_addr)) return false;

    uint8_t payload_buf[252];
    if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf, payload_len_internal)) return false;

    /* Return modem to sleep. */
    if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

    /* Decode RadioHead header */
    if (payload_len_internal >= 4) {
        *receive_data_length = payload_len_internal - 4;
        memcpy(receive_data, payload_buf + 4, *receive_data_length);
        return true;
    }

    return false;
}

void rfm95_on_interrupt(rfm95_handle_t *handle, rfm95_interrupt_t interrupt)
{
    handle->interrupt_times[interrupt] = (uint32_t)handle->get_tick_us();
}
