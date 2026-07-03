/**
 * @file bl808_rfm95_hal.c
 * @brief BL808 hardware abstraction layer implementation for RFM95.
 *
 * Uses:
 *   - hosal_spi for SPI1 master (full-duplex, mode 0, ~5 MHz)
 *   - GLB_GPIO for output pins (NSS, NRST) and DIO0 input (polled)
 *   - CPU_MTimer / CPU_Get_MTimer_US for timing
 *
 * Note: The D0/c906 core does NOT have a GPIO interrupt vector
 * (GPIO_INT0_IRQn is only available on the M0/E907 core).
 * DIO0 is therefore polled by the driver's wait_for_irq() loop.
 */
#include "bl808_rfm95_hal.h"

#include <stdio.h>
#include <string.h>

#include <hosal_spi.h>
#include <hosal_dma.h>
#include <bl808_glb.h>
#include <bl808_glb_gpio.h>
#include <bl808_clock.h>

/* ------------------------------------------------------------------ */
/*  Static state                                                       */
/* ------------------------------------------------------------------ */
static hosal_spi_dev_t  rfm95_spi = {0};
static rfm95_handle_t   rfm95_handle = {0};

/* ------------------------------------------------------------------ */
/*  GPIO helpers                                                       */
/* ------------------------------------------------------------------ */

static void gpio_write_nss(uint8_t value)
{
    GLB_GPIO_Write(RFM95_PIN_NSS, value);
}

static void gpio_write_nrst(uint8_t value)
{
    GLB_GPIO_Write(RFM95_PIN_NRST, value);
}

/**
 * Poll the DIO0 input pin.
 * Returns true when DIO0 is HIGH (RFM95 signals TX-done or RX-done).
 */
static bool poll_dio0(void)
{
    return GLB_GPIO_Read(RFM95_PIN_DIO0) != 0;
}

/* ------------------------------------------------------------------ */
/*  SPI helper                                                         */
/* ------------------------------------------------------------------ */

static bool spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    /*
     * hosal_spi_send_recv does full-duplex transfer.
     * If tx is NULL we still need to clock out dummy bytes;
     * if rx is NULL the received data is simply discarded.
     */
    uint8_t tx_buf[256];
    uint8_t rx_buf[256];

    if (len > sizeof(tx_buf)) {
        return false;  /* safety guard */
    }

    if (tx) {
        memcpy(tx_buf, tx, len);
    } else {
        memset(tx_buf, 0x00, len);
    }

    int ret = hosal_spi_send_recv(&rfm95_spi, tx_buf, rx_buf, (uint16_t)len,
                                  RFM95_SPI_TIMEOUT);

    if (rx && ret == 0) {
        memcpy(rx, rx_buf, len);
    }

    return ret == 0;
}

/* ------------------------------------------------------------------ */
/*  Timing helpers                                                     */
/* ------------------------------------------------------------------ */

static void delay_ms(uint32_t ms)
{
    CPU_MTimer_Delay_MS(ms);
}

static uint64_t get_tick_us(void)
{
    return CPU_Get_MTimer_US();
}

/* ------------------------------------------------------------------ */
/*  GPIO initialisation                                                */
/* ------------------------------------------------------------------ */

static void gpio_init(void)
{
    GLB_GPIO_Cfg_Type cfg;
    cfg.pullType   = GPIO_PULL_NONE;
    cfg.drive      = 0;
    cfg.smtCtrl    = 1;

    /* ----- NSS output (active low, start HIGH) ----- */
    cfg.gpioPin     = RFM95_PIN_NSS;
    cfg.gpioMode    = GPIO_MODE_OUTPUT;
    cfg.gpioFun     = GPIO_FUN_GPIO;
    cfg.outputMode  = GPIO_OUTPUT_VALUE_MODE;
    GLB_GPIO_Init(&cfg);
    GLB_GPIO_Write(RFM95_PIN_NSS, 1);

    /* ----- NRST output (active low, start HIGH) ----- */
    cfg.gpioPin     = RFM95_PIN_NRST;
    GLB_GPIO_Init(&cfg);
    GLB_GPIO_Write(RFM95_PIN_NRST, 1);

    /* ----- DIO0 input (polled — no IRQ on D0/c906 core) ----- */
    cfg.gpioPin     = RFM95_PIN_DIO0;
    cfg.gpioMode    = GPIO_MODE_INPUT;
    cfg.gpioFun     = GPIO_FUN_GPIO;
    cfg.pullType    = GPIO_PULL_DOWN;
    GLB_GPIO_Init(&cfg);
}

/* ------------------------------------------------------------------ */
/*  SPI initialisation                                                 */
/* ------------------------------------------------------------------ */

static int spi_init(void)
{
    hosal_dma_init();

    rfm95_spi.port               = 1;  /* SPI1 (MM_SPI) on D0 core! */
    rfm95_spi.config.mode        = HOSAL_SPI_MODE_MASTER;
    rfm95_spi.config.dma_enable  = 0;  /* polled mode for short transfers */
    rfm95_spi.config.polar_phase = 0;  /* CPOL=0, CPHA=0 (SPI mode 0) */
    rfm95_spi.config.freq        = RFM95_SPI_FREQ;
    rfm95_spi.config.pin_clk    = RFM95_PIN_SCLK;
    rfm95_spi.config.pin_mosi   = RFM95_PIN_MOSI;
    rfm95_spi.config.pin_miso   = RFM95_PIN_MISO;

    int ret = hosal_spi_init(&rfm95_spi);
    if (ret != 0) {
        return ret;
    }

    GLB_GPIO_Cfg_Type spi_pin_cfg;
    spi_pin_cfg.pullType   = GPIO_PULL_NONE;
    spi_pin_cfg.drive      = 1;
    spi_pin_cfg.smtCtrl    = 1;
    spi_pin_cfg.gpioMode   = GPIO_MODE_AF;
    spi_pin_cfg.gpioFun    = GPIO_FUN_SPI1;

    uint8_t spi_pins[] = { RFM95_PIN_SCLK, RFM95_PIN_MOSI, RFM95_PIN_MISO };
    for (int i = 0; i < 3; i++) {
        spi_pin_cfg.gpioPin = spi_pins[i];
        GLB_GPIO_Init(&spi_pin_cfg);
    }

    return 0;
}


/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

rfm95_handle_t *bl808_rfm95_hal_init(void)
{
    memset(&rfm95_handle, 0, sizeof(rfm95_handle));

    /* Wire platform callbacks */
    rfm95_handle.spi_transfer      = spi_transfer;
    rfm95_handle.gpio_write_nss    = gpio_write_nss;
    rfm95_handle.gpio_write_nrst   = gpio_write_nrst;
    rfm95_handle.delay_ms          = delay_ms;
    rfm95_handle.get_tick_us       = get_tick_us;
    rfm95_handle.poll_dio0         = poll_dio0;
    rfm95_handle.on_after_interrupts_configured = NULL;

    gpio_init();

    if (spi_init() != 0) {
        printf("[RFM95 HAL] SPI init failed!\r\n");
        return NULL;
    }

    printf("[RFM95 HAL] BL808 peripherals initialised (SPI1 @ %u Hz, DIO0 polled)\r\n",
           (unsigned)RFM95_SPI_FREQ);

    return &rfm95_handle;
}

void bl808_rfm95_hal_deinit(void)
{
    hosal_spi_finalize(&rfm95_spi);
}
