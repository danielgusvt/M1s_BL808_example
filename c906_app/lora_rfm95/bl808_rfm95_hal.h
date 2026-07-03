/**
 * @file bl808_rfm95_hal.h
 * @brief BL808 hardware abstraction layer for the RFM95 LoRa module.
 *
 * Initialises SPI1, GPIO outputs (NSS, NRST), and GPIO input (DIO0, polled)
 * on the D0/C906 core.
 *
 * Note: The D0/c906 core does NOT have a GPIO interrupt vector
 * (GPIO_INT0_IRQn exists only on M0/E907), so DIO0 is polled.
 */
#pragma once

#include "rfm95.h"
#include <bl808_glb.h>

/*  Pin assignments
 * IMPORTANT — BL808 SPI pad roles are fixed by (pin % 4), NOT freely
 * assignable.  GPIO_FUN_SPI* only attaches the pad to the SPI block; the
 * role is decided in silicon:
 *      pin % 4 == 0 -> CS/SS
 *      pin % 4 == 1 -> MOSI
 *      pin % 4 == 2 -> MISO
 *      pin % 4 == 3 -> SCLK
 * So MISO must be on a pad where (pin % 4 == 2).  GPIO 24 (24%4==0) is a
 * CS pad and can never read MISO — it was the cause of "Version: 0x00".
 */
#define RFM95_PIN_MOSI   GLB_GPIO_PIN_25   /* 25 % 4 == 1 -> MOSI */
#define RFM95_PIN_MISO   GLB_GPIO_PIN_26   /* 26 % 4 == 2 -> MISO (was 24 = CS pad) */
#define RFM95_PIN_SCLK   GLB_GPIO_PIN_11   /* 11 % 4 == 3 -> SCLK */
#define RFM95_PIN_NSS    GLB_GPIO_PIN_12   /* manual GPIO chip-select       */
#define RFM95_PIN_NRST   GLB_GPIO_PIN_13
#define RFM95_PIN_DIO0   GLB_GPIO_PIN_31

/* SPI clock frequency (~5 MHz) */
#define RFM95_SPI_FREQ   (5 * 1000 * 1000)

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise the BL808 peripherals (SPI + GPIO) and populate
 *         the rfm95 handle with the correct platform callbacks.
 *
 * @return Pointer to the statically allocated, ready-to-use handle,
 *         or NULL on failure.
 */
rfm95_handle_t *bl808_rfm95_hal_init(void);

/**
 * @brief  De-initialise the BL808 peripherals.
 */
void bl808_rfm95_hal_deinit(void);
