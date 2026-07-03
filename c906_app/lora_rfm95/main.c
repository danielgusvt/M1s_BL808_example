/**
 * @file main.c
 * @brief LoRa RFM95 grayscale-JPEG sender for the BL808 D0/C906 core.
 *
 * Captures a grayscale (YUV400) JPEG from the MIPI camera and transmits it
 * over LoRa when a push-button on GPIO 22 is pressed.  No acknowledgement /
 * retransmission (v1): the receiver simply reassembles whatever it hears.
 *
 * ---------------------------------------------------------------------------
 *  On-air application framing (inside each LoRa payload, max 248 bytes).
 *  The rfm95 driver prepends its own 4-byte RadioHead header internally, so
 *  these bytes are what the receiver sees from rfm95_receive_package().
 *
 *   HEADER  : [0xA1][len31..len0 (4, big-endian)][chunks15..0 (2, BE)]   (7 B)
 *   DATA    : [0xA2][idx15..idx0 (2, BE)][ up to 240 JPEG bytes ]        (<=243 B)
 *   TAIL    : [0xA3][len31..len0 (4, big-endian)]                        (5 B)
 *
 *  Chunks are sent strictly in order (idx = 0,1,2,...).  Total image length
 *  and chunk count are announced up-front so the receiver can sanity-check.
 * ---------------------------------------------------------------------------
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>

/* BL808 + RFM95 driver/HAL */
#include <bl808_glb.h>
#include <bl_cam.h>
#include <bl808_clock.h>
#include "rfm95.h"
#include "bl808_rfm95_hal.h"

/* ------------------------------------------------------------------ */
/*  Configuration                                                      */
/* ------------------------------------------------------------------ */
#define BUTTON_PIN              GLB_GPIO_PIN_22   /* push-button to GND (active low) */
#define BUTTON_DEBOUNCE_MS      30

/* Application packet framing (see banner comment) */
#define PKT_TYPE_HEADER         0xA1
#define PKT_TYPE_DATA           0xA2
#define PKT_TYPE_TAIL           0xA3
#define CHUNK_DATA_LEN          240               /* JPEG bytes per DATA packet      */
#define INTER_PACKET_DELAY_MS   100               /* let the receiver re-arm RX       */
#define HEADER_REPEAT           1                 /* send HEADER/TAIL N× for resilience.
                                                     A lost BEGIN/END loses the whole
                                                     image, so duplicate the cheap
                                                     framing packets; RX dedups them.  */

/* Max grayscale JPEG we are prepared to buffer (320x240 q75 is ~6-15 KB). */
#define IMG_BUF_SIZE            (64 * 1024)

static uint8_t *img_buf = NULL;

/* ------------------------------------------------------------------ */
/*  Button (polled, no GPIO IRQ on the D0/c906 core)                   */
/* ------------------------------------------------------------------ */
static void button_init(void)
{
    GLB_GPIO_Cfg_Type cfg;
    cfg.gpioPin    = BUTTON_PIN;
    cfg.gpioMode   = GPIO_MODE_INPUT;
    cfg.gpioFun    = GPIO_FUN_GPIO;
    cfg.pullType   = GPIO_PULL_UP;     /* idle high, pressed pulls to GND */
    cfg.drive      = 0;
    cfg.smtCtrl    = 1;
    GLB_GPIO_Init(&cfg);
}

/* Returns true once per press (debounced falling edge). */
static bool button_pressed(void)
{
    static bool prev_down = false;
    bool down = (GLB_GPIO_Read(BUTTON_PIN) == 0);

    if (down && !prev_down) {
        /* debounce: confirm still pressed */
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        down = (GLB_GPIO_Read(BUTTON_PIN) == 0);
        prev_down = down;
        return down;
    }
    prev_down = down;
    return false;
}

/* ------------------------------------------------------------------ */
/*  Image capture                                                      */
/* ------------------------------------------------------------------ */

/* Grab the freshest *valid* grayscale JPEG into img_buf. Returns length, or 0. */
static uint32_t capture_jpeg(void)
{
    uint8_t  *ptr = NULL;
    uint32_t  len = 0;

    /* The camera free-ran during the previous transmission and its 1 MB ring
       wrapped several times, so the FIFO head may point at frame memory that
       newer frames have already overwritten.  Flush the whole FIFO, then take
       the first freshly-completed frame that is a complete JPEG. */
    bl_cam_mjpeg_flush();

    for (int tries = 0; tries < 600; tries++) {
        if (bl_cam_mjpeg_get(&ptr, &len) == 0) {
            bool valid = (len >= 4 && len <= IMG_BUF_SIZE &&
                          ptr[0] == 0xFF && ptr[1] == 0xD8 &&          /* SOI */
                          ptr[len - 2] == 0xFF && ptr[len - 1] == 0xD9); /* EOI */
            if (valid) {
                /* Copy out immediately: TX takes seconds and the camera keeps
                   overwriting its ring buffer. */
                memcpy(img_buf, ptr, len);
                bl_cam_mjpeg_pop();
                return len;
            }
            /* Stale / incomplete frame — drop it and keep waiting. */
            bl_cam_mjpeg_pop();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    printf("[CAM] timed out waiting for a valid frame\r\n");
    return 0;
}

/* ------------------------------------------------------------------ */
/*  LoRa image transmission                                            */
/* ------------------------------------------------------------------ */
static bool send_image(rfm95_handle_t *rfm95, const uint8_t *jpg, uint32_t len)
{
    uint16_t total_chunks = (uint16_t)((len + CHUNK_DATA_LEN - 1) / CHUNK_DATA_LEN);
    uint8_t  pkt[3 + CHUNK_DATA_LEN];

    /* ---- HEADER (sent HEADER_REPEAT× ; RX keeps the first, drops the rest) ---- */
    pkt[0] = PKT_TYPE_HEADER;
    pkt[1] = (uint8_t)(len >> 24);
    pkt[2] = (uint8_t)(len >> 16);
    pkt[3] = (uint8_t)(len >> 8);
    pkt[4] = (uint8_t)(len);
    pkt[5] = (uint8_t)(total_chunks >> 8);
    pkt[6] = (uint8_t)(total_chunks);
    for (int r = 0; r < HEADER_REPEAT; r++) {
        if (!rfm95_send_package(rfm95, pkt, 7)) {
            printf("[TX] header send failed\r\n");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(INTER_PACKET_DELAY_MS));
    }

    /* ---- DATA ---- */
    for (uint16_t idx = 0; idx < total_chunks; idx++) {
        uint32_t offset = (uint32_t)idx * CHUNK_DATA_LEN;
        uint16_t n = (uint16_t)((len - offset) < CHUNK_DATA_LEN
                                ? (len - offset) : CHUNK_DATA_LEN);

        pkt[0] = PKT_TYPE_DATA;
        pkt[1] = (uint8_t)(idx >> 8);
        pkt[2] = (uint8_t)(idx);
        memcpy(&pkt[3], &jpg[offset], n);

        if (!rfm95_send_package(rfm95, pkt, (size_t)(3 + n))) {
            printf("[TX] data chunk %u send failed\r\n", idx);
            return false;
        }
        if ((idx % 10) == 0) {
            printf("[TX] chunk %u/%u\r\n", idx, total_chunks);
        }
        vTaskDelay(pdMS_TO_TICKS(INTER_PACKET_DELAY_MS));
    }

    /* ---- TAIL (sent HEADER_REPEAT× ; RX keeps the first, drops the rest) ---- */
    pkt[0] = PKT_TYPE_TAIL;
    pkt[1] = (uint8_t)(len >> 24);
    pkt[2] = (uint8_t)(len >> 16);
    pkt[3] = (uint8_t)(len >> 8);
    pkt[4] = (uint8_t)(len);
    for (int r = 0; r < HEADER_REPEAT; r++) {
        if (!rfm95_send_package(rfm95, pkt, 5)) {
            printf("[TX] tail send failed\r\n");
            return false;
        }
        if (r < HEADER_REPEAT - 1) {
            vTaskDelay(pdMS_TO_TICKS(INTER_PACKET_DELAY_MS));
        }
    }

    return true;
}

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */
void main(void)
{
    printf("\r\n========================================\r\n");
    printf(" BL808 D0/C906 — LoRa grayscale-JPEG sender\r\n");
    printf("========================================\r\n\r\n");

    img_buf = pvPortMalloc(IMG_BUF_SIZE);
    if (img_buf == NULL) {
        printf("[MAIN] image buffer alloc failed — halting.\r\n");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    /* --- RFM95 (SPI + GPIO) --- */
    rfm95_handle_t *rfm95 = bl808_rfm95_hal_init();
    if (rfm95 == NULL || !rfm95_init(rfm95)) {
        printf("[MAIN] RFM95 init failed — halting.\r\n");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    printf("[MAIN] RFM95 ready (868 MHz, SF7, BW125, 20 dBm)\r\n");

    /* --- Camera in grayscale MJPEG (YUV400) mode --- */
    if (bl_cam_mipi_mjpeg_gray_init() != 0) {
        printf("[MAIN] camera init failed — halting.\r\n");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    printf("[MAIN] camera ready (grayscale JPEG 320x240)\r\n");

    /* --- Button --- */
    button_init();
    printf("[MAIN] press the button on GPIO %d to capture & send.\r\n", BUTTON_PIN);

    while (1) {
        if (button_pressed()) {
            printf("\r\n[BTN] pressed — capturing frame...\r\n");

            uint32_t len = capture_jpeg();
            if (len == 0) {
                printf("[MAIN] capture failed, ignoring press.\r\n");
                continue;
            }
            printf("[MAIN] captured %lu byte JPEG, transmitting...\r\n",
                   (unsigned long)len);

            if (send_image(rfm95, img_buf, len)) {
                printf("[MAIN] image sent (%lu bytes).\r\n", (unsigned long)len);
            } else {
                printf("[MAIN] image send aborted.\r\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
