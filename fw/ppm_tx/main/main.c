/*
 * PPM (Pulse-Position Modulation) transmitter
 * Board : Heltec WiFi LoRa 32 V3  (HTIT-WB32LA(F)_V3, ESP32-S3)
 *
 * Constant 200 ns pulse, variable interval. The whole frame is pre-rendered
 * into a buffer and fired as ONE DMA burst via the RMT peripheral -- the
 * peripheral walks the buffer itself, zero CPU per pulse. (A per-pulse timer
 * ISR can't keep up: the shortest interval here is 400 ns, well under the
 * interrupt service time, so DMA is the only way to stream this cleanly.)
 *
 * Timing (RMT runs at 80 MHz, the S3 maximum -> 12.5 ns / tick):
 *   Pulse width : 200 ns  = 16 ticks  (constant)
 *   Interval    : 400 ns + position * 12.5 ns
 *   Payload     : value 0..127, position = value >> 1  (2:1 -> 64 positions)
 *                 -> interval 400 ns .. 1187.5 ns, inside the 400..1200 ns window
 *
 * Outputs (complementary / differential, zero skew -- both derive from the
 * same RMT signal; OUT- is the same signal inverted at the pad):
 *   OUT+  GPIO5
 *   OUT-  GPIO6
 */

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"

static const char *TAG = "ppm_tx";

/* ---- User configuration ------------------------------------------------ */
#define OUT_A_GPIO    5            /* OUT+  (RMT TX pad)                      */
#define OUT_B_GPIO    6            /* OUT-  (inverted mirror of OUT+)         */

#define RMT_RES_HZ    80000000     /* 80 MHz -> 12.5 ns / tick (RMT max on S3)*/
#define PULSE_TICKS   16           /* 200 ns constant high time (16 x 12.5 ns)*/
#define BASE_TICKS    32           /* 400 ns minimum interval (position 0)    */

#define N_VALUES      128          /* payload sweep 0..127                    */
#define INTER_BURST_MS 10          /* gap between repeated demo bursts        */
/* ------------------------------------------------------------------------ */

static uint8_t          s_payload[N_VALUES];   /* the data buffer: 0..127 sweep   */
static rmt_symbol_word_t s_frame[N_VALUES];    /* one RMT symbol per PPM pulse     */

/*
 * Render one payload value into a PPM RMT symbol:
 *   level0 = HIGH for PULSE_TICKS           (constant 200 ns pulse)
 *   level1 = LOW  for the rest of the interval
 * interval = BASE_TICKS + (value >> 1) ticks ; gap = interval - pulse.
 */
static inline rmt_symbol_word_t ppm_symbol(uint8_t value)
{
    uint32_t interval_ticks = BASE_TICKS + (value >> 1);     /* 32 .. 95  */
    uint32_t low_ticks      = interval_ticks - PULSE_TICKS;  /* 16 .. 79  */
    rmt_symbol_word_t sym = {
        .level0 = 1, .duration0 = PULSE_TICKS,   /* 200 ns high */
        .level1 = 0, .duration1 = low_ticks,     /* gap         */
    };
    return sym;
}

void app_main(void)
{
    /* 1. Fill the payload buffer with the 0..127 sweep, then pre-render the
     *    entire frame into RMT symbols. */
    for (int i = 0; i < N_VALUES; i++) {
        s_payload[i] = (uint8_t)i;
        s_frame[i]   = ppm_symbol(s_payload[i]);
    }

    /* 2. RMT TX channel on OUT+, DMA-capable, 80 MHz (12.5 ns ticks). */
    rmt_channel_handle_t chan = NULL;
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,   /* APB, 80 MHz */
        .gpio_num          = OUT_A_GPIO,
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = 256,                   /* DMA buffer (must be even) */
        .trans_queue_depth = 4,
        .flags.with_dma    = true,                  /* burst straight from RAM */
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &chan));

    /* 3. Copy encoder: streams our pre-built symbol array verbatim. */
    rmt_encoder_handle_t encoder = NULL;
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &encoder));

    /* 4. Complementary leg: mirror the RMT output signal onto OUT-, inverted,
     *    through the GPIO matrix. Both pads come from the same internal signal,
     *    so OUT- is a true zero-skew inverse of OUT+. */
    uint32_t rmt_sig = GPIO.func_out_sel_cfg[OUT_A_GPIO].func_sel;
    esp_rom_gpio_pad_select_gpio(OUT_B_GPIO);
    gpio_set_direction(OUT_B_GPIO, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(OUT_B_GPIO, rmt_sig, true /*invert*/, false);
    gpio_set_drive_capability(OUT_A_GPIO, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(OUT_B_GPIO, GPIO_DRIVE_CAP_3);

    ESP_ERROR_CHECK(rmt_enable(chan));

    ESP_LOGI(TAG, "PPM TX ready: OUT+=GPIO%d OUT-=GPIO%d | 200 ns pulse, "
                  "12.5 ns grid | %d-value sweep (%u symbols) per DMA burst",
             OUT_A_GPIO, OUT_B_GPIO, N_VALUES, (unsigned)N_VALUES);

    /* 5. Fire the sweep as one finite DMA burst; repeat with a gap so a scope
     *    or logic analyzer can retrigger on it. */
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,            /* single shot, no hardware looping */
        .flags.eot_level = 0,       /* idle LOW on OUT+ after the burst  */
    };

    uint32_t bursts = 0;
    while (1) {
        ESP_ERROR_CHECK(rmt_transmit(chan, encoder, s_frame, sizeof(s_frame), &tx_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(chan, portMAX_DELAY));
        if (++bursts % 100 == 0) {
            ESP_LOGI(TAG, "%u bursts sent", (unsigned)bursts);
        }
        vTaskDelay(pdMS_TO_TICKS(INTER_BURST_MS));
    }
}
