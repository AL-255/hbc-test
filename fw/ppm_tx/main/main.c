/*
 * PPM transmitter: two same-polarity pulses with dead time + a 1 MHz reference
 * Board : Heltec WiFi LoRa 32 V3  (HTIT-WB32LA(F)_V3, ESP32-S3)
 *
 * Constant-width pulse, data carried in the inter-pulse interval. The frame is
 * pre-rendered and streamed by the RMT peripheral (every edge comes from
 * hardware RAM; the CPU only refills a block every ~20-40 us, never per pulse).
 *
 * Outputs:
 *   IO5  PPM pulse, 200 ns wide  (RMT channel A)
 *   IO6  SAME polarity as IO5 but NARROWER: it sits inside IO5's pulse, inset
 *        by the dead time on each edge -> width = 200 ns - 2 x DT. Driven by a
 *        second RMT channel started on the same clock edge (sync manager).
 *   IO7  free-running 1 MHz square wave, 50% duty (LEDC) -- a timing reference.
 *
 *      IO5  ___|‾‾‾‾‾‾‾‾|________________|‾‾‾‾‾‾‾‾|___    200 ns
 *      IO6  _____|‾‾‾‾|__________________ __|‾‾‾‾|_____    200 ns - 2*DT, centered
 *              D ^    ^ D                                 (dead-time margins)
 *      IO7  ‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|     1 MHz, 50%
 *
 * Timing (RMT @ 80 MHz, the S3 maximum -> 12.5 ns / tick):
 *   IO5 width  : 200 ns = 16 ticks (constant)
 *   IO6 width  : (16 - 2 x DEAD_TICKS) ticks, centered in IO5's pulse
 *   Interval   : 400 ns + position x 12.5 ns ; position = value >> 1
 *   Payload    : value 0..127 -> 64 positions (2:1), 400 ns .. 1187.5 ns
 *   Burst      : the full 0..127 sweep = 128 symbols per channel
 */

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "ppm_tx";

/* ---- User configuration ------------------------------------------------ */
#define OUT_A_GPIO     6           /* IO5: PPM pulse                          */
#define OUT_B_GPIO     19           /* IO6: same polarity, narrower by 2 x DT  */
#define PWM_GPIO       5           /* IO7: 1 MHz / 50% reference (LEDC)       */

#define RMT_RES_HZ     80000000    /* 80 MHz -> 12.5 ns / tick (RMT max on S3)*/
#define PULSE_TICKS    33          /* 200 ns IO5 pulse (16 x 12.5 ns)         */
#define BASE_TICKS     64          /* 400 ns minimum interval (position 0)    */
#define DEAD_TICKS     4           /* dead time per edge, in 12.5 ns ticks    */
                                   /*   1..7 (12.5..87.5 ns); 2 = 25 ns       */
#define N_VALUES       128         /* payload sweep 0..127                    */
#define INTER_BURST_US 100         /* gap between repeated demo bursts (us)   */

#define PWM_FREQ_HZ    1000000     /* IO7 PWM rate                            */
#define PWM_RES        LEDC_TIMER_6_BIT  /* 64 duty steps (50% = 32, exact)   */
/* ------------------------------------------------------------------------ */

static uint8_t          s_payload[N_VALUES];   /* the data buffer: 0..127 sweep   */
static rmt_symbol_word_t s_frame_a[N_VALUES];  /* IO5 symbols (one per pulse)      */
static rmt_symbol_word_t s_frame_b[N_VALUES];  /* IO6 symbols (one per pulse)      */

static inline uint32_t interval_ticks(uint8_t value)
{
    return BASE_TICKS + (value >> 1);          /* 32 .. 95 ticks */
}

static void start_pwm_reference(float duty)
{
    /* Free-running 1 MHz, 50% square wave on IO7 via LEDC -- runs forever,
     * independent of the RMT bursts. */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       =   (1 << PWM_RES) * duty,   /* 50% duty */
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    gpio_set_drive_capability(PWM_GPIO, GPIO_DRIVE_CAP_3);
}

static void set_pwm_duty(float duty)
{
    /* Adjust the PWM duty cycle on the fly. The change takes effect immediately
     * and doesn't disrupt the RMT bursts. */
    uint32_t duty_val = (1 << PWM_RES) * duty;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_val));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void app_main(void)
{
    /* 1. Fill the payload buffer with the 0..127 sweep and render both legs.
     *
     *    IO5 : HIGH PULSE_TICKS, then LOW for the rest of the interval.
     *    IO6 : same polarity, a narrower pulse nested inside IO5's, inset by
     *          DEAD_TICKS on each edge. One symbol per cycle, {LOW gap, HIGH
     *          body}: the gap leads up to this cycle's pulse (DEAD_TICKS for
     *          the first cycle; previous interval - P + 2*DEAD_TICKS after),
     *          and the body is the narrow high of (P - 2*DEAD_TICKS). */
    for (int i = 0; i < N_VALUES; i++) {
        s_payload[i] = (uint8_t)i;
        uint32_t I = interval_ticks(s_payload[i]);

        s_frame_a[i] = (rmt_symbol_word_t){
            .level0 = 1, .duration0 = PULSE_TICKS,            /* 200 ns high */
            .level1 = 0, .duration1 = I - PULSE_TICKS,        /* gap         */
        };

        uint32_t gap = (i == 0)
            ? DEAD_TICKS
            : (interval_ticks(s_payload[i - 1]) - PULSE_TICKS + 2 * DEAD_TICKS);
        s_frame_b[i] = (rmt_symbol_word_t){
            .level0 = 0, .duration0 = gap,                        /* low up to pulse */
            .level1 = 1, .duration1 = PULSE_TICKS - 2 * DEAD_TICKS,/* narrow high     */
        };
    }

    /* 2. Two RMT TX channels (non-DMA -- only one channel can use DMA on the
     *    S3, and both legs must share a group for the sync manager). */
    rmt_channel_handle_t chan_a = NULL, chan_b = NULL;
    rmt_tx_channel_config_t cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,   /* APB, 80 MHz */
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = 48,                    /* one block; driver streams via wrap */
        .trans_queue_depth = 4,
    };
    cfg.gpio_num = OUT_A_GPIO;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&cfg, &chan_a));
    cfg.gpio_num = OUT_B_GPIO;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&cfg, &chan_b));

    /* 3. A copy encoder per channel (streams our pre-built symbol arrays). */
    rmt_encoder_handle_t enc_a = NULL, enc_b = NULL;
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &enc_a));
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &enc_b));

    ESP_ERROR_CHECK(rmt_enable(chan_a));
    ESP_ERROR_CHECK(rmt_enable(chan_b));

    /* 4. Sync manager: neither channel starts until BOTH have been queued, so
     *    they begin on the same clock edge and IO6 stays centered in IO5. */
    rmt_sync_manager_handle_t synchro = NULL;
    rmt_channel_handle_t chans[] = { chan_a, chan_b };
    rmt_sync_manager_config_t sync_cfg = {
        .tx_channel_array = chans,
        .array_size       = sizeof(chans) / sizeof(chans[0]),
    };
    ESP_ERROR_CHECK(rmt_new_sync_manager(&sync_cfg, &synchro));

    gpio_set_drive_capability(OUT_A_GPIO, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(OUT_B_GPIO, GPIO_DRIVE_CAP_3);

    /* 5. Start the independent 1 MHz / 50% reference on IO7. */
    start_pwm_reference(0.8);

    ESP_LOGI(TAG, "IO5=GPIO%d (200 ns) IO6=GPIO%d (200ns-2xDT, same polarity) "
                  "IO7=GPIO%d (1 MHz 50%%) | dead time %d.%d ns | %d-value sweep",
             OUT_A_GPIO, OUT_B_GPIO, PWM_GPIO,
             DEAD_TICKS * 125 / 10, (DEAD_TICKS * 125) % 10, N_VALUES);

    /* 6. Fire the sweep on both PPM legs as one synchronized burst; repeat with
     *    a gap so a scope/LA can retrigger. rmt_sync_reset re-aligns the pair
     *    before each round. The IO7 PWM keeps running throughout. */
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,            /* single shot, no hardware looping */
        .flags.eot_level = 0,       /* both PPM legs idle LOW after the burst */
    };

    uint32_t bursts = 0;
    while (1) {
        ESP_ERROR_CHECK(rmt_sync_reset(synchro));
        ESP_ERROR_CHECK(rmt_transmit(chan_a, enc_a, s_frame_a, sizeof(s_frame_a), &tx_cfg));
        ESP_ERROR_CHECK(rmt_transmit(chan_b, enc_b, s_frame_b, sizeof(s_frame_b), &tx_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(chan_a, portMAX_DELAY));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(chan_b, portMAX_DELAY));
        if (++bursts % 1000 == 0) {
            ESP_LOGI(TAG, "%u bursts sent", (unsigned)bursts);
        }
        /* sub-tick gap, so busy-wait instead of vTaskDelay(). The
         * wait_all_done() above blocks on a semaphore each loop, so the idle
         * task still runs and the task watchdog stays fed. */
        esp_rom_delay_us(INTER_BURST_US);
    }
}
