/*
 * PPM (Pulse-Position Modulation) transmitter
 * Board : Seeed Studio XIAO ESP32-S3
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
 *   PWM   D0 / GPIO1: 1 MHz, 50% duty, independent LEDC output
 *   OUT+  D4 / GPIO5
 *   OUT-  D3 / GPIO4
 */

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_struct.h"
#include "esp_log.h"

static const char *TAG = "ppm_tx_xiao";

/* ---- User configuration ------------------------------------------------ */
#define PWM_GPIO      1            /* D0: independent PWM output             */
#define PWM_FREQ_HZ   1000000      /* 1 MHz, 50% duty                        */
#define OUT_A_GPIO    5            /* D4: OUT+ (RMT TX pad)                   */
#define OUT_B_GPIO    4            /* D3: OUT- (inverted mirror of OUT+)      */

#define RMT_RES_HZ    80000000     /* 80 MHz -> 12.5 ns / tick (RMT max on S3)*/
#define PULSE_TICKS   16           /* 200 ns constant high time (16 x 12.5 ns)*/
#define BASE_TICKS    32           /* 400 ns minimum interval (position 0)    */

#define N_VALUES      128          /* payload sweep 0..127                    */
#define INTER_BURST_US 100         /* gap between repeated demo bursts (us)   */
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

/* Continuous PWM runs in hardware, independently of the RMT DMA bursts.
 * A one-bit counter with duty=1 gives exactly 50% duty. */
static void pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_1_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_USE_XTAL_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 1,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    ESP_ERROR_CHECK(gpio_set_drive_capability(PWM_GPIO, GPIO_DRIVE_CAP_3));
    ESP_LOGI(TAG, "PWM configured: D0/GPIO%d | %u Hz, 50%% duty",
             PWM_GPIO, (unsigned)PWM_FREQ_HZ);
}

/* Read back transitions at the GPIO pad after all outputs are configured.
 * PCNT enables the pad's input path without replacing its LEDC output route.
 * Count both edges for 1 ms; a 1 MHz PWM should produce about 2000 edges. */
static void pwm_check_output(void)
{
    pcnt_unit_handle_t unit = NULL;
    pcnt_channel_handle_t channel = NULL;
    pcnt_unit_config_t unit_cfg = {
        .clk_src = PCNT_CLK_SRC_DEFAULT,
        .low_limit = -32768,
        .high_limit = 32767,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &unit));
    pcnt_chan_config_t channel_cfg = {
        .edge_gpio_num = PWM_GPIO,
        .level_gpio_num = -1,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel_cfg, &channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    int64_t start_us = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    esp_rom_delay_us(1000);
    ESP_ERROR_CHECK(pcnt_unit_stop(unit));
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    int edges = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &edges));
    uint32_t measured_hz = (uint32_t)((int64_t)edges * 1000000 / (2 * elapsed_us));
    ESP_LOGI(TAG, "PWM pad check: D0/GPIO%d | %d edges in %u us | ~%u Hz",
             PWM_GPIO, edges, (unsigned)elapsed_us, (unsigned)measured_hz);
    if (measured_hz < PWM_FREQ_HZ * 95 / 100 ||
        measured_hz > PWM_FREQ_HZ * 105 / 100) {
        ESP_LOGE(TAG, "PWM pad check failed: expected %u Hz; check output routing and load",
                 (unsigned)PWM_FREQ_HZ);
    }
    ESP_ERROR_CHECK(pcnt_unit_disable(unit));
    ESP_ERROR_CHECK(pcnt_del_channel(channel));
    ESP_ERROR_CHECK(pcnt_del_unit(unit));
}

void app_main(void)
{
    pwm_init();

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
    ESP_ERROR_CHECK(gpio_set_direction(OUT_B_GPIO, GPIO_MODE_OUTPUT));
    esp_rom_gpio_connect_out_signal(OUT_B_GPIO, rmt_sig, true /*invert*/, false);
    ESP_ERROR_CHECK(gpio_set_drive_capability(OUT_A_GPIO, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_drive_capability(OUT_B_GPIO, GPIO_DRIVE_CAP_3));

    ESP_ERROR_CHECK(rmt_enable(chan));
    pwm_check_output();

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
        if (++bursts % 1000 == 0) {
            ESP_LOGI(TAG, "%u bursts sent", (unsigned)bursts);
        }
        /* sub-tick gap, so busy-wait instead of vTaskDelay(). The
         * wait_all_done() above blocks on a semaphore each loop, so the idle
         * task still runs and the task watchdog stays fed. */
        esp_rom_delay_us(INTER_BURST_US);
    }
}
