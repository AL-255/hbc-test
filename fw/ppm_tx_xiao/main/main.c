/*
 * PPM (Pulse-Position Modulation) transmitter
 * Board : Seeed Studio XIAO ESP32-S3
 *
 * Two synchronized RMT channels generate non-overlapping phases. The complete
 * frame is pre-rendered: OUT+ streams by DMA; OUT- fits in hardware RMT RAM.
 * Neither channel needs CPU service to generate individual pulses.
 *
 * Timing (RMT runs at 80 MHz, the S3 maximum -> 12.5 ns / tick):
 *   Pulse width : 200 ns  = 16 ticks  (constant)
 *   Dead time   : ceil(pulse width * 10%) on the 12.5 ns grid = 25 ns
 *   Interval    : 400 ns + position * 12.5 ns + 2 * dead time
 *   Payload     : value 0..127, position = value >> 1  (2:1 -> 64 positions)
 *                 -> interval 450 ns .. 1237.5 ns with the default dead time
 *
 * Both PPM outputs are LOW during dead time and between bursts:
 *   PWM   D0 / GPIO1: configurable frequency/duty; 100% holds HIGH
 *   OUT+  D4 / GPIO5
 *   OUT-  D3 / GPIO4
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "ppm_tx_xiao";

/* ---- User configuration ------------------------------------------------ */
#define PWM_GPIO      1            /* D0: independent PWM output             */
#define PWM_FREQ_HZ   100000       /* frequency used when duty is 1..99%      */
#define PWM_DUTY_PERCENT 100       /* 0 = LOW, 100 = HIGH, otherwise PWM      */
#define OUT_A_GPIO    5            /* D4: OUT+ (RMT TX pad)                   */
#define OUT_B_GPIO    4            /* D3: OUT- (separate synchronized phase)  */

#define RMT_RES_HZ    80000000     /* 80 MHz -> 12.5 ns / tick (RMT max on S3)*/
#define PULSE_TICKS   16           /* 200 ns constant high time (16 x 12.5 ns)*/
#define BASE_TICKS    32           /* 400 ns minimum interval (position 0)    */
#define DEAD_TIME_PERCENT 10       /* percentage of PULSE_TICKS, rounded UP   */
#define DEAD_TICKS ((PULSE_TICKS * DEAD_TIME_PERCENT + 99) / 100)
#define PHASE_B_MEM_SYMBOLS 144     /* three 48-symbol blocks; DMA uses fourth */

#define N_VALUES      128          /* payload sweep 0..127                    */
#define INTER_BURST_US 100         /* gap between repeated demo bursts (us)   */
/* ------------------------------------------------------------------------ */

_Static_assert(PWM_FREQ_HZ > 0, "PWM frequency must be positive");
_Static_assert(PWM_DUTY_PERCENT >= 0 && PWM_DUTY_PERCENT <= 100,
               "PWM duty must be between 0 and 100 percent");
_Static_assert(PULSE_TICKS > 0 && PULSE_TICKS < BASE_TICKS,
               "Both phase widths must be positive");
_Static_assert(DEAD_TICKS > 0, "Dead time must be at least one RMT tick");
_Static_assert(N_VALUES > 0 && N_VALUES <= 128, "Payload is a 7-bit sweep");
_Static_assert(N_VALUES + 2 <= PHASE_B_MEM_SYMBOLS,
               "OUT- frame plus EOF must fit completely in hardware RAM");
_Static_assert(BASE_TICKS + ((N_VALUES - 1) >> 1) + 2 * DEAD_TICKS <= 32767,
               "RMT durations must fit the 15-bit field");

static rmt_symbol_word_t s_frame_a[N_VALUES];
static rmt_symbol_word_t s_frame_b[N_VALUES + 1];

/*
 * Preserve both original HIGH widths, inserting a both-LOW gap between them:
 *   OUT+ HIGH P, both LOW D, OUT- HIGH L, both LOW D.
 *   P = PULSE_TICKS; L = BASE_TICKS + (value >> 1) - P; D = DEAD_TICKS.
 * The OUT- LOW run spans the previous trailing gap, OUT+'s pulse, and the
 * next gap. The first run has no previous trailing gap; the last is explicit.
 */
static void render_frames(void)
{
    for (int i = 0; i < N_VALUES; i++) {
        uint32_t phase_b_ticks = BASE_TICKS + (i >> 1) - PULSE_TICKS;
        s_frame_a[i] = (rmt_symbol_word_t) {
            .level0 = 1, .duration0 = PULSE_TICKS,
            .level1 = 0, .duration1 = phase_b_ticks + 2 * DEAD_TICKS,
        };
        s_frame_b[i] = (rmt_symbol_word_t) {
            .level0 = 0, .duration0 = PULSE_TICKS + (i == 0 ? DEAD_TICKS : 2 * DEAD_TICKS),
            .level1 = 1, .duration1 = phase_b_ticks,
        };
    }
    s_frame_b[N_VALUES] = (rmt_symbol_word_t) {
        .level0 = 0, .duration0 = DEAD_TICKS,
        .level1 = 0, .duration1 = 0, /* stop after the final both-LOW gap */
    };
}

/* D0 runs independently of PPM. Use GPIO for exact 0%/100% duty, avoiding
 * LEDC counter overflow at full duty. Intermediate duties use hardware PWM. */
static void pwm_init(void)
{
    if (PWM_DUTY_PERCENT == 0 || PWM_DUTY_PERCENT == 100) {
        const int level = PWM_DUTY_PERCENT == 100;
        ESP_ERROR_CHECK(gpio_set_level(PWM_GPIO, level));
        gpio_config_t gpio_cfg = {
            .pin_bit_mask = 1ULL << PWM_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
        ESP_ERROR_CHECK(gpio_set_drive_capability(PWM_GPIO, GPIO_DRIVE_CAP_3));
        ESP_LOGI(TAG, "D0/GPIO%d ready: constant %s (%d%% duty)",
                 PWM_GPIO, level ? "HIGH" : "LOW", PWM_DUTY_PERCENT);
        return;
    }

    /* XIAO's XTAL is 40 MHz; choose the best resolution for the frequency. */
    uint32_t duty_bits = ledc_find_suitable_duty_resolution(40000000, PWM_FREQ_HZ);
    ESP_ERROR_CHECK(duty_bits ? ESP_OK : ESP_ERR_INVALID_ARG);
    uint32_t period = 1U << duty_bits;
    uint32_t duty = (period * PWM_DUTY_PERCENT + 50) / 100;
    /* Keep intermediate percentages switching even at coarse resolution. */
    if (duty == 0) duty = 1;
    if (duty >= period) duty = period - 1;

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)duty_bits,
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
        .duty       = duty,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    ESP_ERROR_CHECK(gpio_set_drive_capability(PWM_GPIO, GPIO_DRIVE_CAP_3));
    ESP_LOGI(TAG, "PWM ready: D0/GPIO%d | %u Hz, %.2f%% duty (%d%% requested)",
             PWM_GPIO, (unsigned)PWM_FREQ_HZ, 100.0 * duty / period, PWM_DUTY_PERCENT);
}

void app_main(void)
{
    pwm_init();

    render_frames();

    /* 2. RMT TX channel on OUT+, DMA-capable, 80 MHz (12.5 ns ticks). */
    rmt_channel_handle_t channels[2] = {NULL, NULL};
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,   /* APB, 80 MHz */
        .gpio_num          = OUT_A_GPIO,
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = 256,                   /* DMA buffer (must be even) */
        .trans_queue_depth = 4,
        .flags.with_dma    = true,                  /* burst straight from RAM */
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &channels[0]));

    /* ESP32-S3 has only one DMA TX channel. Reserve the other three memory
     * blocks for OUT- so all 129 symbols and EOF are preloaded before start. */
    chan_cfg.gpio_num = OUT_B_GPIO;
    chan_cfg.mem_block_symbols = PHASE_B_MEM_SYMBOLS;
    chan_cfg.flags.with_dma = false;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &channels[1]));

    /* 3. Copy encoder: streams our pre-built symbol array verbatim. */
    rmt_encoder_handle_t encoders[2] = {NULL, NULL};
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &encoders[0]));
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &encoders[1]));

    /* 4. Start both phases on the same hardware trigger. A pad inverter
     * cannot create dead time because it has no independent edge timing. */
    ESP_ERROR_CHECK(gpio_set_drive_capability(OUT_A_GPIO, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_drive_capability(OUT_B_GPIO, GPIO_DRIVE_CAP_3));

    ESP_ERROR_CHECK(rmt_enable(channels[0]));
    ESP_ERROR_CHECK(rmt_enable(channels[1]));
    rmt_sync_manager_handle_t sync = NULL;
    rmt_sync_manager_config_t sync_cfg = {
        .tx_channel_array = channels,
        .array_size = 2,
    };
    ESP_ERROR_CHECK(rmt_new_sync_manager(&sync_cfg, &sync));

    ESP_LOGI(TAG, "PPM TX ready: OUT+=GPIO%d OUT-=GPIO%d | pulse %.1f ns | "
                  "dead time %d%% requested, %.1f ns actual at each handoff | %d-value sweep",
             OUT_A_GPIO, OUT_B_GPIO, PULSE_TICKS * 1e9 / RMT_RES_HZ,
             DEAD_TIME_PERCENT, DEAD_TICKS * 1e9 / RMT_RES_HZ, N_VALUES);

    /* 5. Fire the sweep as one finite DMA burst; repeat with a gap so a scope
     *    or logic analyzer can retrigger on it. */
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,            /* single shot, no hardware looping */
        .flags.eot_level = 0,       /* both outputs idle LOW after the burst */
    };

    uint32_t bursts = 0;
    while (1) {
        ESP_ERROR_CHECK(rmt_transmit(channels[0], encoders[0], s_frame_a, sizeof(s_frame_a), &tx_cfg));
        ESP_ERROR_CHECK(rmt_transmit(channels[1], encoders[1], s_frame_b, sizeof(s_frame_b), &tx_cfg));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(channels[0], -1));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(channels[1], -1));
        ESP_ERROR_CHECK(rmt_sync_reset(sync));
        if (++bursts % 1000 == 0) {
            ESP_LOGI(TAG, "%u bursts sent", (unsigned)bursts);
        }
        /* sub-tick gap, so busy-wait instead of vTaskDelay(). The
         * wait_all_done() above blocks on a semaphore each loop, so the idle
         * task still runs and the task watchdog stays fed. */
        esp_rom_delay_us(INTER_BURST_US);
    }
}
