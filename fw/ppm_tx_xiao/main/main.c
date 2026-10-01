/*
 * PPM (Pulse-Position Modulation) transmitter
 * Board : Seeed Studio XIAO ESP32-S3
 *
 * Two synchronized RMT channels generate same-polarity, nested pulses. The complete
 * frame is pre-rendered: OUT+ streams by DMA; OUT- fits in hardware RMT RAM.
 * Neither channel needs CPU service to generate individual pulses.
 *
 * Timing (RMT runs at 80 MHz, the S3 maximum -> 12.5 ns / tick):
 *   Pulse width : 200 ns  = 16 ticks  (constant)
 *   Dead time   : ceil(pulse width * 10%) on the 12.5 ns grid = 25 ns per edge
 *   Inner pulse : 200 ns - 2 * dead time = 150 ns, centered in the outer pulse
 *   Interval    : 400 ns + position * 12.5 ns
 *   Payload     : value 0..127, position = value >> 1  (2:1 -> 64 positions)
 *                 -> interval 400 ns .. 1187.5 ns
 *
 * Both PPM outputs idle LOW between pulses and between bursts:
 *   PWM   D0 / GPIO1: configurable frequency/duty; 100% holds HIGH
 *   PPM A D4 / GPIO5: outer pulse
 *   PPM B D3 / GPIO4: same polarity, inset by dead time on each edge
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_clk_tree.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"
#include "esp_log.h"

#if CONFIG_PM_ENABLE || CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 80 || !CONFIG_FREERTOS_UNICORE
#error "Use the fixed 80 MHz, single-core settings in sdkconfig.defaults"
#endif

static const char *TAG = "ppm_tx_xiao";

/* ---- User configuration ------------------------------------------------ */
#define PWM_GPIO      1            /* D0: independent PWM output             */
#define PWM_FREQ_HZ   100000       /* frequency used when duty is 1..99%      */
#define PWM_DUTY_PERCENT 50        /* 0 = LOW, 100 = HIGH, otherwise PWM      */
#define OUT_A_GPIO    5            /* D4: outer PPM pulse                     */
#define OUT_B_GPIO    4            /* D3: same polarity, narrower by 2 x DT   */

#define RMT_RES_HZ    80000000     /* 80 MHz -> 12.5 ns / tick (RMT max on S3)*/
#define PULSE_TICKS   16           /* 200 ns constant high time (16 x 12.5 ns)*/
#define BASE_TICKS    32           /* 400 ns minimum interval (position 0)    */
#define DEAD_TIME_PERCENT 10       /* percentage of PULSE_TICKS, rounded UP   */
#define DEAD_TICKS ((PULSE_TICKS * DEAD_TIME_PERCENT + 99) / 100)
#define PHASE_A_DMA_SYMBOLS 288     /* frame + EOF fit in the first DMA half */
#define PHASE_B_MEM_SYMBOLS 144     /* three 48-symbol blocks; DMA uses fourth */

#define N_VALUES      128          /* payload sweep 0..127                    */
#define INTER_BURST_US 100         /* gap between repeated demo bursts (us)   */
#define STATUS_INTERVAL_US 5000000 /* infrequent logging reduces CPU/USB work */
/* ------------------------------------------------------------------------ */

_Static_assert(PWM_FREQ_HZ > 0, "PWM frequency must be positive");
_Static_assert(PWM_DUTY_PERCENT >= 0 && PWM_DUTY_PERCENT <= 100,
               "PWM duty must be between 0 and 100 percent");
_Static_assert(PULSE_TICKS > 0 && PULSE_TICKS < BASE_TICKS,
               "Outer pulse and inter-pulse gap must be positive");
_Static_assert(DEAD_TICKS > 0, "Dead time must be at least one RMT tick");
_Static_assert(2 * DEAD_TICKS < PULSE_TICKS,
               "Dead-time margins must leave a positive inner pulse");
_Static_assert(N_VALUES > 0 && N_VALUES <= 128, "Payload is a 7-bit sweep");
_Static_assert(N_VALUES + 1 <= PHASE_A_DMA_SYMBOLS / 2,
               "A frame and EOF must fit in one DMA descriptor");
_Static_assert(N_VALUES + 2 <= PHASE_B_MEM_SYMBOLS,
               "OUT- frame plus EOF must fit completely in hardware RAM");
_Static_assert(BASE_TICKS + ((N_VALUES - 1) >> 1) <= 32767,
               "RMT durations must fit the 15-bit field");
_Static_assert(INTER_BURST_US > 0, "Inter-burst timer delay must be positive");

static rmt_symbol_word_t s_frame_a[N_VALUES];
static rmt_symbol_word_t s_frame_b[N_VALUES + 1];

typedef struct {
    gptimer_handle_t gap_timer;
    TaskHandle_t task;
    volatile unsigned completed_channels;
} burst_state_t;

static burst_state_t s_burst;
static portMUX_TYPE s_completion_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_idle_entries;

/* Returning true lets ESP-IDF execute WAITI after this hook. Never poll here. */
static bool idle_sleep_hook(void)
{
    ++s_idle_entries;
    return true;
}

static bool IRAM_ATTR gap_elapsed(gptimer_handle_t timer,
                                  const gptimer_alarm_event_data_t *event, void *context)
{
    burst_state_t *state = context;
    BaseType_t task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(state->task, &task_woken);
    return task_woken == pdTRUE;
}

/* The task sleeps through completion; the second IRQ starts the gap. Protect
 * the shared count even if the RMT and GDMA interrupt priorities differ. */
static bool IRAM_ATTR ppm_finished(rmt_channel_handle_t channel,
                                   const rmt_tx_done_event_data_t *event, void *context)
{
    burst_state_t *state = context;
    portENTER_CRITICAL_ISR(&s_completion_lock);
    bool both_done = ++state->completed_channels == 2;
    portEXIT_CRITICAL_ISR(&s_completion_lock);
    if (both_done) {
        ESP_ERROR_CHECK(gptimer_start(state->gap_timer));
    }
    return false;
}

static void gap_timer_init(void)
{
    s_burst.task = xTaskGetCurrentTaskHandle();
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_XTAL,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 1,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&config, &s_burst.gap_timer));
    gptimer_event_callbacks_t callbacks = {.on_alarm = gap_elapsed};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_burst.gap_timer, &callbacks, &s_burst));
    ESP_ERROR_CHECK(gptimer_enable(s_burst.gap_timer));
}

/*
 * Back-port ppm_tx's nested, same-polarity waveform. A's width and rising-edge
 * interval are unchanged; B rises D ticks later and falls D ticks earlier.
 * B's leading LOW run uses the PREVIOUS interval so changing payload values
 * cannot move its pulse away from the center of the corresponding A pulse.
 * Append a LOW tail to keep both frame lengths equal and both outputs idle LOW.
 */
static inline uint32_t interval_ticks(uint8_t value)
{
    return BASE_TICKS + (value >> 1);
}

static void render_frames(void)
{
    for (int i = 0; i < N_VALUES; i++) {
        uint32_t interval = interval_ticks(i);
        uint32_t gap = (i == 0) ? DEAD_TICKS
            : interval_ticks(i - 1) - PULSE_TICKS + 2 * DEAD_TICKS;
        s_frame_a[i] = (rmt_symbol_word_t) {
            .level0 = 1, .duration0 = PULSE_TICKS,
            .level1 = 0, .duration1 = interval - PULSE_TICKS,
        };
        s_frame_b[i] = (rmt_symbol_word_t) {
            .level0 = 0, .duration0 = gap,
            .level1 = 1, .duration1 = PULSE_TICKS - 2 * DEAD_TICKS,
        };
    }
    s_frame_b[N_VALUES] = (rmt_symbol_word_t) {
        .level0 = 0, .duration0 = interval_ticks(N_VALUES - 1) - PULSE_TICKS + DEAD_TICKS,
        .level1 = 0, .duration1 = 0, /* stop after the final inter-pulse gap */
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
    uint32_t apb_hz;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_APB,
                    ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &apb_hz));
    ESP_ERROR_CHECK(apb_hz == RMT_RES_HZ ? ESP_OK : ESP_ERR_INVALID_STATE);
    ESP_ERROR_CHECK(esp_register_freertos_idle_hook(idle_sleep_hook));
    gap_timer_init();
    pwm_init();

    render_frames();

    /* 2. RMT TX channel on OUT+, DMA-capable, 80 MHz (12.5 ns ticks). */
    rmt_channel_handle_t channels[2] = {NULL, NULL};
    rmt_tx_channel_config_t chan_cfg = {
        .clk_src           = RMT_CLK_SRC_APB,       /* fixed APB, 80 MHz */
        .gpio_num          = OUT_A_GPIO,
        .resolution_hz     = RMT_RES_HZ,
        .mem_block_symbols = PHASE_A_DMA_SYMBOLS,    /* one descriptor per frame */
        .trans_queue_depth = 4,
        .intr_priority     = 1,
        .flags.with_dma    = true,                  /* burst straight from RAM */
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &channels[0]));

    /* ESP32-S3 has only one DMA TX channel. Reserve the other three memory
     * blocks for OUT- so all 129 symbols and EOF are preloaded before start. */
    chan_cfg.gpio_num = OUT_B_GPIO;
    chan_cfg.mem_block_symbols = PHASE_B_MEM_SYMBOLS;
    chan_cfg.flags.with_dma = false;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &channels[1]));

    rmt_tx_event_callbacks_t tx_callbacks = {.on_trans_done = ppm_finished};
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(channels[0], &tx_callbacks, &s_burst));
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(channels[1], &tx_callbacks, &s_burst));

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

    ESP_LOGI(TAG, "PPM TX ready: A=GPIO%d B=GPIO%d (same polarity) | outer %.1f ns, inner %.1f ns | "
                  "dead time %d%% requested, %.1f ns inset per edge | %d-value sweep",
             OUT_A_GPIO, OUT_B_GPIO, PULSE_TICKS * 1e9 / RMT_RES_HZ,
             (PULSE_TICKS - 2 * DEAD_TICKS) * 1e9 / RMT_RES_HZ,
             DEAD_TIME_PERCENT, DEAD_TICKS * 1e9 / RMT_RES_HZ, N_VALUES);

    /* 5. Fire the sweep as one finite DMA burst; repeat with a gap so a scope
     *    or logic analyzer can retrigger on it. */
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,            /* single shot, no hardware looping */
        .flags.eot_level = 0,       /* both outputs idle LOW after the burst */
    };
    const gptimer_alarm_config_t gap_alarm = {
        .alarm_count = INTER_BURST_US,
        .flags.auto_reload_on_alarm = false,
    };

    ESP_LOGI(TAG, "Power: fixed CPU/APB 80 MHz, one core, radio drivers excluded; "
                  "CPU WAITI between interrupts (not chip light-sleep)");

    uint32_t bursts = 0;
    int64_t last_status = esp_timer_get_time();
    configRUN_TIME_COUNTER_TYPE last_idle = ulTaskGetIdleRunTimeCounter();
    uint32_t last_idle_entries = s_idle_entries;
    while (1) {
        s_burst.completed_channels = 0;
        ESP_ERROR_CHECK(gptimer_set_raw_count(s_burst.gap_timer, 0));
        ESP_ERROR_CHECK(gptimer_set_alarm_action(s_burst.gap_timer, &gap_alarm));
        ESP_ERROR_CHECK(rmt_sync_reset(sync));
        ESP_ERROR_CHECK(rmt_transmit(channels[0], encoders[0], s_frame_a, sizeof(s_frame_a), &tx_cfg));
        ESP_ERROR_CHECK(rmt_transmit(channels[1], encoders[1], s_frame_b, sizeof(s_frame_b), &tx_cfg));
        /* No CPU work until both channels finish AND the 100 us alarm fires.
         * RMT/GDMA completion IRQs only arm GPTimer; its IRQ wakes this task. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_ERROR_CHECK(gptimer_stop(s_burst.gap_timer));
        /* Drain the already-completed driver transactions before requeueing. */
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(channels[0], -1));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(channels[1], -1));
        ++bursts;
        int64_t now = esp_timer_get_time();
        if (now - last_status >= STATUS_INTERVAL_US) {
            configRUN_TIME_COUNTER_TYPE idle = ulTaskGetIdleRunTimeCounter();
            uint32_t idle_entries = s_idle_entries;
            configRUN_TIME_COUNTER_TYPE idle_delta = idle - last_idle;
            ESP_LOGI(TAG, "%u bursts | idle task %.1f%% | %u WAITI entries since last report",
                     (unsigned)bursts, 100.0 * idle_delta / (now - last_status),
                     (unsigned)(idle_entries - last_idle_entries));
            last_status = now;
            last_idle = idle;
            last_idle_entries = idle_entries;
        }
    }
}
