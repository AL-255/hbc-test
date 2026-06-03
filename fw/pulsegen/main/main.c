/*
 * Complementary (differential) 1 MHz pulse generator
 * Board : Heltec WiFi LoRa 32 V3  (HTIT-WB32LA(F)_V3, ESP32-S3)
 *
 * Uses the ESP32-S3 MCPWM peripheral. A single timer drives two generators,
 * so OUT- is the *exact* inverse of OUT+ with no firmware skew (they switch
 * on the same hardware timer events).
 *
 *   Repetition rate : 160 MHz timer / 160 ticks = 1.000 MHz
 *   Pulse high-time : 1 tick = 6.25 ns  (smallest the peripheral can emit)
 *
 *   OUT+  GPIO5  ---\
 *                    >--- differential pair to your receiver (terminate!)
 *   OUT-  GPIO6  ---/
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "diff_pulse";

/* ---- User configuration ------------------------------------------------ */
#define GEN_A_GPIO     5            /* OUT+  (true output)          */
#define GEN_B_GPIO     6            /* OUT-  (complementary output) */

#define TIMER_RES_HZ   160000000    /* 160 MHz: max MCPWM clock on S3 -> 6.25 ns/tick */
#define PERIOD_TICKS   160          /* 160 MHz / 160 = 1.000 MHz repetition rate      */
#define PULSE_TICKS    1            /* high-time in ticks; 1 = 6.25 ns (smallest)     */
/* ------------------------------------------------------------------------ */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting 1 MHz complementary pulse generator");

    /* 1. Timer: counts up 0..PERIOD_TICKS-1 then wraps -> 1 MHz */
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,   /* PLL160M */
        .resolution_hz = TIMER_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    /* 2. Operator, connected to the timer */
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    /* 3. Comparator defines the pulse width (high-time) */
    mcpwm_cmpr_handle_t cmp = NULL;
    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &cmp));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp, PULSE_TICKS));

    /* 4. Two generators driven by the SAME timer/comparator -> zero skew */
    mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;
    mcpwm_generator_config_t gen_a_cfg = { .gen_gpio_num = GEN_A_GPIO };
    mcpwm_generator_config_t gen_b_cfg = { .gen_gpio_num = GEN_B_GPIO };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_a_cfg, &gen_a));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_b_cfg, &gen_b));

    /* OUT+ : HIGH at counter==0, LOW at counter==PULSE_TICKS  -> narrow high pulse */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_a,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmp, MCPWM_GEN_ACTION_LOW)));

    /* OUT- : exact inverse (LOW at counter==0, HIGH at counter==PULSE_TICKS) */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_b,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmp, MCPWM_GEN_ACTION_HIGH)));

    /* 5. Strongest pad drive for the sharpest edges on a 6.25 ns pulse */
    gpio_set_drive_capability(GEN_A_GPIO, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(GEN_B_GPIO, GPIO_DRIVE_CAP_3);

    /* 6. Enable and run forever */
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    ESP_LOGI(TAG, "Running: OUT+=GPIO%d  OUT-=GPIO%d  rate=1 MHz  high-time=%.2f ns",
             GEN_A_GPIO, GEN_B_GPIO, (double)PULSE_TICKS * 1e9 / TIMER_RES_HZ);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
