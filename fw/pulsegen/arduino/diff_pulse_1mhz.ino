/*
 * Arduino-ESP32 (v3.x) version of the 1 MHz complementary pulse generator.
 * Board: Heltec WiFi LoRa 32 V3 (HTIT-WB32LA(F)_V3, ESP32-S3).
 *
 * Arduino-ESP32 3.x ships ESP-IDF v5.x underneath, so the same modern
 * MCPWM driver is available. Select board "Heltec WiFi LoRa 32(V3)".
 *
 *   OUT+ = GPIO5,  OUT- = GPIO6,  1 MHz,  6.25 ns high-time.
 */

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

#define GEN_A_GPIO     5
#define GEN_B_GPIO     6
#define TIMER_RES_HZ   160000000   // 6.25 ns / tick
#define PERIOD_TICKS   160         // -> 1.000 MHz
#define PULSE_TICKS    1           // -> 6.25 ns high-time (smallest)

void setup() {
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_cfg = {
    .group_id      = 0,
    .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = TIMER_RES_HZ,
    .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks  = PERIOD_TICKS,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  mcpwm_cmpr_handle_t cmp = NULL;
  mcpwm_comparator_config_t cmp_cfg = { .flags = { .update_cmp_on_tez = true } };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &cmp));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp, PULSE_TICKS));

  mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;
  mcpwm_generator_config_t gen_a_cfg = { .gen_gpio_num = GEN_A_GPIO };
  mcpwm_generator_config_t gen_b_cfg = { .gen_gpio_num = GEN_B_GPIO };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_a_cfg, &gen_a));
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_b_cfg, &gen_b));

  // OUT+ : HIGH at zero, LOW at compare
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_a,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));

  // OUT- : exact inverse
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_b,
      MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                   MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_HIGH)));

  gpio_set_drive_capability((gpio_num_t)GEN_A_GPIO, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)GEN_B_GPIO, GPIO_DRIVE_CAP_3);

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void loop() {
  delay(1000);   // everything runs in hardware; nothing to do here
}
