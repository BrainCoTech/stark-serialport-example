/**
 * @file revo2_touch_pdo.cpp
 * @brief Touch PDO example for REVO2 Touch EtherCAT hand
 *
 * This example demonstrates PDO communication with touch sensor feedback.
 *
 * Features:
 * - All control modes (position, speed, current, PWM, gestures)
 * - Touch sensor feedback reading
 */

#include "ec_common.h"

/****************************************************************************/
// Touch feedback wrapper
/****************************************************************************/

static void read_touch_feedback_wrapper() {
  ec_app_context_t *ctx = ec_app_get_context();
  touch_feedback_t feedback;
  ec_read_touch_feedback_data(ctx->domain_data, ctx->off_in, &feedback);
  ec_print_touch_feedback_data(&feedback);
}

/****************************************************************************/
// Main cyclic task
/****************************************************************************/

void cyclic_task() {
  // Use generic cyclic task with touch feedback callback
  ec_app_run_cyclic_task(read_touch_feedback_wrapper);
}

int main(int argc, char **argv) {
  return ec_app_main_pdo(PDO_CONFIG_TOUCH, cyclic_task);
}
