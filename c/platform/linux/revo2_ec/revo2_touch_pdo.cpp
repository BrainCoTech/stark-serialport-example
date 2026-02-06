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
#include <stdlib.h> // for atoi

/****************************************************************************/
// Touch feedback wrapper
/****************************************************************************/

static void read_touch_feedback_wrapper() {
  ec_app_context_t *ctx = ec_app_get_context();
  touch_feedback_t feedback;
  ec_read_touch_feedback_data(ctx->domain_data, ctx->slaves[0].off_in, &feedback);
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
  uint16_t slave_pos = 0; // Slave position (can be changed via command line argument)
  if (argc > 1) {
    slave_pos = (uint16_t)atoi(argv[1]);
  }
  return ec_app_main_pdo_auto(slave_pos, cyclic_task);
}
