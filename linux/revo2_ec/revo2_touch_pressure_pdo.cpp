/**
 * @file revo2_touch_pressure_pdo.cpp
 * @brief Touch + Pressure PDO example for REVO2 EtherCAT hand
 *
 * This example demonstrates PDO communication with touch and pressure sensor
 * feedback.
 *
 * Features:
 * - All control modes (position, speed, current, PWM, gestures)
 * - Pressure sensor feedback reading
 */

#include "ec_common.h"
#include <stdlib.h> // for atoi

/****************************************************************************/
// Pressure touch feedback wrapper
/****************************************************************************/

static void read_pressure_touch_feedback_wrapper() {
  ec_app_context_t *ctx = ec_app_get_context();
  pressure_touch_feedback_t feedback;
  // Use first slave's input offset for feedback
  ec_read_pressure_touch_feedback_data(ctx->domain_data, ctx->slaves[0].off_in,
                                       &feedback);
  ec_print_pressure_touch_feedback_data(&feedback);
}

/****************************************************************************/
// Main cyclic task
/****************************************************************************/

void cyclic_task() {
  // Use generic cyclic task with pressure touch feedback callback
  ec_app_run_cyclic_task(read_pressure_touch_feedback_wrapper);
}

int main(int argc, char **argv) {
  uint16_t slave_pos = 0; // Slave position (can be changed via command line argument)
  if (argc > 1) {
    slave_pos = (uint16_t)atoi(argv[1]);
  }
  return ec_app_main_pdo(PDO_CONFIG_TOUCH_PRESSURE, slave_pos, cyclic_task);
}
