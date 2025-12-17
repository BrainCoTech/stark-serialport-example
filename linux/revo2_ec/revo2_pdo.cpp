/**
 * @file revo2_pdo.cpp
 * @brief Basic PDO example for REVO2 EtherCAT hand
 *
 * This example demonstrates basic PDO (Process Data Object) communication
 * with the REVO2 EtherCAT hand using the ec_app framework.
 *
 * The demo automatically cycles through different control modes:
 * - Position with duration
 * - Position with speed
 * - Speed control
 * - Current control
 * - PWM control
 * - Fist gesture
 * - Open hand gesture
 * - OK gesture
 */

#include "ec_common.h"
#include <stdlib.h> // for atoi

/****************************************************************************/
// Main cyclic task
/****************************************************************************/

void cyclic_task() {
  // Use generic cyclic task without additional feedback callback (only joint
  // feedback)
  ec_app_run_cyclic_task(NULL);
}

int main(int argc, char **argv) {
  uint16_t slave_pos = 0; // Slave position (can be changed via command line argument)
  if (argc > 1) {
    slave_pos = (uint16_t)atoi(argv[1]);
  }
  return ec_app_main_pdo(PDO_CONFIG_BASIC, slave_pos, cyclic_task);
}
