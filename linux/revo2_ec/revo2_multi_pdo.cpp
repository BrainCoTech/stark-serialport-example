/**
 * @file revo2_multi_pdo.cpp
 * @brief Multi-slave PDO example (mixed PDO configs)
 *
 * Default:
 *   - slave 0: BASIC
 *   - slave 1: TOUCH
 *
 * Usage:
 *   ./revo2_multi_pdo.exe [slave0_pos] [slave1_pos]
 *     slave0_pos default 0
 *     slave1_pos default 1
 */

#include "ec_common.h"
#include <stdlib.h> // for atoi
#include "ec_app.h"

static void cyclic_task() {
  // Reuse generic cyclic task; demo/feedback uses the first slave offsets
  ec_app_run_cyclic_task(NULL);
}

int main(int argc, char **argv) {
  uint16_t slave0_pos = 1;
  uint16_t slave1_pos = 2;
  if (argc > 1) {
    slave0_pos = (uint16_t)atoi(argv[1]);
  }
  if (argc > 2) {
    slave1_pos = (uint16_t)atoi(argv[2]);
  }

  ec_app_context_t *ctx = ec_app_get_context();

  // Single-slave config for now (use TOUCH_PRESSURE on slave1_pos)
//   const int slave_count = 1;
//   ec_slave_runtime_t slaves[1] = {
//       {.slave_pos = slave1_pos,
//        .pdo_type = PDO_CONFIG_TOUCH_PRESSURE,
//        .syncs = get_slave_pdo_syncs(PDO_CONFIG_TOUCH_PRESSURE)}};

  // Example: two slaves, default positions 1 and 2
  const int slave_count = 2;
  ec_slave_runtime_t slaves[2] = {
      {.slave_pos = slave0_pos,
       .pdo_type = PDO_CONFIG_TOUCH,
       .syncs = get_slave_pdo_syncs(PDO_CONFIG_TOUCH)},
      {.slave_pos = slave1_pos,
       .pdo_type = PDO_CONFIG_TOUCH_PRESSURE,
       .syncs = get_slave_pdo_syncs(PDO_CONFIG_TOUCH_PRESSURE)}};

  const int REALTIME_PRIORITY = 49;
  if (ec_app_init_pdo_multi(ctx, slaves, slave_count, REALTIME_PRIORITY) != 0) {
    ec_app_cleanup(ctx);
    return -1;
  }

  cyclic_task();

  ec_app_cleanup(ctx);
  return 0;
}

