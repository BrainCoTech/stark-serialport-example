/**
 * @file revo2_multi_pdo.cpp
 * @brief Multi-slave PDO example with auto-detection
 *
 * PDO types are automatically detected from each slave's configured sync managers:
 *   - BASIC: Only 0x1A00 (no 0x1A01)
 *   - TOUCH: 0x1A00 + 0x1A01 with 5 entries
 *   - TOUCH_PRESSURE: 0x1A00 + 0x1A01 with 10 entries
 *
 * Usage:
 *   ./revo2_multi_pdo.exe [slave0_pos] [slave1_pos]
 *     slave0_pos default 1
 *     slave1_pos default 2
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

  EC_LOG_TIME_PREFIX(stdout);
  printf("=== EtherCAT Multi-Slave PDO Application (Auto-detect mode) ===\n");
  printf("Slave 0 position: %u (PDO type will be auto-detected)\n", slave0_pos);
  printf("Slave 1 position: %u (PDO type will be auto-detected)\n", slave1_pos);

  // Auto-detect PDO type for each slave
  // PDO type will be automatically detected during setup_pdo_registration
  // based on the configured sync managers (0x1A01 entry count: 5=TOUCH, 10=TOUCH_PRESSURE, none=BASIC)
  
  // Single-slave config example
//   const int slave_count = 1;
//   ec_slave_runtime_t slaves[1] = {
//       {.slave_pos = slave0_pos,
//        .pdo_type = PDO_CONFIG_BASIC}};  // Will be auto-detected

  // Example: two slaves, default positions 1 and 2
  // PDO types will be auto-detected from each slave's configuration
  const int slave_count = 2;
  ec_slave_runtime_t slaves[2] = {
      {.slave_pos = slave0_pos,
       .pdo_type = PDO_CONFIG_BASIC},  // Will be auto-detected
      {.slave_pos = slave1_pos,
       .pdo_type = PDO_CONFIG_BASIC}};  // Will be auto-detected

  const int REALTIME_PRIORITY = 49;
  if (ec_app_init_pdo_multi(ctx, slaves, slave_count, REALTIME_PRIORITY) != 0) {
    ec_app_cleanup(ctx);
    return -1;
  }

  cyclic_task();

  ec_app_cleanup(ctx);
  return 0;
}

