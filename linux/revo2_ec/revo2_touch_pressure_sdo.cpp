/**
 * @file revo2_touch_pressure_sdo.cpp
 * @brief Touch Pressure SDO example for REVO2 EtherCAT hand
 */

// Standard C headers
#include <stdio.h>  // printf
#include <unistd.h> // sleep

// EtherCAT library (includes all EC modules)
#include "ec_common.h"

/****************************************************************************/
// Touch Pressure SDO demo function using ec_sdo library
/****************************************************************************/

void touch_pressure_sdo_demo_function() {
  // Get application context
  ec_app_context_t *ctx = ec_app_get_context();

  printf("\n=== Touch Pressure SDO Configuration Demo ===\n");

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Use common helper functions to read and print configurations
  print_firmware_info(ctx->slave_config);
  print_general_config(ctx->slave_config);
  print_protection_current_config(ctx->slave_config);
  print_pressure_touch_config(ctx->slave_config);

  // Demonstrate configuration changes
  demo_basic_controls(ctx->slave_config);

  // Demonstrate pressure-specific configuration
  printf("\n=== Pressure Data Type Demo ===\n");
  set_pressure_touch_data_type(ctx->slave_config, RAW);
  usleep(500000); // 500ms
  set_pressure_touch_data_type(ctx->slave_config, CALIBRATED);
  usleep(500000); // 500ms
  set_pressure_touch_data_type(ctx->slave_config, FORCE);

  printf("\n=== Touch Pressure SDO Demo Completed ===\n");
}

/****************************************************************************/
// Main function
/****************************************************************************/

int main(int argc, char **argv) {
  return ec_app_main_sdo(touch_pressure_sdo_demo_function);
}
