/**
 * @file revo2_touch_pressure_sdo.cpp
 * @brief Touch Pressure SDO example for REVO2 EtherCAT hand
 */

// Standard C headers
#include <stdio.h>  // printf
#include <stdlib.h> // for atoi
#include <unistd.h> // sleep

// EtherCAT library (includes all EC modules)
#include "ec_common.h"

/****************************************************************************/
// Touch Pressure SDO demo function using ec_sdo library
/****************************************************************************/

// Global variable to store slave position for demo function
static uint16_t g_slave_pos = 0;

void touch_pressure_sdo_demo_function() {
  // Get application context
  ec_app_context_t *ctx = ec_app_get_context();

  printf("\n=== Touch Pressure SDO Configuration Demo ===\n");

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Use common helper functions to read and print configurations
  print_firmware_info(ctx->slave_config, g_slave_pos);
  print_general_config(ctx->slave_config, g_slave_pos);
  print_protection_current_config(ctx->slave_config, g_slave_pos);
  print_pressure_touch_config(ctx->slave_config, g_slave_pos);

  // Demonstrate configuration changes
  demo_basic_controls(ctx->slave_config, g_slave_pos);

  // Demonstrate pressure-specific configuration
  printf("\n=== Pressure Data Type Demo ===\n");
  set_pressure_touch_data_type(ctx->slave_config, g_slave_pos, RAW);
  usleep(500000); // 500ms
  set_pressure_touch_data_type(ctx->slave_config, g_slave_pos, CALIBRATED);
  usleep(500000); // 500ms
  set_pressure_touch_data_type(ctx->slave_config, g_slave_pos, FORCE);

  printf("\n=== Touch Pressure SDO Demo Completed ===\n");
}

/****************************************************************************/
// Main function
/****************************************************************************/

int main(int argc, char **argv) {
  uint16_t slave_pos = 0; // Default slave position (can be changed via command line argument)
  if (argc > 1) {
    slave_pos = (uint16_t)atoi(argv[1]);
  }
  g_slave_pos = slave_pos; // Store for demo function
  return ec_app_main_sdo(slave_pos, touch_pressure_sdo_demo_function);
}
