/**
 * @file revo2_touch_sdo.cpp
 * @brief Touch SDO example for REVO2 Touch EtherCAT hand
 */

// Standard C headers
#include <stdio.h>  // printf
#include <stdlib.h> // for atoi
#include <unistd.h> // sleep

// EtherCAT library (includes all EC modules)
#include "ec_common.h"

/****************************************************************************/
// Touch SDO demo function using ec_sdo library
/****************************************************************************/

// Global variable to store slave position for demo function
static uint16_t g_slave_pos = 0;

void touch_sdo_demo_function() {
  // Get application context
  ec_app_context_t *ctx = ec_app_get_context();

  printf("\n=== Touch SDO Configuration Demo ===\n");

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Use common helper functions to read and print configurations
  print_firmware_info(ctx->slave_config, g_slave_pos);
  print_general_config(ctx->slave_config, g_slave_pos);
  print_protection_current_config(ctx->slave_config, g_slave_pos);
  print_touch_config(ctx->slave_config, g_slave_pos);

  // Demonstrate configuration changes
  demo_basic_controls(ctx->slave_config, g_slave_pos);

  printf("\n=== Touch SDO Demo Completed ===\n");
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
  return ec_app_main_sdo(slave_pos, touch_sdo_demo_function);
}
