/**
 * @file revo2_touch_sdo.cpp
 * @brief Touch SDO example for REVO2 Touch EtherCAT hand
 */

// Standard C headers
#include <stdio.h>  // printf
#include <unistd.h> // sleep

// EtherCAT library (includes all EC modules)
#include "ec_common.h"

/****************************************************************************/
// Touch SDO demo function using ec_sdo library
/****************************************************************************/

void touch_sdo_demo_function() {
  // Get application context
  ec_app_context_t *ctx = ec_app_get_context();

  printf("\n=== Touch SDO Configuration Demo ===\n");

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Use common helper functions to read and print configurations
  print_firmware_info(ctx->slave_config);
  print_general_config(ctx->slave_config);
  print_protection_current_config(ctx->slave_config);
  print_touch_config(ctx->slave_config);

  // Demonstrate configuration changes
  demo_basic_controls(ctx->slave_config);

  printf("\n=== Touch SDO Demo Completed ===\n");
}

/****************************************************************************/
// Main function
/****************************************************************************/

int main(int argc, char **argv) {
  return ec_app_main_sdo(touch_sdo_demo_function);
}
