/**
 * @file revo2_sdo.cpp
 * @brief REVO2 SDO example for EtherCAT hand
 */

// Standard C headers
#include <stdio.h>  // printf
#include <unistd.h> // usleep

// EtherCAT library (includes all EC modules)
#include "ec_common.h"

/****************************************************************************/
// SDO Demo Function
/****************************************************************************/

void sdo_demo_function() {
  // Get application context
  ec_app_context_t *ctx = ec_app_get_context();

  printf("\n=== Starting SDO Demo ===\n");

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Use common helper functions to read and print configurations
  print_firmware_info(ctx->slave_config);
  print_general_config(ctx->slave_config);
  print_protection_current_config(ctx->slave_config);

  // Demonstrate configuration changes
  demo_basic_controls(ctx->slave_config);

  printf("\n=== SDO Demo Completed ===\n");
}

/****************************************************************************/
// Main function
/****************************************************************************/

int main(int argc, char **argv) { return ec_app_main_sdo(sdo_demo_function); }
