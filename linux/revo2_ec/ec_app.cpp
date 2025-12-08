/**
 * @file ec_app.cpp
 * @brief EtherCAT application-level initialization and management
 * implementation
 */

#include "ec_common.h"
#include <execinfo.h>
#include <malloc.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

/****************************************************************************/
// Signal handling (for graceful shutdown and crash debugging)
/****************************************************************************/

// Global flag for graceful shutdown
static volatile sig_atomic_t g_shutdown_requested = 0;

static void crash_signal_handler(int sig) {
  void *array[10];
  size_t size;

  // Get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // Print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

static void shutdown_signal_handler(int sig) {
  printf("\n[Signal %d] Shutdown requested, cleaning up...\n", sig);
  g_shutdown_requested = 1;
}

static void setup_signal_handlers(void) {
  // Handle crash signals (for debugging)
  signal(SIGSEGV, crash_signal_handler);
  signal(SIGABRT, crash_signal_handler);

  // Handle shutdown signals (for graceful exit)
  signal(SIGINT, shutdown_signal_handler);  // Ctrl+C
  signal(SIGTERM, shutdown_signal_handler); // kill command
}

int is_shutdown_requested(void) { return g_shutdown_requested; }

/****************************************************************************/
// System initialization (realtime, master/domain)
/****************************************************************************/

static int setup_realtime(int priority) {
  struct sched_param param = {};
  param.sched_priority = priority;

  printf("Using priority %i.\n", param.sched_priority);
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
    return -1;
  }

  /* Lock memory */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    fprintf(stderr, "Warning: Failed to lock memory: ");
    perror("");
  }

  return 0;
}

static int init_ethercat_master_domain(ec_master_t **master_ptr,
                                       ec_domain_t **domain_ptr) {
  // 1. Request EtherCAT master
  printf("Requesting master...\n");
  *master_ptr = ecrt_request_master(0);
  if (!*master_ptr) {
    fprintf(stderr, "Failed to request master.\n");
    return -1;
  }

  // 2. Create domain (if domain_ptr is not NULL)
  if (domain_ptr) {
    printf("Creating domain...\n");
    *domain_ptr = ecrt_master_create_domain(*master_ptr);
    if (!*domain_ptr) {
      fprintf(stderr, "Failed to create domain.\n");
      ecrt_release_master(*master_ptr);
      *master_ptr = NULL;
      return -1;
    }
  }

  return 0;
}

static int setup_ethercat(ec_master_t **master_ptr, ec_domain_t **domain_ptr,
                          int priority) {
  // 1. Setup signal handlers for crash debugging
  setup_signal_handlers();

  // 2. Setup real-time scheduling (skip for SDO-only operations)
  if (priority > 0) {
    if (setup_realtime(priority) != 0) {
      return -1;
    }
  }

  // 3. Initialize EtherCAT master and domain
  if (init_ethercat_master_domain(master_ptr, domain_ptr) != 0) {
    return -1;
  }

  return 0;
}

/****************************************************************************/
// Application context
/****************************************************************************/

// Global application context
static ec_app_context_t g_app_ctx = {0};

/****************************************************************************/
// Internal helper functions
/****************************************************************************/

static int setup_slave_config(ec_app_context_t *ctx,
                              ec_sync_info_t *slave_syncs) {
  printf("Requesting slave configuration...\n");
  ctx->slave_config = ecrt_master_slave_config(
      ctx->master, 0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE);
  if (!ctx->slave_config) {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return -1;
  }

  if (slave_syncs) {
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(ctx->slave_config, EC_END, slave_syncs)) {
      fprintf(stderr, "Failed to configure PDOs.\n");
      return -1;
    }
  }

  return 0;
}

static int setup_pdo_registration(ec_app_context_t *ctx,
                                  pdo_config_type_t pdo_type) {
  printf("Setting up PDO registration...\n");

  // Get appropriate domain registration based on PDO config type
  ctx->domain_regs = get_domain_regs(pdo_type, &ctx->off_in, &ctx->off_out);

  printf("Registering PDO entries...\n");
  if (ecrt_domain_reg_pdo_entry_list(ctx->domain, ctx->domain_regs)) {
    fprintf(stderr, "PDO entry registration failed!\n");
    return -1;
  }

  printf("PDO entry registration successful, off_in: %d, off_out: %d\n",
         ctx->off_in, ctx->off_out);
  return 0;
}

static int activate_master(ec_app_context_t *ctx) {
  printf("Activating master...\n");
  if (ecrt_master_activate(ctx->master)) {
    fprintf(stderr, "Failed to activate master.\n");
    return -1;
  }

  if (ctx->app_type != EC_APP_SDO_ONLY) {
    ctx->domain_data = ecrt_domain_data(ctx->domain);
    if (!ctx->domain_data) {
      fprintf(stderr, "Failed to get domain data.\n");
      return -1;
    }
  }

  return 0;
}

/****************************************************************************/
// Public API functions
/****************************************************************************/

int ec_app_init_pdo(ec_app_context_t *ctx, pdo_config_type_t pdo_type,
                    ec_sync_info_t *slave_syncs, int realtime_priority) {
  ctx->app_type = EC_APP_PDO_ONLY;

  // Setup EtherCAT system (signal handling + realtime + master/domain)
  if (setup_ethercat(&ctx->master, &ctx->domain, realtime_priority) != 0) {
    return -1;
  }

  // Setup slave configuration
  if (setup_slave_config(ctx, slave_syncs) != 0) {
    return -1;
  }

  // Setup PDO registration
  if (setup_pdo_registration(ctx, pdo_type) != 0) {
    return -1;
  }

  // Activate master and get domain data
  if (activate_master(ctx) != 0) {
    return -1;
  }

  printf("EtherCAT PDO application initialized successfully.\n");
  return 0;
}

int ec_app_init_sdo(ec_app_context_t *ctx) {
  ctx->app_type = EC_APP_SDO_ONLY;

  // Setup EtherCAT system (no domain needed for SDO-only)
  if (setup_ethercat(&ctx->master, NULL, 0) != 0) {
    return -1;
  }

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Setup slave configuration (no PDO sync needed)
  if (setup_slave_config(ctx, NULL) != 0) {
    return -1;
  }

  printf("EtherCAT SDO application initialized successfully.\n");

  return 0;
}

void ec_app_cleanup(ec_app_context_t *ctx) {
  if (ctx->master) {
    printf("Releasing EtherCAT master...\n");
    ecrt_release_master(ctx->master);
    ctx->master = NULL;
  }

  // Clear context
  ctx->domain = NULL;
  ctx->slave_config = NULL;
  ctx->domain_data = NULL;
  ctx->domain_regs = NULL;
}

ec_app_context_t *ec_app_get_context(void) { return &g_app_ctx; }

/****************************************************************************/
// Simplified main function helpers
/****************************************************************************/

int ec_app_main_pdo(pdo_config_type_t pdo_type, ec_callback_t cyclic_func) {
  ec_app_context_t *ctx = &g_app_ctx;
  const int REALTIME_PRIORITY = 49;

  printf("=== EtherCAT PDO Application Starting ===\n");

  // Initialize PDO application
  if (ec_app_init_pdo(ctx, pdo_type, get_slave_pdo_syncs(pdo_type),
                      REALTIME_PRIORITY) != 0) {
    ec_app_cleanup(ctx);
    return -1;
  }

  // Run cyclic function
  printf("Starting cyclic function.\n");
  if (cyclic_func) {
    cyclic_func();
  }

  // Cleanup
  ec_app_cleanup(ctx);
  return 0;
}

int ec_app_main_sdo(ec_callback_t demo_func) {
  ec_app_context_t *ctx = &g_app_ctx;

  printf("=== EtherCAT SDO Application Starting ===\n");

  // Initialize SDO application
  if (ec_app_init_sdo(ctx) != 0) {
    ec_app_cleanup(ctx);
    return -1;
  }

  // Run demo function
  if (demo_func) {
    demo_func();
  }

  // Cleanup
  ec_app_cleanup(ctx);
  return 0;
}

/****************************************************************************/
// Generic cyclic task implementation
/****************************************************************************/

void ec_app_run_cyclic_task(ec_callback_t read_feedback_func) {
  // Get application context to access master and domain data
  ec_app_context_t *ctx = ec_app_get_context();
  ec_master_t *master = ctx->master;
  ec_domain_t *domain = ctx->domain;
  uint8_t *domain_data = ctx->domain_data;
  unsigned int off_in = ctx->off_in;
  unsigned int off_out = ctx->off_out;

  // Local state variables
  ec_master_state_t master_state = {};
  ec_domain_state_t domain_state = {};
  unsigned int counter = 0;
  unsigned int sync_ref_counter = 0;
  const struct timespec cycletime = {0, PERIOD_NS};

  // Trajectory control
  TrajectoryControl traj_ctrl = {0};
  ec_init_trajectory(&traj_ctrl);

  // Demo control state
  bool initial_command_sent = false;
  int demo_stage = 0;
  int demo_counter = 0;

  struct timespec wakeupTime, time;
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

  printf("Press Ctrl+C to exit gracefully.\n");

  while (!is_shutdown_requested()) {
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // Write application time to master
    ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // check process data state (optional)
    check_domain_state(domain, &domain_state);

    if (counter) {
      counter--;
    } else {
      counter = FREQUENCY;
      check_master_state(master, &master_state);
      printf("Master AL state: 0x%02X (need 0x08 for OPERATIONAL)\n",
             master_state.al_states);
    }

    // Only perform read/write operations when the slave is in OPERATIONAL state
    if (master_state.al_states == 0x08) {
      // Demonstrate different control modes and gestures
      if (!initial_command_sent) {
        printf("=== Starting Joint Control Demo ===\n");
        initial_command_sent = true;
      }

      // Switch demo mode every 20 seconds
      if (demo_counter++ % (FREQUENCY * 20) == 0) {
        switch (demo_stage % 8) {
        case 0: {
          // Position-and-time control mode
          uint16_t positions[6] = {400, 400, 1000, 1000, 1000, 1000};
          uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
          EC_WRITE_U16(domain_data + off_out, PositionWithDuration);
          memcpy(domain_data + off_out + 2, positions, 12);
          memcpy(domain_data + off_out + 14, durations, 12);
          ec_set_all_joints_position_duration(&traj_ctrl, positions, durations);
          break;
        }
        case 1: {
          // Position-and-speed control mode
          uint16_t positions[6] = {400, 400, 0, 0, 0, 0};
          uint16_t speeds[6] = {300, 300, 300, 300, 300, 300};
          EC_WRITE_U16(domain_data + off_out, PositionWithSpeed);
          memcpy(domain_data + off_out + 2, positions, 12);
          memcpy(domain_data + off_out + 14, speeds, 12);
          break;
        }
        case 2: {
          // Speed control mode
          int16_t speeds[6] = {0, 0, 300, 300, 300, 300};
          EC_WRITE_U16(domain_data + off_out, Speed);
          memcpy(domain_data + off_out + 2, speeds, 12);
          memset(domain_data + off_out + 14, 0, 12);
          break;
        }
        case 3: {
          // Current control mode
          int16_t currents[6] = {-100, -100, -500, -500, -500, -500};
          EC_WRITE_U16(domain_data + off_out, Current);
          memcpy(domain_data + off_out + 2, currents, 12);
          memset(domain_data + off_out + 14, 0, 12);
          break;
        }
        case 4: {
          // PWM control mode
          uint16_t pwm_values[6] = {200, 100, 500, 500, 500, 500};
          EC_WRITE_U16(domain_data + off_out, Pwm);
          memcpy(domain_data + off_out + 2, pwm_values, 12);
          memset(domain_data + off_out + 14, 0, 12);
          break;
        }
        case 5: {
          // Fist gesture
          ec_set_fist_gesture(domain_data, off_out, 400);
          break;
        }
        case 6: {
          // Open-hand gesture
          ec_set_open_hand_gesture(domain_data, off_out, 500);
          break;
        }
        case 7: {
          // OK gesture
          ec_set_ok_gesture(domain_data, off_out, 400);
          break;
        }
        }
        demo_stage++;
      }

      // Update trajectory control
      ec_update_trajectory(&traj_ctrl, domain_data, off_out);

      // Periodically print feedback data
      static int print_counter = 0;
      if (print_counter++ % 10 == 0) {
        // Read joint feedback (always)
        joint_feedback_t feedback;
        ec_read_joint_feedback_data(domain_data, off_in, &feedback);
        ec_print_joint_feedback_data(&feedback);

        // Read additional feedback if callback provided
        if (read_feedback_func) {
          read_feedback_func();
        }
      }
    }

    if (sync_ref_counter) {
      sync_ref_counter--;
    } else {
      sync_ref_counter = 1;
      clock_gettime(CLOCK_TO_USE, &time);
      ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
    }
    ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
  }
}
