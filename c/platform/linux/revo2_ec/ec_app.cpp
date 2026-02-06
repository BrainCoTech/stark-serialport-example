/**
 * @file ec_app.cpp
 * @brief EtherCAT application-level initialization and management
 * implementation
 */

#include "ec_common.h"
#include <errno.h>
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
  EC_LOG_TIME_PREFIX(stdout);
  printf("Requesting master...\n");
  *master_ptr = ecrt_request_master(0);
  if (!*master_ptr) {
    fprintf(stderr, "Failed to request master.\n");
    return -1;
  }

  // 2. Create domain (if domain_ptr is not NULL)
  if (domain_ptr) {
    EC_LOG_TIME_PREFIX(stdout);
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

static int setup_slave_config(ec_app_context_t *ctx, ec_slave_runtime_t *slave_rt) {
  EC_LOG_TIME_PREFIX(stdout);
  printf("Requesting slave configuration (slave_pos=%u)...\n", slave_rt->slave_pos);
  ctx->slave_config = ecrt_master_slave_config(
      ctx->master, 0, slave_rt->slave_pos, STARK_VENDOR_ID, STARK_PRODUCT_CODE);
  if (!ctx->slave_config) {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return -1;
  }

  // Configure PDOs if syncs is available (will be set in setup_pdo_registration)
  // Note: syncs may be NULL initially, it will be set during setup_pdo_registration
  // We configure PDOs after setup_pdo_registration sets syncs
  return 0;
}

static int setup_pdo_registration(ec_app_context_t *ctx, ec_slave_runtime_t *slave_rt) {
  EC_LOG_TIME_PREFIX(stdout);
  printf("Setting up PDO registration (slave_pos=%u)...\n", slave_rt->slave_pos);

  // Try to read PDO mappings from slave's configured sync managers
  // This reads from currently active sync managers (similar to Rust's ioctl method)
  // Note: This requires master to be activated and slave to be configured
  int pdo_count = read_pdo_from_sync_managers(ctx->master, slave_rt);
  if (pdo_count <= 0) {
    // Fallback to hardcoded PDO configuration if sync manager read fails
    EC_LOG_TIME_PREFIX(stdout);
    printf("Sync manager read failed (slave may not be configured yet), "
           "falling back to hardcoded configuration (slave_pos=%u, pdo_type=%d)...\n",
           slave_rt->slave_pos, slave_rt->pdo_type);
    build_domain_regs(slave_rt->pdo_type, slave_rt->slave_pos,
                      &slave_rt->off_in, &slave_rt->off_out, slave_rt->domain_regs);
    slave_rt->syncs = get_slave_pdo_syncs(slave_rt->pdo_type);
  } else {
    EC_LOG_TIME_PREFIX(stdout);
    printf("Successfully read %d PDO entries from sync managers (slave %u)\n", 
           pdo_count, slave_rt->slave_pos);
    // syncs is already set by read_pdo_from_sync_managers
  }

  EC_LOG_TIME_PREFIX(stdout);
  printf("Registering PDO entries (slave_pos=%u)...\n", slave_rt->slave_pos);
  
  // Debug: Print domain_regs before registration
  for (int i = 0; i < 3; i++) {
    if (slave_rt->domain_regs[i].index == 0 && slave_rt->domain_regs[i].subindex == 0) {
      EC_LOG_TIME_PREFIX(stdout);
      printf("  Entry[%d]: terminator\n", i);
      break;
    }
    EC_LOG_TIME_PREFIX(stdout);
    printf("  Entry[%d]: alias=%u, pos=%u, vendor=0x%08X, product=0x%08X, "
           "index=0x%04X, subindex=0x%02X\n",
           i, slave_rt->domain_regs[i].alias, slave_rt->domain_regs[i].position,
           slave_rt->domain_regs[i].vendor_id, slave_rt->domain_regs[i].product_code,
           slave_rt->domain_regs[i].index, slave_rt->domain_regs[i].subindex);
  }
  
  if (ecrt_domain_reg_pdo_entry_list(ctx->domain, slave_rt->domain_regs)) {
    fprintf(stderr, "Failed to register PDO entry: %s\n", strerror(errno));
    fprintf(stderr, "PDO entry registration failed for slave %u!\n", slave_rt->slave_pos);
    return -1;
  }

  EC_LOG_TIME_PREFIX(stdout);
  printf("PDO entry registration successful, slave_pos=%u, off_in: %u, off_out: %u\n",
         slave_rt->slave_pos, slave_rt->off_in, slave_rt->off_out);
  return 0;
}

// Demo control: cycle gestures for all slaves every DEMO_INTERVAL
static void demo_apply_gesture_all(ec_app_context_t *ctx, uint8_t *domain_data, int demo_stage) {
  int slave_count = ctx->slave_count;
  switch (demo_stage) {
  case 0:
    printf("[Demo] Fist gesture (all slaves)\n");
    for (int i = 0; i < slave_count; ++i) {
      ec_set_fist_gesture(domain_data, ctx->slaves[i].off_out, 500);
    }
    break;
  case 1:
    printf("[Demo] Open-hand gesture (all slaves)\n");
    for (int i = 0; i < slave_count; ++i) {
      ec_set_open_hand_gesture(domain_data, ctx->slaves[i].off_out, 500);
    }
    break;
  case 2:
    printf("[Demo] OK gesture (all slaves)\n");
    for (int i = 0; i < slave_count; ++i) {
      ec_set_ok_gesture(domain_data, ctx->slaves[i].off_out, 500);
    }
    break;
  case 3:
    printf("[Demo] Peace gesture (all slaves)\n");
    for (int i = 0; i < slave_count; ++i) {
      ec_set_peace_gesture(domain_data, ctx->slaves[i].off_out, 500);
    }
    break;
  }
}

#ifdef DEBUG_DATA
// Debug sent data on mode change (caller controls sampling)
static void debug_print_sent_data(ec_app_context_t *ctx, ec_master_state_t master_state, uint8_t *domain_data) {
  static uint16_t last_multi_mode[EC_MAX_SLAVES] = {0};
  static uint8_t last_single_mode[EC_MAX_SLAVES] = {0};
  if (!(master_state.al_states & 0x08)) return; // skip if no OP

  for (int i = 0; i < ctx->slave_count && i < EC_MAX_SLAVES; ++i) {
    unsigned int off_out = ctx->slaves[i].off_out;
    uint16_t multi_mode = *(uint16_t*)MULTI_FINGER_CTRL_MODE_ADDR(domain_data, off_out);
    if (multi_mode != 0 && multi_mode != last_multi_mode[i]) {
      if (multi_mode == Speed || multi_mode == Current) {
        int16_t first_val = *(int16_t*)MULTI_FINGER_PARAM1_ADDR(domain_data, off_out);
        DATA_DEBUG("[Slave %u] Multi-finger: mode=%u, first_val=%d (int16_t)",
                   ctx->slaves[i].slave_pos, multi_mode, first_val);
      } else {
        uint16_t first_val = *(uint16_t*)MULTI_FINGER_PARAM1_ADDR(domain_data, off_out);
        DATA_DEBUG("[Slave %u] Multi-finger: mode=%u, first_val=%u (uint16_t)",
                   ctx->slaves[i].slave_pos, multi_mode, first_val);
      }
      last_multi_mode[i] = multi_mode;
    }

    uint8_t single_mode = *(uint8_t*)SINGLE_FINGER_CTRL_MODE_ADDR(domain_data, off_out);
    if (single_mode != 0 && single_mode != last_single_mode[i]) {
      uint8_t single_id = *(uint8_t*)SINGLE_FINGER_JOINT_ID_ADDR(domain_data, off_out);
      uint16_t single_param1 = *(uint16_t*)SINGLE_FINGER_PARAM1_ADDR(domain_data, off_out);
      uint16_t single_param2 = *(uint16_t*)SINGLE_FINGER_PARAM2_ADDR(domain_data, off_out);
      DATA_DEBUG("[Slave %u] Single-finger: mode=%u, joint_id=%u, param1=%u, param2=%u",
                 ctx->slaves[i].slave_pos, single_mode, single_id, single_param1, single_param2);
      last_single_mode[i] = single_mode;
    }
  }
}
#endif

static int activate_master(ec_app_context_t *ctx) {
  EC_LOG_TIME_PREFIX(stdout);
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

    // Run one minimal cycle after activation so slaves see a PDO frame (some
    // devices require at least one PDO before entering OP; low-cost, keep enabled)
    ecrt_domain_queue(ctx->domain);
    ecrt_master_send(ctx->master);
    usleep(1000);  // Small delay to allow frame transmission
    ecrt_master_receive(ctx->master);
    ecrt_domain_process(ctx->domain);

    EC_LOG_TIME_PREFIX(stdout);
    printf("Initial RxPDO cycle completed (all slaves queued/sent/received once).\n");
  }

  return 0;
}

/****************************************************************************/
// Public API functions
/****************************************************************************/

int ec_app_init_pdo(ec_app_context_t *ctx, pdo_config_type_t pdo_type,
                    uint16_t slave_pos, int realtime_priority) {
  ec_slave_runtime_t slave_cfg = {
      .slave_pos = slave_pos,
      .pdo_type = pdo_type,  // Will be auto-detected if using master API, otherwise use provided
  };
  return ec_app_init_pdo_multi(ctx, &slave_cfg, 1, realtime_priority);
}

// Overloaded version without pdo_type - will auto-detect
int ec_app_init_pdo_auto(ec_app_context_t *ctx, uint16_t slave_pos, int realtime_priority) {
  ec_slave_runtime_t slave_cfg = {
      .slave_pos = slave_pos,
      .pdo_type = PDO_CONFIG_BASIC,  // Will be auto-detected during setup_pdo_registration
  };
  return ec_app_init_pdo_multi(ctx, &slave_cfg, 1, realtime_priority);
}

int ec_app_init_pdo_multi(ec_app_context_t *ctx,
                          const ec_slave_runtime_t *slave_cfgs,
                          int slave_count,
                          int realtime_priority) {
  if (slave_count <= 0 || slave_count > EC_MAX_SLAVES) {
    fprintf(stderr, "Invalid slave_count=%d (max=%d)\n", slave_count, EC_MAX_SLAVES);
    return -1;
  }

  ctx->app_type = EC_APP_PDO_ONLY;
  ctx->slave_count = slave_count;

  // Copy slave configs into context
  for (int i = 0; i < slave_count; ++i) {
    ctx->slaves[i] = slave_cfgs[i];
  }

  // Setup EtherCAT system (signal handling + realtime + master/domain)
  if (setup_ethercat(&ctx->master, &ctx->domain, realtime_priority) != 0) {
    return -1;
  }

  // Configure each slave
  for (int i = 0; i < slave_count; ++i) {
    ec_slave_runtime_t *rt = &ctx->slaves[i];

    // Setup slave configuration (creates slave_config, but doesn't configure PDOs yet)
    if (setup_slave_config(ctx, rt) != 0) {
      return -1;
    }

    // Register PDO entries for this slave (this will set rt->syncs)
    // If syncs is NULL, we rely on dynamic PDO reading or slave's existing PDO configuration
    if (setup_pdo_registration(ctx, rt) != 0) {
      return -1;
    }

    // Configure PDOs now that syncs is set (if available)
    if (rt->syncs) {
      EC_LOG_TIME_PREFIX(stdout);
      printf("Configuring PDOs for slave %u...\n", rt->slave_pos);
      if (ecrt_slave_config_pdos(ctx->slave_config, EC_END, rt->syncs)) {
        fprintf(stderr, "Failed to configure PDOs for slave %u.\n", rt->slave_pos);
        return -1;
      }
    }
  }

  // Activate master and get domain data
  if (activate_master(ctx) != 0) {
    return -1;
  }

  EC_LOG_TIME_PREFIX(stdout);
  printf("EtherCAT multi-slave PDO application initialized successfully. count=%d\n",
         slave_count);
  return 0;
}

int ec_app_init_sdo(ec_app_context_t *ctx, uint16_t slave_pos) {
  ctx->app_type = EC_APP_SDO_ONLY;

  // Setup EtherCAT system (no domain needed for SDO-only)
  if (setup_ethercat(&ctx->master, NULL, 0) != 0) {
    return -1;
  }

  // Set master for SDO operations
  ec_sdo_set_master(ctx->master);

  // Setup slave configuration (no PDO sync needed)
  ec_slave_runtime_t dummy_rt = {
      .slave_pos = slave_pos,
      .pdo_type = PDO_CONFIG_BASIC,  // Not used for SDO-only
  };
  if (setup_slave_config(ctx, &dummy_rt) != 0) {
    return -1;
  }

  EC_LOG_TIME_PREFIX(stdout);
  printf("EtherCAT SDO application initialized successfully.\n");

  return 0;
}

void ec_app_cleanup(ec_app_context_t *ctx) {
  if (ctx->master) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("Releasing EtherCAT master...\n");
    ecrt_release_master(ctx->master);
    ctx->master = NULL;
  }

  // Clear context
  ctx->domain = NULL;
  ctx->slave_config = NULL;
  ctx->domain_data = NULL;
}

ec_app_context_t *ec_app_get_context(void) { return &g_app_ctx; }

/****************************************************************************/
// Simplified main function helpers
/****************************************************************************/

int ec_app_main_pdo(pdo_config_type_t pdo_type, uint16_t slave_pos, ec_callback_t cyclic_func) {
  ec_app_context_t *ctx = &g_app_ctx;
  const int REALTIME_PRIORITY = 49;

  EC_LOG_TIME_PREFIX(stdout);
  printf("=== EtherCAT PDO Application Starting ===\n");

  // Initialize PDO application
  if (ec_app_init_pdo(ctx, pdo_type, slave_pos, REALTIME_PRIORITY) != 0) {
    ec_app_cleanup(ctx);
    return -1;
  }

  // Run cyclic function
  EC_LOG_TIME_PREFIX(stdout);
  printf("Starting cyclic function.\n");
  if (cyclic_func) {
    cyclic_func();
  }

  // Cleanup
  ec_app_cleanup(ctx);
  return 0;
}

int ec_app_main_sdo(uint16_t slave_pos, ec_callback_t demo_func) {
  ec_app_context_t *ctx = &g_app_ctx;

  EC_LOG_TIME_PREFIX(stdout);
  printf("=== EtherCAT SDO Application Starting ===\n");

  // Initialize SDO application
  if (ec_app_init_sdo(ctx, slave_pos) != 0) {
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

int ec_app_main_pdo_auto(uint16_t slave_pos, ec_callback_t cyclic_func) {
  ec_app_context_t *ctx = &g_app_ctx;
  const int REALTIME_PRIORITY = 49;

  EC_LOG_TIME_PREFIX(stdout);
  printf("=== EtherCAT PDO Application Starting (Auto-detect mode) ===\n");

  // Initialize PDO application with auto-detection
  if (ec_app_init_pdo_auto(ctx, slave_pos, REALTIME_PRIORITY) != 0) {
    ec_app_cleanup(ctx);
    return -1;
  }

  // Run cyclic function
  EC_LOG_TIME_PREFIX(stdout);
  printf("Starting cyclic function.\n");
  cyclic_func();

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
  int slave_count = ctx->slave_count;
  if (slave_count <= 0) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("No slaves configured, aborting cyclic task.\n");
    return;
  }

  // Local state variables
  ec_master_state_t master_state = {};
  ec_domain_state_t domain_state = {};
  unsigned int counter = 0;
  unsigned int sync_ref_counter = 0;
  const struct timespec cycletime = {0, PERIOD_NS};

  // Demo control state
  int demo_stage = 0;
  int demo_interval_counter = 0;
  bool demo_started = false;

  struct timespec wakeupTime, time;
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

  // Initialize master state before entering loop
  check_master_state(master, &master_state);
  for (int i = 0; i < slave_count; ++i) {
    int slave_op = check_slave_operational(master, ctx->slaves[i].slave_pos);
    EC_LOG_TIME_PREFIX(stdout);
    printf("Initial Master AL state: 0x%02X, Slave %u: %s\n",
           master_state.al_states, ctx->slaves[i].slave_pos,
           slave_op ? "OPERATIONAL" : "not operational");
  }

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

    // Update master state periodically (every second)
    if (counter) {
      counter--;
    } else {
      counter = FREQUENCY;
      check_master_state(master, &master_state);
      // Check each configured slave
      for (int i = 0; i < slave_count; ++i) {
        int slave_op = check_slave_operational(master, ctx->slaves[i].slave_pos);
        EC_LOG_TIME_PREFIX(stdout);
        printf("Master AL state: 0x%02X, Slave %u: %s\n",
               master_state.al_states, ctx->slaves[i].slave_pos,
               slave_op ? "OPERATIONAL" : "not operational");
      }
    }

    // Only perform read/write operations when the configured slave is in OPERATIONAL state
    // Master AL state can be 0x08 (all slaves OP) or 0x0C (some OP, some SAFEOP)
    // We check if OP state (0x08) is present, which means at least our slave is in OP
    // Note: Check master_state.al_states directly - it's updated by check_master_state()
    if (master_state.al_states & 0x08) {
      // Demo: switch gesture every 2 seconds
      // FREQUENCY=1000 cycles/sec, so 2000 cycles = 2 seconds
      const int DEMO_INTERVAL = FREQUENCY * 2;
      
      // First time entering OP: start demo immediately
      if (!demo_started) {
        printf("=== Starting Gesture Demo ===\n");
        demo_started = true;
        demo_interval_counter = DEMO_INTERVAL;  // Trigger first gesture immediately
      }
      
      // Check if it's time to switch gesture
      if (++demo_interval_counter >= DEMO_INTERVAL) {
        demo_interval_counter = 0;
        
        // Cycle through 4 gestures, applied to all slaves
        demo_apply_gesture_all(ctx, domain_data, demo_stage);
        
        // Move to next stage (0 -> 1 -> 2 -> 3 -> 0 -> ...)
        demo_stage = (demo_stage + 1) % 4;
      }

      // Read joint feedback every cycle (1kHz) for each slave
      for (int i = 0; i < slave_count; ++i) {
        joint_feedback_t feedback;
        ec_read_joint_feedback_data(domain_data, ctx->slaves[i].off_in, &feedback);
        ec_print_joint_feedback_data(&feedback); // Already rate-limited & timestamped
      }

      // Read additional feedback if callback provided
      if (read_feedback_func) {
        read_feedback_func();
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
    
    // Debug: Print sent data on change (sampled every 1000 cycles ~1s)
#ifdef DEBUG_DATA
    debug_print_sent_data(ctx, master_state, domain_data);
#endif
  }
}
