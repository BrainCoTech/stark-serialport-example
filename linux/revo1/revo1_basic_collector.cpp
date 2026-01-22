#include "stark-sdk.h"
#include "../common/stark_common.h"
#include "../common/trajectory_control.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

/**
 * Revo1 Basic Version - High-Performance Data Collector Example with Trajectory Control
 * 
 * This example demonstrates high-performance motor data collection using shared buffers
 * while simultaneously running trajectory control:
 * - Motor status data collection at 50Hz (Linux) / 20Hz (other platforms)
 * - Trajectory control at 100Hz
 * - Efficient data transfer via shared buffer API
 * - Lock-free buffer design for minimal overhead
 * - Observe motor response to trajectory commands in real-time
 * 
 * Hardware Support:
 * - ✅ Revo1 Basic (基础版)
 * - ✅ Revo1 Touch (电容式触觉)
 * 
 * Compile:
 *   make revo1_basic_collector
 * 
 * Run:
 *   ./revo1_basic_collector
 */

// Trajectory control parameters
#define TRAJ_LEN 20           // Number of trajectory points
#define CTRL_FREQUENCY 100    // Control frequency: 100Hz
#define ENABLE_CONTROL 1      // Enable/disable trajectory control (1=enable, 0=disable)

// Global variable for signal handling
static volatile int keep_running = 1;

void signal_handler(int signum) {
  printf("\n[INFO] Received signal %d, stopping...\n", signum);
  keep_running = 0;
}

int main(int argc, char const *argv[]) {
  // Setup signal handlers
  setup_signal_handlers();
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Initialize SDK
  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO);
  
  // Auto-detect device
  auto cfg = auto_detect_modbus_revo1("/dev/ttyUSB0", true);
  if (cfg == NULL) {
    fprintf(stderr, "[ERROR] Failed to auto-detect Modbus device.\n");
    return -1;
  }
  
  uint8_t slave_id = cfg->slave_id;
  printf("[INFO] Detected device on %s, slave_id=%d, baudrate=%d\n", 
         cfg->port_name, slave_id, cfg->baudrate);
  
  // Open device
  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  free_device_config(cfg);
  
  if (handle == NULL) {
    fprintf(stderr, "[ERROR] Failed to open device.\n");
    return -1;
  }

  // Get device info
  auto info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("[INFO] Device: SKU=%d, SN=%s, FW=%s\n",
           (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    free_device_info(info);
  }

  // ========================================================================
  // Create shared buffer
  // ========================================================================
  
  printf("[INFO] Creating shared buffer...\n");
  
  // Motor status buffer (max 1000 items)
  auto motor_buffer = motor_buffer_new(1000);
  if (motor_buffer == NULL) {
    fprintf(stderr, "[ERROR] Failed to create motor buffer.\n");
    return -1;
  }

  // ========================================================================
  // Create and start data collector - Basic mode
  // ========================================================================
  
  printf("[INFO] Creating data collector (Basic mode)...\n");
  
  // Platform-dependent frequencies
  // Linux: 50Hz (Revo1 is slower than Revo2)
  // Other platforms: 20Hz (conservative for compatibility)
  #ifdef __linux__
    const uint32_t motor_frequency = 50;   // Linux: 50Hz (Revo1)
  #else
    const uint32_t motor_frequency = 20;   // Windows/macOS: 20Hz
  #endif
  
  printf("[INFO] Motor frequency: %dHz\n", motor_frequency);
  
  auto collector = data_collector_new_basic(
    handle,                    // Device context
    motor_buffer,              // Motor buffer
    slave_id,                  // Slave ID
    motor_frequency,           // Motor frequency (platform-dependent)
    1                          // Enable stats
  );
  
  if (collector == NULL) {
    fprintf(stderr, "[ERROR] Failed to create data collector.\n");
    motor_buffer_free(motor_buffer);
    return -1;
  }

  // Start data collection
  printf("[INFO] Starting data collector...\n");
  if (data_collector_start(collector) != 0) {
    fprintf(stderr, "[ERROR] Failed to start data collector.\n");
    data_collector_free(collector);
    motor_buffer_free(motor_buffer);
    return -1;
  }

  printf("[INFO] Data collector started successfully!\n");
  printf("[INFO] Basic mode: Motor data collection at %dHz\n", motor_frequency);

  // ========================================================================
  // Start trajectory control (optional)
  // ========================================================================
  
  TrajectoryControl traj_ctrl;
  uint16_t *trajectory = NULL;
  
  #if ENABLE_CONTROL
    printf("[INFO] Initializing trajectory control...\n");
    
    // Generate cosine trajectory (0-1000 representing 0-100%)
    // Note: Thumb trajectory will be auto-clamped to max 500 if needed
    trajectory = init_cosine_trajectory(TRAJ_LEN, 0, 1000);
    if (trajectory == NULL) {
      fprintf(stderr, "[ERROR] Failed to create trajectory.\n");
      data_collector_free(collector);
      motor_buffer_free(motor_buffer);
      return -1;
    }
    
    // Initialize trajectory control for ring finger
    trajectory_control_init(
      &traj_ctrl,
      handle,
      slave_id,
      STARK_FINGER_ID_RING,  // Control ring finger
      trajectory,
      TRAJ_LEN,
      CTRL_FREQUENCY
    );
    
    // Start trajectory control
    if (trajectory_control_start(&traj_ctrl) != 0) {
      fprintf(stderr, "[ERROR] Failed to start trajectory control.\n");
      free(trajectory);
      data_collector_free(collector);
      motor_buffer_free(motor_buffer);
      return -1;
    }
    
    printf("[INFO] Trajectory control started (Ring finger, %dHz)\n", CTRL_FREQUENCY);
  #endif
  
  printf("[INFO] Press Ctrl+C to stop...\n\n");

  // ========================================================================
  // Main loop: Read data from buffer
  // ========================================================================
  
  // Allocate array for reading data
  const size_t MAX_MOTOR_DATA = 1000;
  MotorStatusData motor_data[MAX_MOTOR_DATA];
  size_t motor_count = 0;
  
  int loop_count = 0;
  
  while (keep_running) {
    usleep(100 * 1000); // Sleep 100ms
    loop_count++;
    
    // Read motor status every loop
    size_t buffer_len = motor_buffer_len(motor_buffer);
    if (buffer_len > 0 && loop_count % 2 == 0) {  // Print every 2 times
      motor_buffer_pop_all(motor_buffer, motor_data, &motor_count);
      
      if (motor_count > 0) {
        auto latest = &motor_data[motor_count - 1];
        
        #if ENABLE_CONTROL
          size_t traj_index = trajectory_control_get_index(&traj_ctrl);
          printf("[Motor] Count=%zu [Traj Index: %zu], Latest: Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                 motor_count, traj_index,
                 latest->positions[0], latest->positions[1], latest->positions[2],
                 latest->positions[3], latest->positions[4], latest->positions[5]);
        #else
          printf("[Motor] Count=%zu, Latest: Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                 motor_count,
                 latest->positions[0], latest->positions[1], latest->positions[2],
                 latest->positions[3], latest->positions[4], latest->positions[5]);
        #endif
        
        printf("        Speeds=[%hd,%hd,%hd,%hd,%hd,%hd]\n",
               latest->speeds[0], latest->speeds[1], latest->speeds[2],
               latest->speeds[3], latest->speeds[4], latest->speeds[5]);
        printf("        Currents=[%hd,%hd,%hd,%hd,%hd,%hd]\n\n",
               latest->currents[0], latest->currents[1], latest->currents[2],
               latest->currents[3], latest->currents[4], latest->currents[5]);
      }
    }
  }

  // ========================================================================
  // Cleanup
  // ========================================================================
  
  #if ENABLE_CONTROL
    printf("\n[INFO] Stopping trajectory control...\n");
    trajectory_control_stop(&traj_ctrl);
    if (trajectory) {
      free(trajectory);
    }
  #endif
  
  printf("\n[INFO] Stopping data collector...\n");
  data_collector_stop(collector);
  
  printf("[INFO] Cleaning up...\n");
  data_collector_free(collector);
  motor_buffer_free(motor_buffer);
  
  printf("[INFO] Done!\n");
  
  return 0;
}
