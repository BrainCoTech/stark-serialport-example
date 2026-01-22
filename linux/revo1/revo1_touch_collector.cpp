#include "stark-sdk.h"
#include "../common/stark_common.h"
#include "../common/trajectory_control.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

/**
 * Revo1 Touch (Capacitive) - High-Performance Data Collector Example
 * 
 * This example demonstrates high-performance data collection for capacitive touch sensors:
 * - Motor status data collection at 100Hz (Linux) or 20Hz (Windows/macOS)
 * - Capacitive touch sensor data collection at 100Hz (Linux) or 10Hz (Windows/macOS)
 * - Efficient data transfer via shared buffer API
 * - Lock-free buffer design for minimal overhead
 * - Optional trajectory control for testing motor response
 * - Suitable for real-time tactile feedback and control
 * 
 * Hardware Support:
 * - ✅ Revo1 Touch (Capacitive Sensor)
 * 
 * Configuration:
 *   #define ENABLE_CONTROL 1    // Enable trajectory control (0=disabled, 1=enabled)
 *   #define TRAJ_LEN 20         // Number of trajectory points
 *   #define CTRL_FREQUENCY 100  // Control frequency in Hz
 * 
 * Compile:
 *   make revo1_touch_collector
 * 
 * Run:
 *   ./revo1_touch_collector
 */

// ============================================================================
// Configuration
// ============================================================================

#define ENABLE_CONTROL 1      // Enable trajectory control (0=disabled, 1=enabled)
#define TRAJ_LEN 20           // Number of trajectory points
#define CTRL_FREQUENCY 100    // Control frequency in Hz

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
    
    // Check if device has touch sensor
    bool has_touch = (info->hardware_type == STARK_HARDWARE_TYPE_REVO1_TOUCH);
    
    if (!has_touch) {
      fprintf(stderr, "[ERROR] Device does not have touch sensor!\n");
      free_device_info(info);
      return -1;
    }
    
    printf("[INFO] Device has capacitive touch sensor\n");
    free_device_info(info);
  }

  // Enable touch sensor
  printf("[INFO] Enabling touch sensor...\n");
  stark_enable_touch_sensor(handle, slave_id, 0x1F);  // Enable all 5 fingers
  usleep(1000 * 1000); // Wait 1 second

  // ========================================================================
  // Create shared buffers
  // ========================================================================
  
  printf("[INFO] Creating shared buffers...\n");
  
  // Motor status buffer (max 1000 items)
  auto motor_buffer = motor_buffer_new(1000);
  if (motor_buffer == NULL) {
    fprintf(stderr, "[ERROR] Failed to create motor buffer.\n");
    return -1;
  }
  
  // Touch status buffer (max 1000 items per finger)
  auto touch_buffer = touch_buffer_new(1000);
  if (touch_buffer == NULL) {
    fprintf(stderr, "[ERROR] Failed to create touch buffer.\n");
    motor_buffer_free(motor_buffer);
    return -1;
  }

  // ========================================================================
  // Create and start data collector - Capacitive mode
  // ========================================================================
  
  printf("[INFO] Creating data collector (Capacitive mode)...\n");
  
  // Platform-dependent frequencies
  // Linux: 50Hz motor, 10Hz touch (Revo1 is slower)
  // Platform-specific frequencies
  #ifdef __linux__
    const uint32_t motor_frequency = 100;  // Linux: 100Hz (Revo1)
    const uint32_t touch_frequency = 100;  // Touch: 100Hz
  #else
    const uint32_t motor_frequency = 20;   // Windows/macOS: 20Hz (conservative)
    const uint32_t touch_frequency = 10;   // Touch: 10Hz
  #endif
  
  printf("[INFO] Motor frequency: %dHz, Touch frequency: %dHz\n", 
         motor_frequency, touch_frequency);
  
  auto collector = data_collector_new_capacitive(
    handle,                    // Device context
    motor_buffer,              // Motor buffer
    touch_buffer,              // Touch buffer
    slave_id,                  // Slave ID
    motor_frequency,           // Motor frequency (platform-dependent)
    touch_frequency,           // Touch frequency (platform-dependent)
    1                          // Enable stats
  );
  
  if (collector == NULL) {
    fprintf(stderr, "[ERROR] Failed to create data collector.\n");
    touch_buffer_free(touch_buffer);
    motor_buffer_free(motor_buffer);
    return -1;
  }

  // Start data collection
  printf("[INFO] Starting data collector...\n");
  if (data_collector_start(collector) != 0) {
    fprintf(stderr, "[ERROR] Failed to start data collector.\n");
    data_collector_free(collector);
    touch_buffer_free(touch_buffer);
    motor_buffer_free(motor_buffer);
    return -1;
  }

  printf("[INFO] Data collector started successfully!\n");
  printf("[INFO] Capacitive mode: Motor %dHz + Touch %dHz\n", motor_frequency, touch_frequency);
  printf("[INFO] Please touch the sensor to see touch data...\n");

  // ========================================================================
  // Optional: Start trajectory control
  // ========================================================================
  
  #if ENABLE_CONTROL
  printf("[INFO] Initializing trajectory control...\n");
  
  // Generate cosine trajectory (0-1000 range)
  // Note: Thumb trajectory will be auto-clamped to max 500 if needed
  uint16_t *trajectory = init_cosine_trajectory(TRAJ_LEN, 0, 1000);
  if (trajectory == NULL) {
    fprintf(stderr, "[ERROR] Failed to create trajectory.\n");
    data_collector_stop(collector);
    data_collector_free(collector);
    touch_buffer_free(touch_buffer);
    motor_buffer_free(motor_buffer);
    return -1;
  }
  
  // Initialize trajectory control for Ring finger
  TrajectoryControl traj_ctrl;
  trajectory_control_init(
    &traj_ctrl, handle, slave_id,
    STARK_FINGER_ID_RING,  // Control Ring finger
    trajectory, TRAJ_LEN, CTRL_FREQUENCY
  );
  
  // Start trajectory control
  if (trajectory_control_start(&traj_ctrl) != 0) {
    fprintf(stderr, "[ERROR] Failed to start trajectory control.\n");
    free(trajectory);
    data_collector_stop(collector);
    data_collector_free(collector);
    touch_buffer_free(touch_buffer);
    motor_buffer_free(motor_buffer);
    return -1;
  }
  
  printf("[INFO] Trajectory control started (Index finger, %dHz)\n", CTRL_FREQUENCY);
  #endif
  
  printf("[INFO] Press Ctrl+C to stop...\n\n");

  // ========================================================================
  // Main loop: Read data from buffers
  // ========================================================================
  
  // Allocate arrays for reading data
  const size_t MAX_MOTOR_DATA = 1000;
  const size_t MAX_TOUCH_DATA = 1000;
  
  MotorStatusData motor_data[MAX_MOTOR_DATA];
  TouchFingerItem touch_data[5][MAX_TOUCH_DATA];
  size_t motor_count = 0;
  size_t touch_counts[5] = {0};
  
  const char* finger_names[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
  int loop_count = 0;
  
  while (keep_running) {
    usleep(100 * 1000); // Sleep 100ms
    loop_count++;
    
    // Read motor status every 10 loops (1 second)
    if (loop_count % 10 == 0) {
      size_t buffer_len = motor_buffer_len(motor_buffer);
      if (buffer_len > 0) {
        motor_buffer_pop_all(motor_buffer, motor_data, &motor_count);
        
        if (motor_count > 0) {
          auto latest = &motor_data[motor_count - 1];
          
          #if ENABLE_CONTROL
          size_t traj_idx = trajectory_control_get_index(&traj_ctrl);
          printf("[Motor] Count=%zu, Traj=%zu/%d, Latest: Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                 motor_count, traj_idx, TRAJ_LEN,
                 latest->positions[0], latest->positions[1], latest->positions[2],
                 latest->positions[3], latest->positions[4], latest->positions[5]);
          #else
          printf("[Motor] Count=%zu, Latest: Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                 motor_count,
                 latest->positions[0], latest->positions[1], latest->positions[2],
                 latest->positions[3], latest->positions[4], latest->positions[5]);
          #endif
        }
      }
    }
    
    // Read touch data
    bool has_touch_data = false;
    for (int finger = 0; finger < 5; finger++) {
      int result = touch_buffer_pop_finger(
        touch_buffer, finger, 
        touch_data[finger], &touch_counts[finger]
      );
      
      if (result == 0 && touch_counts[finger] > 0) {
        has_touch_data = true;
      }
    }
    
    if (has_touch_data) {
      printf("[Touch] Data counts: [%zu,%zu,%zu,%zu,%zu]\n",
             touch_counts[0], touch_counts[1], touch_counts[2],
             touch_counts[3], touch_counts[4]);
      
      // Print each finger's latest data
      for (int finger = 0; finger < 5; finger++) {
        if (touch_counts[finger] > 0) {
          auto latest_item = &touch_data[finger][touch_counts[finger] - 1];
          
          // Check if there is contact
          bool is_contact = latest_item->normal_force1 > 50;
          
          if (is_contact) {
            printf("  %s: normal_force=%d, tangential_force=%d, proximity=%d [CONTACT]\n",
                   finger_names[finger],
                   latest_item->normal_force1,
                   latest_item->tangential_force1,
                   latest_item->self_proximity1);
          }
        }
      }
      printf("\n");
    }
  }

  // ========================================================================
  // Cleanup
  // ========================================================================
  
  #if ENABLE_CONTROL
  printf("\n[INFO] Stopping trajectory control...\n");
  trajectory_control_stop(&traj_ctrl);
  free(trajectory);
  #endif
  
  printf("\n[INFO] Stopping data collector...\n");
  data_collector_stop(collector);
  
  printf("[INFO] Cleaning up...\n");
  data_collector_free(collector);
  touch_buffer_free(touch_buffer);
  motor_buffer_free(motor_buffer);
  
  printf("[INFO] Done!\n");
  
  return 0;
}
