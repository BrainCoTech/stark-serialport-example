#include "stark-sdk.h"
#include "stark_common.h"
#include "../common/trajectory_control.h"
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

/**
 * Revo2 Modulus (Pressure) - High-Performance Data Collector Example
 * 
 * This example demonstrates high-performance data collection for pressure touch sensors.
 * Features:
 * - Efficient data transfer via shared buffer API
 * - Lock-free buffer design for minimal overhead
 * - Optional trajectory control for testing motor response
 * - Supports three collection modes:
 *   - summary: Fast pressure monitoring (100Hz)
 *   - detailed: Detailed sensor point analysis (10Hz)
 *   - dual: Summary + Detailed simultaneously (100Hz + 10Hz)
 * 
 * Hardware Support:
 * - ✅ Revo2 Touch (Pressure Sensor)
 * 
 * Configuration:
 *   #define ENABLE_CONTROL 1    // Enable trajectory control (0=disabled, 1=enabled)
 *   #define TRAJ_LEN 20         // Number of trajectory points
 *   #define CTRL_FREQUENCY 100  // Control frequency in Hz
 * 
 * Compile:
 *   make revo2_touch_pressure_collector
 * 
 * Run:
 *   # Summary mode (default)
 *   ./revo2_touch_pressure_collector
 *   
 *   # Detailed mode
 *   ./revo2_touch_pressure_collector detailed
 *   
 *   # Dual mode (Summary + Detailed)
 *   ./revo2_touch_pressure_collector dual
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
  // Parse command line arguments
  const char* mode = "summary";  // Default mode
  if (argc > 1) {
    mode = argv[1];
    if (strcmp(mode, "summary") != 0 && 
        strcmp(mode, "detailed") != 0 && 
        strcmp(mode, "dual") != 0) {
      fprintf(stderr, "[ERROR] Invalid mode: %s\n", mode);
      fprintf(stderr, "Usage: %s [summary|detailed|dual]\n", argv[0]);
      return -1;
    }
  }
  
  printf("[INFO] Collection mode: %s\n", mode);

  // Setup signal handlers
  setup_signal_handlers();
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_INFO); // Initialize configuration
  auto cfg = auto_detect_modbus_revo2(NULL, true); // Replace with actual serial port name; passing NULL will try auto-detection
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
    bool has_touch = (info->hardware_type == STARK_HARDWARE_TYPE_REVO1_TOUCH ||
                      info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH);
    
    if (!has_touch) {
      fprintf(stderr, "[ERROR] Device does not have touch sensor!\n");
      free_device_info(info);
      return -1;
    }
    
    printf("[INFO] Device has pressure touch sensor (Modulus)\n");
    free_device_info(info);
  }

  // Enable touch sensor
  printf("[INFO] Enabling pressure sensor...\n");
  stark_enable_touch_sensor(handle, slave_id, 0x3F);  // Enable all 5 fingers + palm
  usleep(1000 * 1000); // Wait 1 second

  // ========================================================================
  // Create shared buffers based on mode
  // ========================================================================
  
  printf("[INFO] Creating shared buffers for %s mode...\n", mode);
  
  // Motor status buffer (max 1000 items)
  auto motor_buffer = motor_buffer_new(1000);
  if (motor_buffer == NULL) {
    fprintf(stderr, "[ERROR] Failed to create motor buffer.\n");
    return -1;
  }
  
  // Create pressure buffers based on mode
  CPressureSummaryBuffer* pressure_summary_buffer = NULL;
  CPressureDetailedBuffer* pressure_detailed_buffer = NULL;
  
  if (strcmp(mode, "summary") == 0 || strcmp(mode, "dual") == 0) {
    pressure_summary_buffer = pressure_summary_buffer_new(1000);
    if (pressure_summary_buffer == NULL) {
      fprintf(stderr, "[ERROR] Failed to create pressure summary buffer.\n");
      motor_buffer_free(motor_buffer);
      return -1;
    }
  }
  
  if (strcmp(mode, "detailed") == 0 || strcmp(mode, "dual") == 0) {
    pressure_detailed_buffer = pressure_detailed_buffer_new(1000);
    if (pressure_detailed_buffer == NULL) {
      fprintf(stderr, "[ERROR] Failed to create pressure detailed buffer.\n");
      if (pressure_summary_buffer) pressure_summary_buffer_free(pressure_summary_buffer);
      motor_buffer_free(motor_buffer);
      return -1;
    }
  }

  // ========================================================================
  // Create and start data collector based on mode
  // ========================================================================
  
  printf("[INFO] Creating data collector (%s mode)...\n", mode);
  
  // Platform-dependent frequencies
  // Linux: 100Hz motor, 100Hz summary, 10Hz detailed (high-performance)
  // Other platforms: 20Hz motor, 20Hz summary, 10Hz detailed (conservative)
  #ifdef __linux__
    const uint32_t motor_frequency = 100;    // Linux: 100Hz
    const uint32_t summary_frequency = 100;  // Linux: 100Hz
    const uint32_t detailed_frequency = 10;  // Detailed: 10Hz (same for all)
  #else
    const uint32_t motor_frequency = 20;     // Windows/macOS: 20Hz
    const uint32_t summary_frequency = 20;   // Windows/macOS: 20Hz
    const uint32_t detailed_frequency = 10;  // Detailed: 10Hz (same for all)
  #endif
  
  printf("[INFO] Motor: %dHz, Summary: %dHz, Detailed: %dHz\n", 
         motor_frequency, summary_frequency, detailed_frequency);
  
  CDataCollector* collector = NULL;
  
  if (strcmp(mode, "summary") == 0) {
    collector = data_collector_new_pressure_summary(
      handle, motor_buffer, pressure_summary_buffer,
      slave_id, motor_frequency, summary_frequency, 1
    );
  } else if (strcmp(mode, "detailed") == 0) {
    collector = data_collector_new_pressure_detailed(
      handle, motor_buffer, pressure_detailed_buffer,
      slave_id, motor_frequency, detailed_frequency, 1
    );
  } else {  // dual
    collector = data_collector_new_pressure_hybrid(
      handle, motor_buffer, pressure_summary_buffer, pressure_detailed_buffer,
      slave_id, motor_frequency, summary_frequency, detailed_frequency, 1
    );
  }
  
  if (collector == NULL) {
    fprintf(stderr, "[ERROR] Failed to create data collector.\n");
    if (pressure_detailed_buffer) pressure_detailed_buffer_free(pressure_detailed_buffer);
    if (pressure_summary_buffer) pressure_summary_buffer_free(pressure_summary_buffer);
    motor_buffer_free(motor_buffer);
    return -1;
  }

  // Start data collection
  printf("[INFO] Starting data collector...\n");
  if (data_collector_start(collector) != 0) {
    fprintf(stderr, "[ERROR] Failed to start data collector.\n");
    data_collector_free(collector);
    if (pressure_detailed_buffer) pressure_detailed_buffer_free(pressure_detailed_buffer);
    if (pressure_summary_buffer) pressure_summary_buffer_free(pressure_summary_buffer);
    motor_buffer_free(motor_buffer);
    return -1;
  }

  printf("[INFO] Data collector started successfully!\n");
  if (strcmp(mode, "dual") == 0) {
    printf("[INFO] Dual mode: Summary 100Hz + Detailed 10Hz\n");
  }
  printf("[INFO] Please touch the sensor to see pressure data...\n");

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
    if (pressure_detailed_buffer) pressure_detailed_buffer_free(pressure_detailed_buffer);
    if (pressure_summary_buffer) pressure_summary_buffer_free(pressure_summary_buffer);
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
    if (pressure_detailed_buffer) pressure_detailed_buffer_free(pressure_detailed_buffer);
    if (pressure_summary_buffer) pressure_summary_buffer_free(pressure_summary_buffer);
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
  const size_t MAX_PRESSURE_DATA = 1000;
  
  MotorStatusData motor_data[MAX_MOTOR_DATA];
  uint16_t summary_data[6][MAX_PRESSURE_DATA];
  PressureDetailedItem detailed_data[6][MAX_PRESSURE_DATA];
  size_t motor_count = 0;
  size_t summary_counts[6] = {0};
  size_t detailed_counts[6] = {0};
  
  const char* part_names[] = {"Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"};
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
    
    // Read pressure summary data
    if (pressure_summary_buffer) {
      bool has_summary_data = false;
      for (int part = 0; part < 6; part++) {
        int result = pressure_summary_buffer_pop_finger(
          pressure_summary_buffer, part, 
          summary_data[part], &summary_counts[part]
        );
        
        if (result == 0 && summary_counts[part] > 0) {
          has_summary_data = true;
        }
      }
      
      int print_interval = (strcmp(mode, "dual") == 0) ? 10 : 1;
      if (has_summary_data && loop_count % print_interval == 0) {
        printf("[Summary] Data counts: [%zu,%zu,%zu,%zu,%zu,%zu]\n",
               summary_counts[0], summary_counts[1], summary_counts[2],
               summary_counts[3], summary_counts[4], summary_counts[5]);
        
        // Print each part's latest pressure value
        for (int part = 0; part < 6; part++) {
          if (summary_counts[part] > 0) {
            uint16_t latest_pressure = summary_data[part][summary_counts[part] - 1];
            
            // Check if there is contact (threshold: 50mN)
            bool is_contact = latest_pressure > 50;
            
            if (is_contact) {
              printf("  %s: %hu mN [CONTACT]\n", part_names[part], latest_pressure);
            }
          }
        }
      }
    }
    
    // Read pressure detailed data
    if (pressure_detailed_buffer) {
      bool has_detailed_data = false;
      for (int part = 0; part < 6; part++) {
        int result = pressure_detailed_buffer_pop_finger(
          pressure_detailed_buffer, part, 
          detailed_data[part], &detailed_counts[part]
        );
        
        if (result == 0 && detailed_counts[part] > 0) {
          has_detailed_data = true;
        }
      }
      
      if (has_detailed_data) {
        printf("[Detailed] Data counts: [%zu,%zu,%zu,%zu,%zu,%zu]\n",
               detailed_counts[0], detailed_counts[1], detailed_counts[2],
               detailed_counts[3], detailed_counts[4], detailed_counts[5]);
        
        // Print each part's detailed data
        for (int part = 0; part < 6; part++) {
          if (detailed_counts[part] > 0) {
            auto latest_item = &detailed_data[part][detailed_counts[part] - 1];
            uint8_t sensor_count = latest_item->sensor_count;
            
            // Calculate statistics
            uint32_t total_pressure = 0;
            uint16_t max_pressure = 0;
            int max_idx = 0;
            
            for (int i = 0; i < sensor_count; i++) {
              uint16_t pressure = latest_item->sensors[i];
              total_pressure += pressure;
              if (pressure > max_pressure) {
                max_pressure = pressure;
                max_idx = i;
              }
            }
            
            float avg_pressure = sensor_count > 0 ? 
              (float)total_pressure / sensor_count : 0.0f;
            
            // Check if there is contact (threshold: 50mN)
            bool is_contact = max_pressure > 50;
            
            if (is_contact) {
              printf("  %s: sensors=%u, total=%u mN, max=%u mN (at #%d), "
                     "avg=%.1f mN [CONTACT]\n",
                     part_names[part], sensor_count, total_pressure,
                     max_pressure, max_idx, avg_pressure);
            }
          }
        }
        printf("\n");
      }
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
  if (pressure_detailed_buffer) pressure_detailed_buffer_free(pressure_detailed_buffer);
  if (pressure_summary_buffer) pressure_summary_buffer_free(pressure_summary_buffer);
  motor_buffer_free(motor_buffer);
  
  printf("[INFO] Done!\n");
  
  return 0;
}
