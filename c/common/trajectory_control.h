#ifndef TRAJECTORY_CONTROL_H
#define TRAJECTORY_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include "stark-sdk.h"
#include "platform_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Trajectory Control for Modbus Devices
 * 
 * This module provides trajectory control functionality for testing motor response
 * while simultaneously collecting high-frequency data.
 */

// Trajectory control structure
typedef struct {
  uint16_t *trajectory;      // Trajectory points array
  size_t trajectory_len;     // Number of trajectory points
  size_t current_index;      // Current trajectory index
  bool active;               // Control active flag
  pthread_t thread;          // Control thread
  
  // Control parameters
  DeviceHandler *handle;     // Device handle
  uint8_t slave_id;          // Slave ID
  StarkFingerId finger_id;   // Target finger ID
  uint32_t interval_us;      // Control interval in microseconds
} TrajectoryControl;

/**
 * Initialize cosine trajectory
 * 
 * Generate a complete cosine wave trajectory for position control testing.
 * 
 * Note: Thumb finger trajectory will be automatically clamped to max 500 by
 * trajectory_control_init() because Thumb has 2 joints controlled together.
 * 
 * @param traj_len Number of trajectory points (default: 20)
 * @param min_val Minimum value (0-1000, representing 0-100%)
 * @param max_val Maximum value (0-1000, representing 0-100%)
 * @return Pointer to trajectory array (caller must free)
 */
static inline uint16_t* init_cosine_trajectory(size_t traj_len, uint16_t min_val, uint16_t max_val) {
  uint16_t *trajectory = (uint16_t*)malloc(traj_len * sizeof(uint16_t));
  if (!trajectory) return NULL;
  
  double amplitude = (max_val - min_val) / 2.0;
  double offset = (max_val + min_val) / 2.0;
  
  for (size_t i = 0; i < traj_len; i++) {
    // Generate cosine wave: offset + amplitude * cos(2πi/N)
    double value = offset + amplitude * cos(2.0 * M_PI * i / traj_len);
    trajectory[i] = (uint16_t)value;
  }
  
  return trajectory;
}

/**
 * Trajectory control thread function
 */
static void* trajectory_control_thread(void *arg) {
  TrajectoryControl *ctrl = (TrajectoryControl*)arg;
  
  printf("[Trajectory] Control thread started (interval: %u us)\n", ctrl->interval_us);
  
  while (ctrl->active) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    // Get current target position
    uint16_t target = ctrl->trajectory[ctrl->current_index];
    
    // Send position control command
    stark_set_finger_position(ctrl->handle, ctrl->slave_id, ctrl->finger_id, target);
    // printf("[Trajectory] Sent position control command, index: %zu, target: %u\n", ctrl->current_index, target);
    
    // Move to next trajectory point (circular)
    ctrl->current_index = (ctrl->current_index + 1) % ctrl->trajectory_len;
    
    // Calculate elapsed time
    clock_gettime(CLOCK_MONOTONIC, &end);
    long elapsed_us = (end.tv_sec - start.tv_sec) * 1000000 + 
                      (end.tv_nsec - start.tv_nsec) / 1000;
    
    // Sleep for remaining time
    long remaining_us = ctrl->interval_us - elapsed_us;
    if (remaining_us > 0) {
      usleep(remaining_us);
    }
  }
  
  printf("[Trajectory] Control thread stopped\n");
  return NULL;
}

/**
 * Initialize trajectory control
 * 
 * @param ctrl Trajectory control structure
 * @param handle Device handle
 * @param slave_id Slave ID
 * @param finger_id Target finger ID
 * @param trajectory Trajectory points array
 * @param trajectory_len Number of trajectory points
 * @param frequency_hz Control frequency in Hz (default: 100)
 */
static inline void trajectory_control_init(
  TrajectoryControl *ctrl,
  DeviceHandler *handle,
  uint8_t slave_id,
  StarkFingerId finger_id,
  uint16_t *trajectory,
  size_t trajectory_len,
  uint32_t frequency_hz
) {
  ctrl->trajectory = trajectory;
  ctrl->trajectory_len = trajectory_len;
  ctrl->current_index = 0;
  ctrl->active = false;
  ctrl->handle = handle;
  ctrl->slave_id = slave_id;
  ctrl->finger_id = finger_id;
  ctrl->interval_us = 1000000 / frequency_hz;  // Convert Hz to microseconds
  
  // IMPORTANT: Thumb has 2 joints controlled together, limit trajectory to max 500
  if (finger_id == STARK_FINGER_ID_THUMB) {
    for (size_t i = 0; i < trajectory_len; i++) {
      if (ctrl->trajectory[i] > 500) {
        ctrl->trajectory[i] = 500;
      }
    }
    printf("[Trajectory] WARNING: Thumb finger detected, trajectory clamped to max 500\n");
  }
}

/**
 * Start trajectory control
 * 
 * @param ctrl Trajectory control structure
 * @return 0 on success, -1 on failure
 */
static inline int trajectory_control_start(TrajectoryControl *ctrl) {
  if (ctrl->active) {
    fprintf(stderr, "[Trajectory] Already running\n");
    return -1;
  }
  
  ctrl->active = true;
  ctrl->current_index = 0;
  
  if (pthread_create(&ctrl->thread, NULL, trajectory_control_thread, ctrl) != 0) {
    fprintf(stderr, "[Trajectory] Failed to create control thread\n");
    ctrl->active = false;
    return -1;
  }
  
  return 0;
}

/**
 * Stop trajectory control
 * 
 * @param ctrl Trajectory control structure
 */
static inline void trajectory_control_stop(TrajectoryControl *ctrl) {
  if (!ctrl->active) return;
  
  printf("[Trajectory] Stopping control...\n");
  ctrl->active = false;
  pthread_join(ctrl->thread, NULL);
}

/**
 * Get current trajectory index
 * 
 * @param ctrl Trajectory control structure
 * @return Current trajectory index
 */
static inline size_t trajectory_control_get_index(const TrajectoryControl *ctrl) {
  return ctrl->current_index;
}

/**
 * Check if trajectory control is active
 * 
 * @param ctrl Trajectory control structure
 * @return true if active, false otherwise
 */
static inline bool trajectory_control_is_active(const TrajectoryControl *ctrl) {
  return ctrl->active;
}

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_CONTROL_H
