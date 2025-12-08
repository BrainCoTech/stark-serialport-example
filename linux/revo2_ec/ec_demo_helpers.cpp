/**
 * @file ec_demo_helpers.cpp
 * @brief Implementation of helper functions for PDO demo applications
 */

#include "ec_demo_helpers.h"
#include "ec_control.h"
#include "ec_utils.h"
#include <cstdio>
#include <cstring>
#include <ecrt.h>

/****************************************************************************/
// Predefined gesture positions (from ec_motor)
/****************************************************************************/

// Fist gesture positions (all fingers closed)
static const uint16_t FIST_POSITIONS[6] = {400, 400, 800, 800, 800, 800};

// Open hand gesture positions (all fingers open)
static const uint16_t OPEN_HAND_POSITIONS[6] = {400, 400, 100, 100, 100, 100};

// OK gesture positions (thumb and index contact, others open)
static const uint16_t OK_POSITIONS[6] = {400, 400, 700, 100, 100, 100};

// Peace gesture positions (index and middle up, others closed)
static const uint16_t PEACE_POSITIONS[6] = {400, 400, 100, 100, 800, 800};

// Pointing gesture positions (index extended, others closed)
static const uint16_t POINTING_POSITIONS[6] = {400, 400, 100, 800, 800, 800};

/****************************************************************************/
// Gesture Functions Implementation (from ec_motor)
/****************************************************************************/

void ec_set_fist_gesture(uint8_t *domain_data, unsigned int off_out,
                         uint16_t duration) {
  printf("Setting fist gesture (duration: %u ms)...\n", duration);
  ec_write_all_joints_position_duration(domain_data, off_out, FIST_POSITIONS,
                                        duration);
}

void ec_set_open_hand_gesture(uint8_t *domain_data, unsigned int off_out,
                              uint16_t duration) {
  printf("Setting open hand gesture (duration: %u ms)...\n", duration);
  ec_write_all_joints_position_duration(domain_data, off_out,
                                        OPEN_HAND_POSITIONS, duration);
}

void ec_set_ok_gesture(uint8_t *domain_data, unsigned int off_out,
                       uint16_t duration) {
  printf("Setting OK gesture (duration: %u ms)...\n", duration);
  ec_write_all_joints_position_duration(domain_data, off_out, OK_POSITIONS,
                                        duration);
}

void ec_set_peace_gesture(uint8_t *domain_data, unsigned int off_out,
                          uint16_t duration) {
  printf("Setting peace gesture (duration: %u ms)...\n", duration);
  ec_write_all_joints_position_duration(domain_data, off_out, PEACE_POSITIONS,
                                        duration);
}

void ec_set_pointing_gesture(uint8_t *domain_data, unsigned int off_out,
                             uint16_t duration) {
  printf("Setting pointing gesture (duration: %u ms)...\n", duration);
  ec_write_all_joints_position_duration(domain_data, off_out,
                                        POINTING_POSITIONS, duration);
}

/****************************************************************************/
// Trajectory Control Functions Implementation (from ec_trajectory)
/****************************************************************************/

void ec_init_trajectory(TrajectoryControl *traj_ctrl) {
  memset(traj_ctrl, 0, sizeof(TrajectoryControl));
  traj_ctrl->trajectory_active = false;
  printf("Trajectory control initialized.\n");
}

void ec_set_all_joints_position_duration(TrajectoryControl *traj_ctrl,
                                         uint16_t positions[6],
                                         uint16_t durations[6]) {
  // Copy positions and durations
  for (int i = 0; i < 6; i++) {
    traj_ctrl->positions[i] = positions[i];
    traj_ctrl->durations[i] = durations[i];
  }

  // Record start time
  traj_ctrl->start_time_ms = get_mills();
  traj_ctrl->trajectory_active = true;

  printf("Trajectory started: ");
  for (int i = 0; i < 6; i++) {
    printf("J%d(pos=%d,dur=%d) ", i, positions[i], durations[i]);
  }
  printf("\n");
}

void ec_update_trajectory(TrajectoryControl *traj_ctrl, uint8_t *domain_data,
                          unsigned int off_out) {
  if (!traj_ctrl->trajectory_active) {
    return;
  }

  int current_time_ms = get_mills();
  int elapsed_time = current_time_ms - traj_ctrl->start_time_ms;

  bool any_joint_active = false;

  // Update each joint's trajectory
  for (int i = 0; i < 6; i++) {
    if (elapsed_time < traj_ctrl->durations[i]) {
      // Calculate interpolated position
      float progress = (float)elapsed_time / (float)traj_ctrl->durations[i];
      uint16_t current_pos = (uint16_t)(traj_ctrl->positions[i] * progress);

      // Update single joint position
      ec_set_single_joint_position_duration(
          domain_data, off_out, (finger_index_t)i, current_pos, 100);

      any_joint_active = true;

      // Optional: print trajectory progress (reduce frequency to avoid spam)
      static int debug_counter = 0;
      if (debug_counter++ % 1000 == 0) {
        printf("Joint %s: progress=%.1f%%, pos=%d/%d, time=%d/%d ms\n",
               get_joint_name((finger_index_t)i), progress * 100, current_pos,
               traj_ctrl->positions[i], elapsed_time, traj_ctrl->durations[i]);
      }
    }
  }

  // Stop trajectory if all joints have finished
  if (!any_joint_active) {
    traj_ctrl->trajectory_active = false;
    printf("Trajectory completed after %d ms\n", elapsed_time);
  }
}

bool ec_is_trajectory_active(const TrajectoryControl *traj_ctrl) {
  return traj_ctrl->trajectory_active;
}

void ec_stop_trajectory(TrajectoryControl *traj_ctrl) {
  traj_ctrl->trajectory_active = false;
  printf("Trajectory stopped\n");
}

float ec_get_trajectory_progress(const TrajectoryControl *traj_ctrl) {
  if (!traj_ctrl->trajectory_active) {
    return 1.0f; // Completed
  }

  int current_time_ms = get_mills();
  int elapsed_time = current_time_ms - traj_ctrl->start_time_ms;

  // Find the maximum duration among all joints
  int max_duration = 0;
  for (int i = 0; i < 6; i++) {
    if (traj_ctrl->durations[i] > max_duration) {
      max_duration = traj_ctrl->durations[i];
    }
  }

  if (max_duration == 0) {
    return 1.0f;
  }

  float progress = (float)elapsed_time / (float)max_duration;
  return progress > 1.0f ? 1.0f : progress;
}
