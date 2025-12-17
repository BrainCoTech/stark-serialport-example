/**
 * @file ec_demo_helpers.cpp
 * @brief Implementation of helper functions for PDO demo applications
 */

#include "ec_demo_helpers.h"
#include "ec_control.h"
#include "ec_utils.h"
#include "ec_macros.h"
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
  DEMO_DEBUG("Setting fist gesture (duration: %u ms)", duration);
  uint16_t durations[6] = {duration, duration, duration, duration, duration, duration};
  ec_write_all_joints_position_duration(domain_data, off_out, FIST_POSITIONS,
                                        durations);
}

void ec_set_open_hand_gesture(uint8_t *domain_data, unsigned int off_out,
                              uint16_t duration) {
  DEMO_DEBUG("Setting open hand gesture (duration: %u ms)", duration);
  uint16_t durations[6] = {duration, duration, duration, duration, duration, duration};
  ec_write_all_joints_position_duration(domain_data, off_out,
                                        OPEN_HAND_POSITIONS, durations);
}

void ec_set_ok_gesture(uint8_t *domain_data, unsigned int off_out,
                       uint16_t duration) {
  DEMO_DEBUG("Setting OK gesture (duration: %u ms)", duration);
  uint16_t durations[6] = {duration, duration, duration, duration, duration, duration};
  ec_write_all_joints_position_duration(domain_data, off_out, OK_POSITIONS,
                                        durations);
}

void ec_set_peace_gesture(uint8_t *domain_data, unsigned int off_out,
                          uint16_t duration) {
  DEMO_DEBUG("Setting peace gesture (duration: %u ms)", duration);
  uint16_t durations[6] = {duration, duration, duration, duration, duration, duration};
  ec_write_all_joints_position_duration(domain_data, off_out, PEACE_POSITIONS,
                                        durations);
}

void ec_set_pointing_gesture(uint8_t *domain_data, unsigned int off_out,
                             uint16_t duration) {
  DEMO_DEBUG("Setting pointing gesture (duration: %u ms)", duration);
  uint16_t durations[6] = {duration, duration, duration, duration, duration, duration};
  ec_write_all_joints_position_duration(domain_data, off_out,
                                        POINTING_POSITIONS, durations);
}

/****************************************************************************/
// Trajectory Control Functions Implementation (from ec_trajectory)
/****************************************************************************/

void ec_init_trajectory(TrajectoryControl *traj_ctrl) {
  memset(traj_ctrl, 0, sizeof(TrajectoryControl));
  traj_ctrl->trajectory_active = false;
  DATA_DEBUG("Trajectory control initialized");
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

  DATA_DEBUG("Trajectory started: J0(pos=%d,dur=%d) J1(pos=%d,dur=%d) J2(pos=%d,dur=%d) J3(pos=%d,dur=%d) J4(pos=%d,dur=%d) J5(pos=%d,dur=%d)",
             positions[0], durations[0], positions[1], durations[1], 
             positions[2], durations[2], positions[3], durations[3],
             positions[4], durations[4], positions[5], durations[5]);
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
        DATA_DEBUG("Joint %s: progress=%.1f%%, pos=%d/%d, time=%d/%d ms",
               get_joint_name((finger_index_t)i), progress * 100, current_pos,
               traj_ctrl->positions[i], elapsed_time, traj_ctrl->durations[i]);
      }
    }
  }

  // Stop trajectory if all joints have finished
  if (!any_joint_active) {
    traj_ctrl->trajectory_active = false;
    DATA_DEBUG("Trajectory completed after %d ms", elapsed_time);
  }
}

bool ec_is_trajectory_active(const TrajectoryControl *traj_ctrl) {
  return traj_ctrl->trajectory_active;
}

void ec_stop_trajectory(TrajectoryControl *traj_ctrl) {
  traj_ctrl->trajectory_active = false;
  DATA_DEBUG("Trajectory stopped");
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
