/**
 * @file ec_demo_helpers.h
 * @brief Helper functions for PDO demo applications
 *
 * This module provides wrapper functions to simplify PDO demo code by
 * abstracting common patterns. These functions use global variables
 * (domain_data, off_out, off_in) from the demo context.
 *
 * USAGE EXAMPLE:
 * ```c
 * // 1. Set demo context once at initialization
 * ec_demo_set_context(domain_data, off_out, off_in);
 *
 * // 2. Use simplified wrapper functions
 * ec_demo_set_fist_gesture(300);
 * ec_demo_set_single_joint_position_duration(THUMB_FLEX, 500, 200);
 * ec_demo_set_thumb_control(400, 300, 250);
 * ```
 *
 * DESIGN PHILOSOPHY:
 * - These helpers are OPTIONAL - existing demo code remains valid
 * - Demo files can show full implementation for educational purposes
 * - Or use helpers for more concise code
 * - Production code should use ec_control.h functions directly with explicit
 * parameters
 *
 * Note: This is for demo/example code only. Production code should
 * pass parameters explicitly or use a context structure.
 */

#ifndef EC_DEMO_HELPERS_H
#define EC_DEMO_HELPERS_H

#include "ec_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Gesture Functions (from ec_motor)
/****************************************************************************/

/**
 * @brief Set fist gesture (all fingers closed)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param duration Duration in milliseconds
 */
void ec_set_fist_gesture(uint8_t *domain_data, unsigned int off_out,
                         uint16_t duration);

/**
 * @brief Set open hand gesture (all fingers open)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param duration Duration in milliseconds
 */
void ec_set_open_hand_gesture(uint8_t *domain_data, unsigned int off_out,
                              uint16_t duration);

/**
 * @brief Set OK gesture (thumb and index finger touch, others open)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param duration Duration in milliseconds
 */
void ec_set_ok_gesture(uint8_t *domain_data, unsigned int off_out,
                       uint16_t duration);

/**
 * @brief Set peace gesture (index and middle fingers up, others closed)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param duration Duration in milliseconds
 */
void ec_set_peace_gesture(uint8_t *domain_data, unsigned int off_out,
                          uint16_t duration);

/**
 * @brief Set pointing gesture (index finger extended, others closed)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param duration Duration in milliseconds
 */
void ec_set_pointing_gesture(uint8_t *domain_data, unsigned int off_out,
                             uint16_t duration);

/****************************************************************************/
// Trajectory Control Functions (from ec_trajectory)
/****************************************************************************/

/**
 * @brief Initialize trajectory control structure
 * @param traj_ctrl Pointer to trajectory control structure
 */
void ec_init_trajectory(TrajectoryControl *traj_ctrl);

/**
 * @brief Set all joints position and duration control (start trajectory)
 * @param traj_ctrl Pointer to trajectory control structure
 * @param positions Array of target positions for 6 joints
 * @param durations Array of durations for 6 joints
 */
void ec_set_all_joints_position_duration(TrajectoryControl *traj_ctrl,
                                         uint16_t positions[6],
                                         uint16_t durations[6]);

/**
 * @brief Update trajectory control (call this periodically)
 * @param traj_ctrl Pointer to trajectory control structure
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 */
void ec_update_trajectory(TrajectoryControl *traj_ctrl, uint8_t *domain_data,
                          unsigned int off_out);

/**
 * @brief Check if trajectory is active
 * @param traj_ctrl Pointer to trajectory control structure
 * @return true if trajectory is active, false otherwise
 */
bool ec_is_trajectory_active(const TrajectoryControl *traj_ctrl);

/**
 * @brief Stop current trajectory
 * @param traj_ctrl Pointer to trajectory control structure
 */
void ec_stop_trajectory(TrajectoryControl *traj_ctrl);

/**
 * @brief Get trajectory progress (0.0 to 1.0)
 * @param traj_ctrl Pointer to trajectory control structure
 * @return Progress value between 0.0 and 1.0
 */
float ec_get_trajectory_progress(const TrajectoryControl *traj_ctrl);

#ifdef __cplusplus
}
#endif

#endif // EC_DEMO_HELPERS_H
