/**
 * @file ec_control.h
 * @brief EtherCAT low-level joint control functions
 * 
 * This header provides low-level joint control functions for individual
 * joint control in different modes (position, speed, current, PWM).
 */

#ifndef EC_CONTROL_H
#define EC_CONTROL_H

#include <stdint.h>
#include "ec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Single Joint Control Functions
/****************************************************************************/

/**
 * @brief Set single joint position and duration control
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param joint_id Joint index
 * @param position Target position
 * @param duration Duration in milliseconds
 */
void ec_set_single_joint_position_duration(uint8_t *domain_data, unsigned int off_out,
                                           finger_index_t joint_id, uint16_t position, uint16_t duration);

/**
 * @brief Set single joint position and speed control
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param joint_id Joint index
 * @param position Target position
 * @param speed Target speed
 */
void ec_set_single_joint_position_speed(uint8_t *domain_data, unsigned int off_out,
                                        finger_index_t joint_id, uint16_t position, uint16_t speed);

/**
 * @brief Set single joint speed control
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param joint_id Joint index
 * @param speed Target speed
 */
void ec_set_single_joint_speed(uint8_t *domain_data, unsigned int off_out,
                               finger_index_t joint_id, uint16_t speed);

/**
 * @brief Set single joint current control
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param joint_id Joint index
 * @param current Target current
 */
void ec_set_single_joint_current(uint8_t *domain_data, unsigned int off_out,
                                 finger_index_t joint_id, uint16_t current);

/**
 * @brief Set single joint PWM control
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param joint_id Joint index
 * @param pwm_value PWM value
 */
void ec_set_single_joint_pwm(uint8_t *domain_data, unsigned int off_out,
                             finger_index_t joint_id, uint16_t pwm_value);

/****************************************************************************/
// All Joints Control Functions
/****************************************************************************/

/**
 * @brief Set position and duration control mode for all joints (writes to PDO)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param positions Array of target positions for 6 joints
 * @param duration Duration in milliseconds (same for all joints)
 */
void ec_write_all_joints_position_duration(uint8_t *domain_data, unsigned int off_out,
                                           const uint16_t positions[6], uint16_t duration);

/**
 * @brief Set position and speed control mode for all joints (writes to PDO)
 * @param domain_data Pointer to domain data
 * @param off_out Output offset in domain data
 * @param positions Array of target positions for 6 joints
 * @param speeds Array of target speeds for 6 joints
 */
void ec_write_all_joints_position_speed(uint8_t *domain_data, unsigned int off_out,
                                        const uint16_t positions[6], const uint16_t speeds[6]);

#ifdef __cplusplus
}
#endif

#endif // EC_CONTROL_H
