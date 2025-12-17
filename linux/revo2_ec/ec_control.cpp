/**
 * @file ec_control.cpp
 * @brief Implementation of EtherCAT low-level joint control functions
 */

#include "ec_control.h"
#include "ec_utils.h"
#include "ec_constants.h"
#include "ec_macros.h"
#include <ecrt.h>
#include <stdio.h>
#include <string.h>

/****************************************************************************/
// Single Joint Control Functions Implementation
/****************************************************************************/

void ec_set_single_joint_position_duration(uint8_t *domain_data,
                                           unsigned int off_out,
                                           finger_index_t joint_id,
                                           uint16_t position,
                                           uint16_t duration) {
  // Single-finger control starts after multi-finger PDO (26 bytes)
  EC_WRITE_U8(SINGLE_FINGER_CTRL_MODE_ADDR(domain_data, off_out), PositionWithDuration);
  EC_WRITE_U8(SINGLE_FINGER_JOINT_ID_ADDR(domain_data, off_out), (uint8_t)joint_id);
  EC_WRITE_U16(SINGLE_FINGER_PARAM1_ADDR(domain_data, off_out), position);
  EC_WRITE_U16(SINGLE_FINGER_PARAM2_ADDR(domain_data, off_out), duration);

  DATA_DEBUG("Single joint %s(%d): PositionWithDuration, pos=%d, duration=%d",
         get_joint_name(joint_id), joint_id, position, duration);
}

void ec_set_single_joint_position_speed(uint8_t *domain_data,
                                        unsigned int off_out,
                                        finger_index_t joint_id,
                                        uint16_t position, uint16_t speed) {
  EC_WRITE_U8(SINGLE_FINGER_CTRL_MODE_ADDR(domain_data, off_out), PositionWithSpeed);
  EC_WRITE_U8(SINGLE_FINGER_JOINT_ID_ADDR(domain_data, off_out), (uint8_t)joint_id);
  EC_WRITE_U16(SINGLE_FINGER_PARAM1_ADDR(domain_data, off_out), position);
  EC_WRITE_U16(SINGLE_FINGER_PARAM2_ADDR(domain_data, off_out), speed);

  DATA_DEBUG("Single joint %s(%d): PositionWithSpeed, pos=%d, speed=%d",
         get_joint_name(joint_id), joint_id, position, speed);
}

void ec_set_single_joint_speed(uint8_t *domain_data, unsigned int off_out,
                               finger_index_t joint_id, uint16_t speed) {
  EC_WRITE_U8(SINGLE_FINGER_CTRL_MODE_ADDR(domain_data, off_out), Speed);
  EC_WRITE_U8(SINGLE_FINGER_JOINT_ID_ADDR(domain_data, off_out), (uint8_t)joint_id);
  EC_WRITE_U16(SINGLE_FINGER_PARAM1_ADDR(domain_data, off_out), speed);
  EC_WRITE_U16(SINGLE_FINGER_PARAM2_ADDR(domain_data, off_out), 0); // param2 cleared

  DATA_DEBUG("Single joint %s(%d): Speed, speed=%d", get_joint_name(joint_id),
         joint_id, speed);
}

void ec_set_single_joint_current(uint8_t *domain_data, unsigned int off_out,
                                 finger_index_t joint_id, uint16_t current) {
  EC_WRITE_U8(SINGLE_FINGER_CTRL_MODE_ADDR(domain_data, off_out), Current);
  EC_WRITE_U8(SINGLE_FINGER_JOINT_ID_ADDR(domain_data, off_out), (uint8_t)joint_id);
  EC_WRITE_U16(SINGLE_FINGER_PARAM1_ADDR(domain_data, off_out), current);
  EC_WRITE_U16(SINGLE_FINGER_PARAM2_ADDR(domain_data, off_out), 0); // param2 cleared

  DATA_DEBUG("Single joint %s(%d): Current, current=%d", get_joint_name(joint_id),
         joint_id, current);
}

void ec_set_single_joint_pwm(uint8_t *domain_data, unsigned int off_out,
                             finger_index_t joint_id, uint16_t pwm_value) {
  EC_WRITE_U8(SINGLE_FINGER_CTRL_MODE_ADDR(domain_data, off_out), Pwm);
  EC_WRITE_U8(SINGLE_FINGER_JOINT_ID_ADDR(domain_data, off_out), (uint8_t)joint_id);
  EC_WRITE_U16(SINGLE_FINGER_PARAM1_ADDR(domain_data, off_out), pwm_value);
  EC_WRITE_U16(SINGLE_FINGER_PARAM2_ADDR(domain_data, off_out), 0); // param2 cleared

  DATA_DEBUG("Single joint %s(%d): PWM, pwm=%d", get_joint_name(joint_id),
         joint_id, pwm_value);
}

/****************************************************************************/
// All Joints Control Functions Implementation
/****************************************************************************/

void ec_write_all_joints_position_duration(uint8_t *domain_data,
                                           unsigned int off_out,
                                           const uint16_t positions[6],
                                           const uint16_t durations[6]) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(MULTI_FINGER_CTRL_MODE_ADDR(domain_data, off_out), PositionWithDuration);
  memcpy(MULTI_FINGER_PARAM1_ADDR(domain_data, off_out), positions, 12); // positions (6 x uint16)
  memcpy(MULTI_FINGER_PARAM2_ADDR(domain_data, off_out), durations, 12); // durations (6 x uint16)
  
  DATA_DEBUG("All joints: PositionWithDuration, positions=[%u,%u,%u,%u,%u,%u], durations=[%u,%u,%u,%u,%u,%u]",
             positions[0], positions[1], positions[2], positions[3], positions[4], positions[5],
             durations[0], durations[1], durations[2], durations[3], durations[4], durations[5]);
}

void ec_write_all_joints_position_speed(uint8_t *domain_data,
                                        unsigned int off_out,
                                        const uint16_t positions[6],
                                        const uint16_t speeds[6]) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(MULTI_FINGER_CTRL_MODE_ADDR(domain_data, off_out), PositionWithSpeed);
  memcpy(MULTI_FINGER_PARAM1_ADDR(domain_data, off_out), positions, 12); // positions (6 x uint16)
  memcpy(MULTI_FINGER_PARAM2_ADDR(domain_data, off_out), speeds, 12);   // speeds (6 x uint16)
  
  DATA_DEBUG("All joints: PositionWithSpeed, positions=[%u,%u,%u,%u,%u,%u], speeds=[%u,%u,%u,%u,%u,%u]",
             positions[0], positions[1], positions[2], positions[3], positions[4], positions[5],
             speeds[0], speeds[1], speeds[2], speeds[3], speeds[4], speeds[5]);
}

void ec_write_all_joints_speed(uint8_t *domain_data, unsigned int off_out,
                               const int16_t speeds[6]) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(MULTI_FINGER_CTRL_MODE_ADDR(domain_data, off_out), Speed);
  memcpy(MULTI_FINGER_PARAM1_ADDR(domain_data, off_out), speeds, 12); // speeds (6 x int16)
  memset(MULTI_FINGER_PARAM2_ADDR(domain_data, off_out), 0, 12);     // param2 cleared
  
  DATA_DEBUG("All joints: Speed, speeds=[%d,%d,%d,%d,%d,%d]",
             speeds[0], speeds[1], speeds[2], speeds[3], speeds[4], speeds[5]);
}

void ec_write_all_joints_current(uint8_t *domain_data, unsigned int off_out,
                                 const int16_t currents[6]) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(MULTI_FINGER_CTRL_MODE_ADDR(domain_data, off_out), Current);
  memcpy(MULTI_FINGER_PARAM1_ADDR(domain_data, off_out), currents, 12); // currents (6 x int16)
  memset(MULTI_FINGER_PARAM2_ADDR(domain_data, off_out), 0, 12);      // param2 cleared
  
  DATA_DEBUG("All joints: Current, currents=[%d,%d,%d,%d,%d,%d]",
             currents[0], currents[1], currents[2], currents[3], currents[4], currents[5]);
}

void ec_write_all_joints_pwm(uint8_t *domain_data, unsigned int off_out,
                             const uint16_t pwm_values[6]) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(MULTI_FINGER_CTRL_MODE_ADDR(domain_data, off_out), Pwm);
  memcpy(MULTI_FINGER_PARAM1_ADDR(domain_data, off_out), pwm_values, 12); // pwm_values (6 x uint16)
  memset(MULTI_FINGER_PARAM2_ADDR(domain_data, off_out), 0, 12);         // param2 cleared
  
  DATA_DEBUG("All joints: PWM, pwm_values=[%u,%u,%u,%u,%u,%u]",
             pwm_values[0], pwm_values[1], pwm_values[2], pwm_values[3], pwm_values[4], pwm_values[5]);
}
