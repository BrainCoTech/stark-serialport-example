/**
 * @file ec_control.cpp
 * @brief Implementation of EtherCAT low-level joint control functions
 */

#include "ec_control.h"
#include "ec_utils.h"
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
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, PositionWithDuration);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, position);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, duration);

  printf("Single joint %s(%d): PositionWithDuration, pos=%d, duration=%d\n",
         get_joint_name(joint_id), joint_id, position, duration);
}

void ec_set_single_joint_position_speed(uint8_t *domain_data,
                                        unsigned int off_out,
                                        finger_index_t joint_id,
                                        uint16_t position, uint16_t speed) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, PositionWithSpeed);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, position);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, speed);

  printf("Single joint %s(%d): PositionWithSpeed, pos=%d, speed=%d\n",
         get_joint_name(joint_id), joint_id, position, speed);
}

void ec_set_single_joint_speed(uint8_t *domain_data, unsigned int off_out,
                               finger_index_t joint_id, uint16_t speed) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, Speed);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, speed);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, 0); // param2 cleared

  printf("Single joint %s(%d): Speed, speed=%d\n", get_joint_name(joint_id),
         joint_id, speed);
}

void ec_set_single_joint_current(uint8_t *domain_data, unsigned int off_out,
                                 finger_index_t joint_id, uint16_t current) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, Current);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, current);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, 0); // param2 cleared

  printf("Single joint %s(%d): Current, current=%d\n", get_joint_name(joint_id),
         joint_id, current);
}

void ec_set_single_joint_pwm(uint8_t *domain_data, unsigned int off_out,
                             finger_index_t joint_id, uint16_t pwm_value) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, Pwm);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, pwm_value);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, 0); // param2 cleared

  printf("Single joint %s(%d): PWM, pwm=%d\n", get_joint_name(joint_id),
         joint_id, pwm_value);
}

/****************************************************************************/
// All Joints Control Functions Implementation
/****************************************************************************/

void ec_write_all_joints_position_duration(uint8_t *domain_data,
                                           unsigned int off_out,
                                           const uint16_t positions[6],
                                           uint16_t duration) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(domain_data + off_out, PositionWithDuration);
  memcpy(domain_data + off_out + 2, positions, 12); // positions (6 x uint16)

  // Set same duration for all joints
  uint16_t durations[6];
  for (int i = 0; i < 6; i++) {
    durations[i] = duration;
  }
  memcpy(domain_data + off_out + 14, durations, 12); // durations (6 x uint16)
}

void ec_write_all_joints_position_speed(uint8_t *domain_data,
                                        unsigned int off_out,
                                        const uint16_t positions[6],
                                        const uint16_t speeds[6]) {
  // Write control data to EtherCAT domain
  EC_WRITE_U16(domain_data + off_out, PositionWithSpeed);
  memcpy(domain_data + off_out + 2, positions, 12); // positions (6 x uint16)
  memcpy(domain_data + off_out + 14, speeds, 12);   // speeds (6 x uint16)
}
