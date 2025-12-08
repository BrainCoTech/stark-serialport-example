/**
 * @file ec_feedback.h
 * @brief EtherCAT feedback data reading functions
 * 
 * This header provides functions to read and process feedback data from EtherCAT slaves,
 * including joint positions, speeds, currents, status, and touch sensor data.
 */

#ifndef EC_FEEDBACK_H
#define EC_FEEDBACK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ec_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Joint Feedback Data Structure
/****************************************************************************/

typedef struct {
  uint16_t positions[6];  // Joint positions (6 x uint16)
  uint16_t speeds[6];     // Joint speeds (6 x uint16)
  int16_t currents[6];    // Joint currents (6 x int16)
  uint16_t status[6];     // Joint status (6 x uint16)
} joint_feedback_t;

/****************************************************************************/
// Touch Feedback Data Structure
/****************************************************************************/

typedef struct {
  uint16_t force_normal[5];  // Normal forces for 5 fingers
  uint16_t force_index[5];   // Index forces for 5 fingers
  uint16_t force_middle[5];  // Middle forces for 5 fingers
  uint16_t force_ring[5];    // Ring forces for 5 fingers
  uint16_t force_pinky[5];   // Pinky forces for 5 fingers
  uint16_t touch_status[5];  // Touch status for 5 fingers
} touch_feedback_t;

/****************************************************************************/
// Pressure Touch Feedback Data Structure (Extended)
/****************************************************************************/

typedef struct {
  uint16_t thumb_force[9];   // Thumb force data (9 points)
  uint16_t index_force[9];   // Index force data (9 points)
  uint16_t middle_force[9];  // Middle force data (9 points)
  uint16_t ring_force[9];    // Ring force data (9 points)
  uint16_t pinky_force[9];   // Pinky force data (9 points)
  uint16_t palm_force[9];    // Palm force data (9 points)
  uint16_t force_total[18];  // Total force data (18 points)
} pressure_touch_feedback_t;

/****************************************************************************/
// Feedback Reading Functions
/****************************************************************************/

/**
 * @brief Read joint feedback data (positions, speeds, currents, status)
 * @param domain_data Pointer to domain data
 * @param off_in Input offset in domain data
 * @param feedback Pointer to store joint feedback data
 */
void ec_read_joint_feedback_data(uint8_t *domain_data, unsigned int off_in, 
                                joint_feedback_t *feedback);

/**
 * @brief Read basic touch feedback data
 * @param domain_data Pointer to domain data
 * @param off_in Input offset in domain data
 * @param feedback Pointer to store touch feedback data
 */
void ec_read_touch_feedback_data(uint8_t *domain_data, unsigned int off_in,
                                touch_feedback_t *feedback);

/**
 * @brief Read pressure touch feedback data (extended touch sensing)
 * @param domain_data Pointer to domain data
 * @param off_in Input offset in domain data
 * @param feedback Pointer to store pressure touch feedback data
 */
void ec_read_pressure_touch_feedback_data(uint8_t *domain_data, unsigned int off_in,
                                         pressure_touch_feedback_t *feedback);

/****************************************************************************/
// Feedback Display Functions
/****************************************************************************/

/**
 * @brief Print joint feedback data in a formatted way
 * @param feedback Pointer to joint feedback data
 */
void ec_print_joint_feedback_data(const joint_feedback_t *feedback);

/**
 * @brief Print touch feedback data in a formatted way
 * @param feedback Pointer to touch feedback data
 */
void ec_print_touch_feedback_data(const touch_feedback_t *feedback);

/**
 * @brief Print pressure touch feedback data in a formatted way
 * @param feedback Pointer to pressure touch feedback data
 */
void ec_print_pressure_touch_feedback_data(const pressure_touch_feedback_t *feedback);

#ifdef __cplusplus
}
#endif

#endif // EC_FEEDBACK_H
