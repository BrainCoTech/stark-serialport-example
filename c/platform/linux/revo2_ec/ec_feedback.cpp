/**
 * @file ec_feedback.cpp
 * @brief EtherCAT feedback data reading functions implementation
 */

#include "ec_common.h"
#include <stdio.h>
#include <string.h>

/****************************************************************************/
// Feedback Reading Functions Implementation
/****************************************************************************/

void ec_read_joint_feedback_data(uint8_t *domain_data, unsigned int off_in,
                                 joint_feedback_t *feedback) {
  if (!domain_data || !feedback) {
    printf("Error: Invalid parameters for joint feedback reading\n");
    return;
  }

  // Read joint positions (12 bytes = 6 x uint16)
  memcpy(feedback->positions, domain_data + off_in, 12);

  // Read joint speeds (12 bytes = 6 x uint16)
  memcpy(feedback->speeds, domain_data + off_in + 12, 12);

  // Read joint currents (12 bytes = 6 x int16)
  memcpy(feedback->currents, domain_data + off_in + 24, 12);

  // Read joint status (12 bytes = 6 x uint16)
  memcpy(feedback->status, domain_data + off_in + 36, 12);
}

void ec_read_touch_feedback_data(uint8_t *domain_data, unsigned int off_in,
                                 touch_feedback_t *feedback) {
  if (!domain_data || !feedback) {
    printf("Error: Invalid parameters for touch feedback reading\n");
    return;
  }

  // Touch data starts after joint data (48 bytes offset)
  size_t touch_offset = off_in + 48;

  // Read normal forces (10 bytes = 5 x uint16)
  memcpy(feedback->force_normal, domain_data + touch_offset, 10);

  // Read index forces (10 bytes = 5 x uint16)
  memcpy(feedback->force_index, domain_data + touch_offset + 10, 10);

  // Read middle forces (10 bytes = 5 x uint16)
  memcpy(feedback->force_middle, domain_data + touch_offset + 20, 10);

  // Read ring forces (10 bytes = 5 x uint16)
  memcpy(feedback->force_ring, domain_data + touch_offset + 30, 10);

  // Read pinky forces (10 bytes = 5 x uint16)
  memcpy(feedback->force_pinky, domain_data + touch_offset + 40, 10);

  // Read touch status (10 bytes = 5 x uint16)
  memcpy(feedback->touch_status, domain_data + touch_offset + 50, 10);
}

void ec_read_pressure_touch_feedback_data(uint8_t *domain_data,
                                          unsigned int off_in,
                                          pressure_touch_feedback_t *feedback) {
  if (!domain_data || !feedback) {
    printf("Error: Invalid parameters for pressure touch feedback reading\n");
    return;
  }

  // Pressure touch data starts after joint data (48 bytes offset)
  size_t touch_offset = off_in + 48;

  // Define finger data sizes and offsets
  const size_t FINGER_DATA_SIZE = 18; // 9 x uint16 = 18 bytes
  const size_t PALM_DATA_SIZE = 18;   // 9 x uint16 = 18 bytes
  const size_t TOTAL_DATA_SIZE = 36;  // 18 x uint16 = 36 bytes

  size_t finger_offsets[6] = {
      0,                    // Thumb
      FINGER_DATA_SIZE,     // Index
      FINGER_DATA_SIZE * 2, // Middle
      FINGER_DATA_SIZE * 3, // Ring
      FINGER_DATA_SIZE * 4, // Pinky
      FINGER_DATA_SIZE * 5  // Palm
  };

  // Check data boundaries before reading
  size_t max_offset = touch_offset + FINGER_DATA_SIZE * 6 + TOTAL_DATA_SIZE;
  if (max_offset > off_in + 512) {
    printf("Warning: Data boundary check failed for pressure touch feedback\n");
    return;
  }

  // Read finger force data
  memcpy(feedback->thumb_force, domain_data + touch_offset + finger_offsets[0],
         FINGER_DATA_SIZE);
  memcpy(feedback->index_force, domain_data + touch_offset + finger_offsets[1],
         FINGER_DATA_SIZE);
  memcpy(feedback->middle_force, domain_data + touch_offset + finger_offsets[2],
         FINGER_DATA_SIZE);
  memcpy(feedback->ring_force, domain_data + touch_offset + finger_offsets[3],
         FINGER_DATA_SIZE);
  memcpy(feedback->pinky_force, domain_data + touch_offset + finger_offsets[4],
         FINGER_DATA_SIZE);
  memcpy(feedback->palm_force, domain_data + touch_offset + finger_offsets[5],
         PALM_DATA_SIZE);

  // Read total force data
  size_t total_offset = touch_offset + FINGER_DATA_SIZE * 6;
  memcpy(feedback->force_total, domain_data + total_offset, TOTAL_DATA_SIZE);
}

/****************************************************************************/
// Feedback Display Functions Implementation
// Note: These functions check DEBUG_FEEDBACK and use FEEDBACK_PRINT_INTERVAL
// for rate limiting. Call them every cycle - printing is controlled internally.
/****************************************************************************/

void ec_print_joint_feedback_data(const joint_feedback_t *feedback) {
#ifndef DEBUG_FEEDBACK
  (void)feedback;  // Suppress unused parameter warning
  return;
#else
  if (!feedback) {
    ERROR_PRINT("Invalid feedback data pointer");
    return;
  }

  // Rate limiting: only print at FEEDBACK_PRINT_INTERVAL
  static int counter = 0;
  if (++counter < FEEDBACK_PRINT_INTERVAL) {
    return;
  }
  counter = 0;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tm_now;
  localtime_r(&ts.tv_sec, &tm_now);
  int ms = ts.tv_nsec / 1000000;
  FEEDBACK_DEBUG("=== Joint Feedback Data (t=%02d:%02d:%02d.%03d) ===",
                 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, ms);

  const char *joint_names[6] = {"Thumb_Flex", "Thumb_Abduct", "Index",
                                "Middle",     "Ring",         "Pinky"};

  for (int i = 0; i < 6; i++) {
    FEEDBACK_DEBUG("  %s: pos=%u, spd=%u, cur=%d, status=0x%04X", joint_names[i],
                   feedback->positions[i], feedback->speeds[i], feedback->currents[i],
                   feedback->status[i]);
  }
  FEEDBACK_DEBUG("===========================");
#endif
}

void ec_print_touch_feedback_data(const touch_feedback_t *feedback) {
#ifndef DEBUG_FEEDBACK
  (void)feedback;
  return;
#else
  if (!feedback) {
    ERROR_PRINT("Invalid feedback data pointer");
    return;
  }

  // Rate limiting: only print at FEEDBACK_PRINT_INTERVAL
  static int counter = 0;
  if (++counter < FEEDBACK_PRINT_INTERVAL) {
    return;
  }
  counter = 0;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tm_now;
  localtime_r(&ts.tv_sec, &tm_now);
  int ms = ts.tv_nsec / 1000000;
  FEEDBACK_DEBUG("=== Touch Feedback Data (t=%02d:%02d:%02d.%03d) ===",
                 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, ms);

  const char *finger_names[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

  for (int i = 0; i < 5; i++) {
    FEEDBACK_DEBUG(
        "  %s: normal=%u, index=%u, middle=%u, ring=%u, pinky=%u, status=%u",
        finger_names[i], feedback->force_normal[i], feedback->force_index[i],
        feedback->force_middle[i], feedback->force_ring[i],
        feedback->force_pinky[i], feedback->touch_status[i]);
  }
  FEEDBACK_DEBUG("===========================");
#endif
}

void ec_print_pressure_touch_feedback_data(
    const pressure_touch_feedback_t *feedback) {
#ifndef DEBUG_FEEDBACK
  (void)feedback;
  return;
#else
  if (!feedback) {
    ERROR_PRINT("Invalid feedback data pointer");
    return;
  }

  // Rate limiting: only print at FEEDBACK_PRINT_INTERVAL
  static int counter = 0;
  if (++counter < FEEDBACK_PRINT_INTERVAL) {
    return;
  }
  counter = 0;

  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  struct tm tm_now;
  localtime_r(&ts.tv_sec, &tm_now);
  int ms = ts.tv_nsec / 1000000;
  FEEDBACK_DEBUG("=== Pressure Touch Feedback Data (t=%02d:%02d:%02d.%03d) ===",
                 tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec, ms);

  // Print summary data (first 3 points of each finger)
  FEEDBACK_DEBUG("  Thumb Force (first 3): %u, %u, %u", feedback->thumb_force[0],
                 feedback->thumb_force[1], feedback->thumb_force[2]);
  FEEDBACK_DEBUG("  Index Force (first 3): %u, %u, %u", feedback->index_force[0],
                 feedback->index_force[1], feedback->index_force[2]);
  FEEDBACK_DEBUG("  Middle Force (first 3): %u, %u, %u", feedback->middle_force[0],
                 feedback->middle_force[1], feedback->middle_force[2]);
  FEEDBACK_DEBUG("  Ring Force (first 3): %u, %u, %u", feedback->ring_force[0],
                 feedback->ring_force[1], feedback->ring_force[2]);
  FEEDBACK_DEBUG("  Pinky Force (first 3): %u, %u, %u", feedback->pinky_force[0],
                 feedback->pinky_force[1], feedback->pinky_force[2]);
  FEEDBACK_DEBUG("  Palm Force (first 3): %u, %u, %u", feedback->palm_force[0],
                 feedback->palm_force[1], feedback->palm_force[2]);

  FEEDBACK_DEBUG("  Force Total (first 3): %u, %u, %u", feedback->force_total[0],
                 feedback->force_total[1], feedback->force_total[2]);

  FEEDBACK_DEBUG("====================================");
#endif
}
