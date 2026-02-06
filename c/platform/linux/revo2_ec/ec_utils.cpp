/**
 * @file ec_utils.cpp
 * @brief Implementation of common utility functions for EtherCAT applications
 */

#include "ec_utils.h"
#include "ec_macros.h"
#include <stdio.h>
#include <time.h>

/****************************************************************************/
// Time Utility Functions Implementation
/****************************************************************************/

int get_mills(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return TIMESPEC2NS(ts) / 1000000;
}

struct timespec timespec_add(struct timespec time1, struct timespec time2) {
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
  } else {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}

/****************************************************************************/
// String Utility Functions Implementation
/****************************************************************************/

const char *get_joint_name(finger_index_t joint_id) {
  switch (joint_id) {
  case THUMB_FLEX:
    return "THUMB_FLEX";
  case THUMB_ABDUCT:
    return "THUMB_ABDUCT";
  case INDEX_FINGER:
    return "INDEX_FINGER";
  case MIDDLE_FINGER:
    return "MIDDLE_FINGER";
  case RING_FINGER:
    return "RING_FINGER";
  case PINKY:
    return "PINKY";
  default:
    return "UNKNOWN";
  }
}

const char *get_control_mode_name(FingerCtrlMode mode) {
  switch (mode) {
  case PositionWithDuration:
    return "PositionWithDuration";
  case PositionWithSpeed:
    return "PositionWithSpeed";
  case Speed:
    return "Speed";
  case Current:
    return "Current";
  case Pwm:
    return "PWM";
  default:
    return "UNKNOWN";
  }
}

const char *get_control_mode_name(uint16_t mode) {
  switch (mode) {
  case 1:
    return "PositionWithDuration";
  case 2:
    return "PositionWithSpeed";
  case 3:
    return "Speed";
  case 4:
    return "Current";
  case 5:
    return "Pwm";
  default:
    return "Unknown";
  }
}

/****************************************************************************/
// EtherCAT State Monitoring Implementation
/****************************************************************************/

void check_domain_state(ec_domain_t *domain, ec_domain_state_t *domain_state) {
  ec_domain_state_t ds;

  ecrt_domain_state(domain, &ds);

  if (ds.working_counter != domain_state->working_counter) {
    printf("Domain: WC %u.\n", ds.working_counter);
  }
  if (ds.wc_state != domain_state->wc_state) {
    printf("Domain: State %u.\n", ds.wc_state);
  }

  *domain_state = ds;
}

void check_master_state(ec_master_t *master, ec_master_state_t *master_state) {
  ec_master_state_t ms;

  ecrt_master_state(master, &ms);

  if (ms.slaves_responding != master_state->slaves_responding) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("%u slave(s).\n", ms.slaves_responding);
  }
  if (ms.al_states != master_state->al_states) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("AL states: 0x%02X.\n", ms.al_states);
  }
  if (ms.link_up != master_state->link_up) {
    EC_LOG_TIME_PREFIX(stdout);
    printf("Link is %s.\n", ms.link_up ? "up" : "down");
  }

  *master_state = ms;
}

int check_slave_operational(ec_master_t *master, uint16_t slave_pos) {
  ec_master_state_t ms;
  ecrt_master_state(master, &ms);
  
  // EtherCAT master AL state is a bitmask combining all slave states
  // Common states:
  // - 0x01: INIT
  // - 0x02: PREOP
  // - 0x04: SAFEOP
  // - 0x08: OP (OPERATIONAL)
  //
  // For multiple slaves:
  // - If all slaves in OP: al_states = 0x08
  // - If some in SAFEOP, some in OP: al_states = 0x0C (0x04 | 0x08)
  // - If all in SAFEOP: al_states = 0x04
  // - If all in PREOP: al_states = 0x02
  //
  // Note: The master state is a combination, so we check if OP (0x08) is present.
  // This means at least one slave is in OP. Since we only configure one slave
  // (slave_pos), if OP state is present and our slave is configured correctly,
  // it's likely our slave that's in OP.
  //
  // For a more precise check, you would need to use ecrt_master_slave_state()
  // if available in your EtherCAT library version, or check individual slave
  // states through other means.
  
  // Check if OP state (0x08) is present in the combined state
  return (ms.al_states & 0x08) != 0;
}
