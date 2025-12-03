#include <ecrt.h>
#include <errno.h>
#include <execinfo.h>
#include <malloc.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/****************************************************************************/
// Macro definitions
/****************************************************************************/
#define FREQUENCY 1000 // 1000 Hz
#define MAX_WAIT_LOOP FREQUENCY * 0.1
#define CLOCK_TO_USE CLOCK_MONOTONIC

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B)                                                          \
  (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define MAX_TRAJECTORY_POINTS 10

#define STARK_VENDOR_ID 0x00bc0000
#define STARK_PRODUCT_CODE 0x00009252

/****************************************************************************/
// Enum and struct definitions
/****************************************************************************/

// Control mode enum
typedef enum {
  PositionAndTime = 1,
  PositionAndSpeed = 2,
  Speed = 3,
  Current = 4,
  Pwm = 5
} FingerCtrlMode;

// Joint index enum
typedef enum {
  // Thumb tip
  THUMB_FLEX = 0,
  // Thumb base
  THUMB_ABDUCT = 1,
  // Index finger
  INDEX_FINGER = 2,
  // Middle finger
  MIDDLE_FINGER = 3,
  // Ring finger
  RING_FINGER = 4,
  // Pinky
  PINKY = 5
} finger_index_t;

// Trajectory control struct
typedef struct {
  uint16_t positions[6]; // positions of 6 joints (2 bytes per joint)
  uint16_t durations[6]; // durations of 6 joints (2 bytes per joint)
  bool trajectory_active;
  uint32_t start_time_ms;
} TrajectoryControl;

// Trajectory point struct
typedef struct {
  uint16_t positions[6];
  uint16_t durations[6];
  uint32_t start_time_offset; // time offset relative to trajectory start
} TrajectoryPoint;

/****************************************************************************/
// Global variables
/****************************************************************************/

// EtherCAT related
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain = NULL;
static ec_domain_state_t domain_state = {};
static uint8_t *domain_data = NULL;

// Counters and time
static unsigned int counter = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

// PDO offsets
static unsigned int off_out;
static unsigned int off_in;

// Trajectory control
static TrajectoryControl traj_ctrl = {0};
static TrajectoryPoint trajectory_points[MAX_TRAJECTORY_POINTS];
static int trajectory_point_count = 0;
static int current_trajectory_point = 0;

/****************************************************************************/
// EtherCAT configuration
/****************************************************************************/

// PDO domain registration
const ec_pdo_entry_reg_t domain_regs[] = {
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x6000, 0x01,
     &off_in}, // input data
    {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x7000, 0x01,
     &off_out}, // output data
    {}};

// PDO entry definitions
ec_pdo_entry_info_t slave_pdo_entries[] = {
    // RxPDO: 0x1600 (master -> slave) multi‑finger control mode
    {0x7000, 0x01, 16}, // mult_joint_ctrl_mode
    {0x7000, 0x02, 96}, // joint_param1 (positions)  (6 x uint16)
    {0x7000, 0x03, 96}, // joint_param2 (durations/speeds) (6 x uint16)
                        //
    // RxPDO: 0x1601 (master -> slave) single‑finger control mode
    {0x7010, 0x01, 8},  // single_joint_ctrl_mode
    {0x7010, 0x02, 8},  // single_joint_id
    {0x7010, 0x03, 16}, // single_joint_param1
    {0x7010, 0x04, 16}, // single_joint_param2

    // TxPDO: 0x1A00 (slave -> master)
    {0x6000, 0x01, 96}, // joint_pos (6 x uint16)
    {0x6000, 0x02, 96}, // joint_spd (6 x uint16)
    {0x6000, 0x03, 96}, // joint_cur (6 x int16)
    {0x6000, 0x04, 96}, // joint_status (6 x uint16)

    // TxPDO: 0x1A01 (slave -> master) touch data
    {0x6010, 0x01, 80},  // force_normal (5 x uint16)
    {0x6010, 0x02, 80},  // force_tangential (5 x uint16)
    {0x6010, 0x03, 80},  // force_direction (5 x uint16)
    {0x6010, 0x04, 160}, // proximity (5 x uint32)
    {0x6010, 0x05, 80},  // touch_status (5 x uint16)
};

// PDO mapping
ec_pdo_info_t slave_pdos[] = {
    {0x1600, 3, &slave_pdo_entries[0]},  // RxPDO: 0x7000:01-03 multi‑finger control
    {0x1601, 4, &slave_pdo_entries[3]},  // RxPDO: 0x7010:01-04 single‑finger control
    {0x1A00, 4, &slave_pdo_entries[7]},  // TxPDO: 0x6000:01-04 joint feedback
    {0x1A01, 5, &slave_pdo_entries[11]}, // TxPDO: 0x6010:01-05 touch feedback
};

// Sync manager configuration
ec_sync_info_t slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, &slave_pdos[0], EC_WD_ENABLE}, // SM2: RxPDO
    {3, EC_DIR_INPUT, 2, &slave_pdos[2], EC_WD_DISABLE}, // SM3: TxPDO
    {0xff}};

/****************************************************************************/
// Function declarations
/****************************************************************************/
void print_hex(unsigned char *data, int len);
int get_mills();
struct timespec timespec_add(struct timespec time1, struct timespec time2);
void check_domain_state(void);
void check_master_state(void);
void handler(int sig);

/****************************************************************************/
// Utility functions
/****************************************************************************/

// Get joint name string
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

int get_mills() {
  struct timespec ts;
  clock_gettime(CLOCK_TO_USE, &ts);
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

void print_hex(unsigned char *data, int len) {
  for (int i = 0; i < len; i++) {
    printf("0x%02X", data[i]);
    if (i < len - 1) {
      printf(" ");
    }
  }
  printf("\n");
}

/****************************************************************************/
// EtherCAT state check functions
/****************************************************************************/

void check_domain_state(void) {
  ec_domain_state_t ds;

  ecrt_domain_state(domain, &ds);

  if (ds.working_counter != domain_state.working_counter)
    printf("domain: WC %u.\n", ds.working_counter);
  if (ds.wc_state != domain_state.wc_state)
    printf("domain: State %u.\n", ds.wc_state);

  domain_state = ds;
}

void check_master_state(void) {
  ec_master_state_t ms;
  ecrt_master_state(master, &ms);

  if (ms.slaves_responding != master_state.slaves_responding)
    printf("%u slave(s).\n", ms.slaves_responding);
  if (ms.al_states != master_state.al_states) {
    printf("AL states: 0x%02X.\n", ms.al_states);
    bool slaves_operational = (ms.al_states == 0x08);
    if (slaves_operational) {
      printf("All slaves now OPERATIONAL.\n");
    }
  }
  if (ms.link_up != master_state.link_up)
    printf("Link is %s.\n", ms.link_up ? "up" : "down");

  master_state = ms;
}

/****************************************************************************/
// Multi‑joint control functions
/****************************************************************************/

// Initialize trajectory control
void init_trajectory() {
  memset(&traj_ctrl, 0, sizeof(TrajectoryControl));
  traj_ctrl.trajectory_active = false;
  printf("Trajectory control initialized.\n");
}

// 1. Position‑and‑time control mode
void set_position_and_time_mode(uint16_t *positions, uint16_t *durations) {
  // Copy position and duration data
  memcpy(traj_ctrl.positions, positions, sizeof(uint16_t) * 6);
  memcpy(traj_ctrl.durations, durations, sizeof(uint16_t) * 6);

  // Write control data to EtherCAT domain
  EC_WRITE_U16(domain_data + off_out, PositionAndTime);
  memcpy(domain_data + off_out + 2, positions, 12);  // positions
  memcpy(domain_data + off_out + 14, durations, 12); // durations

  // Record start time
  traj_ctrl.start_time_ms = get_mills();
  traj_ctrl.trajectory_active = true;

  printf("Set PositionAndTime mode:\n");
  printf("Positions: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", positions[i]);
  }
  printf("\nDurations: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", durations[i]);
  }
  printf("\n");
}

// 2. Position‑and‑speed control mode
void set_position_and_speed_mode(uint16_t *positions, uint16_t *speeds) {
  // Write control data
  EC_WRITE_U16(domain_data + off_out, PositionAndSpeed);
  memcpy(domain_data + off_out + 2, positions, 12); // positions
  memcpy(domain_data + off_out + 14, speeds, 12);   // speeds

  printf("Set PositionAndSpeed mode:\n");
  printf("Positions: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", positions[i]);
  }
  printf("\nSpeeds: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", speeds[i]);
  }
  printf("\n");
}

// 3. Speed control mode
void set_speed_mode(int16_t *speeds) {
  // Write control data
  EC_WRITE_U16(domain_data + off_out, Speed);
  memcpy(domain_data + off_out + 2, speeds, 12); // speeds
  memset(domain_data + off_out + 14, 0, 12);     // param2 cleared

  printf("Set Speed mode:\n");
  printf("Speeds: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", speeds[i]);
  }
  printf("\n");
}

// 4. Current control mode
void set_current_mode(int16_t *currents) {
  // Write control data
  EC_WRITE_U16(domain_data + off_out, Current);
  memcpy(domain_data + off_out + 2, currents, 12); // currents
  memset(domain_data + off_out + 14, 0, 12);       // param2 cleared

  printf("Set Current mode:\n");
  printf("Currents: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", currents[i]);
  }
  printf("\n");
}

// 5. PWM control mode
void set_pwm_mode(uint16_t *pwm_values) {
  // Write control data
  EC_WRITE_U16(domain_data + off_out, Pwm);
  memcpy(domain_data + off_out + 2, pwm_values, 12); // pwm_values
  memset(domain_data + off_out + 14, 0, 12);         // param2 cleared

  printf("Set PWM mode:\n");
  printf("PWM values: ");
  for (int i = 0; i < 6; i++) {
    printf("%d ", pwm_values[i]);
  }
  printf("\n");
}

/****************************************************************************/
// Single‑joint control functions
/****************************************************************************/

// Single‑joint position‑and‑time control
void set_single_joint_position_time(finger_index_t joint_id, uint16_t position,
                                    uint16_t duration) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, PositionAndTime);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, position);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, duration);

  // printf("Single joint %s(%d): PositionAndTime, pos=%d, time=%d\n",
  //        get_joint_name(joint_id), joint_id, position, duration);
}

// Single‑joint position‑and‑speed control
void set_single_joint_position_speed(finger_index_t joint_id, uint16_t position,
                                     uint16_t speed) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, PositionAndSpeed);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, position);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, speed);

  // printf("Single joint %s(%d): PositionAndSpeed, pos=%d, speed=%d\n",
  //        get_joint_name(joint_id), joint_id, position, speed);
}

// Single‑joint speed control
void set_single_joint_speed(finger_index_t joint_id, uint16_t speed) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, Speed);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, speed);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, 0);

  // printf("Single joint %s(%d): Speed, speed=%d\n", get_joint_name(joint_id),
  //        joint_id, speed);
}

// Single‑joint current control
void set_single_joint_current(finger_index_t joint_id, uint16_t current) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, Current);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, current);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, 0);

  printf("Single joint %s(%d): Current, current=%d\n", get_joint_name(joint_id),
         joint_id, current);
}

// Single‑joint PWM control
void set_single_joint_pwm(finger_index_t joint_id, uint16_t pwm_value) {
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data + off_out + single_offset, Pwm);
  EC_WRITE_U8(domain_data + off_out + single_offset + 1, (uint8_t)joint_id);
  EC_WRITE_U16(domain_data + off_out + single_offset + 2, pwm_value);
  EC_WRITE_U16(domain_data + off_out + single_offset + 4, 0);

  printf("Single joint %s(%d): PWM, value=%d\n", get_joint_name(joint_id),
         joint_id, pwm_value);
}

/****************************************************************************/
// Advanced gesture control functions
/****************************************************************************/

// Set thumb control
void set_thumb_control(uint16_t flex_pos, uint16_t abduct_pos,
                       uint16_t duration) {
  set_single_joint_position_time(THUMB_FLEX, flex_pos, duration);
  set_single_joint_position_time(THUMB_ABDUCT, abduct_pos, duration);
  printf("Thumb control set: flex=%d, abduct=%d, duration=%d\n", flex_pos,
         abduct_pos, duration);
}

// Set finger control (index, middle, ring, pinky)
void set_finger_control(finger_index_t finger, uint16_t position,
                        uint16_t duration) {
  if (finger >= INDEX_FINGER && finger <= PINKY) {
    set_single_joint_position_time(finger, position, duration);
  } else {
    printf("Error: Invalid finger index for finger control\n");
  }
}

// Batch set all joints
void set_all_joints_position_time(uint16_t positions[6],
                                  uint16_t durations[6]) {
  for (int i = 0; i < 6; i++) {
    set_single_joint_position_time((finger_index_t)i, positions[i],
                                   durations[i]);
  }
  printf("All joints position and time set\n");
}

// Set fist gesture
void set_fist_gesture(uint16_t duration) {
  uint16_t fist_positions[6] = {400, 400, 800, 800, 800, 800}; // example values
  uint16_t fist_durations[6] = {duration, duration, duration,
                                duration, duration, duration};

  printf("Setting fist gesture...\n");
  set_position_and_time_mode(fist_positions, fist_durations);
}

// Set open hand gesture
void set_open_hand_gesture(uint16_t duration) {
  uint16_t open_positions[6] = {400, 400, 100, 100, 100, 100}; // example values
  uint16_t open_durations[6] = {duration, duration, duration,
                                duration, duration, duration};

  printf("Setting open hand gesture...\n");
  set_position_and_time_mode(open_positions, open_durations);
}

// Set OK gesture
void set_ok_gesture(uint16_t duration) {
  uint16_t ok_positions[6] = {400, 400, 700,
                              100, 100, 100}; // thumb and index touch, others open
  uint16_t ok_durations[6] = {duration, duration, duration,
                              duration, duration, duration};

  printf("Setting OK gesture...\n");
  set_position_and_time_mode(ok_positions, ok_durations);
}

/****************************************************************************/
// Trajectory control functions
/****************************************************************************/

// Configure trajectory point sequence (supports multi‑segment trajectory)
void set_trajectory_sequence(TrajectoryPoint *points, int count) {
  if (count > MAX_TRAJECTORY_POINTS) {
    printf("Warning: Trajectory point count exceeds maximum, truncating\n");
    count = MAX_TRAJECTORY_POINTS;
  }

  memcpy(trajectory_points, points, sizeof(TrajectoryPoint) * count);
  trajectory_point_count = count;
  current_trajectory_point = 0;

  // Start the first trajectory point
  if (count > 0) {
    set_position_and_time_mode(points[0].positions, points[0].durations);
    printf("Started trajectory sequence with %d points\n", count);
  }
}

// Periodic trajectory update function
void update_trajectory() {
  if (!traj_ctrl.trajectory_active) {
    return;
  }

  uint32_t current_time = get_mills();
  uint32_t elapsed_time = current_time - traj_ctrl.start_time_ms;

  // Check whether each joint needs trajectory update
  bool any_joint_active = false;

  for (int i = 0; i < 6; i++) {
    // Check whether current joint trajectory is completed
    if (elapsed_time < traj_ctrl.durations[i]) {
      any_joint_active = true;

      // Compute current target position (linear interpolation)
      uint16_t target_pos = traj_ctrl.positions[i];
      float progress = (float)elapsed_time / (float)traj_ctrl.durations[i];
      uint16_t current_pos = (uint16_t)(target_pos * progress);

      // Update single joint position
      set_single_joint_position_time((finger_index_t)i, current_pos, 100);

      // Optional: print trajectory progress
      static int debug_counter = 0;
      if (debug_counter++ % 1000 == 0) { // print once per second
        printf("Joint %s(%d): progress=%.2f%%, current_pos=%d, target=%d\n",
               get_joint_name((finger_index_t)i), i, progress * 100,
               current_pos, target_pos);
      }
    }
  }

  // If all joint trajectories are completed, stop trajectory control
  if (!any_joint_active) {
    traj_ctrl.trajectory_active = false;
    printf("All joint trajectories completed after %d ms\n", elapsed_time);
  }
}

// Advanced trajectory update function (supports multi‑segment trajectory)
void update_advanced_trajectory() {
  if (!traj_ctrl.trajectory_active || trajectory_point_count == 0) {
    return;
  }

  uint32_t current_time = get_mills();
  uint32_t elapsed_time = current_time - traj_ctrl.start_time_ms;

  // Check whether we need to switch to the next trajectory point
  if (current_trajectory_point < trajectory_point_count - 1) {
    TrajectoryPoint *next_point =
        &trajectory_points[current_trajectory_point + 1];

    if (elapsed_time >= next_point->start_time_offset) {
      current_trajectory_point++;
      set_position_and_time_mode(next_point->positions, next_point->durations);
      printf("Switched to trajectory point %d at time %d ms\n",
             current_trajectory_point, elapsed_time);
    }
  }

  // Check whether entire trajectory sequence is completed
  TrajectoryPoint *last_point = &trajectory_points[trajectory_point_count - 1];
  uint32_t total_duration = last_point->start_time_offset;
  for (int i = 0; i < 6; i++) {
    if (last_point->durations[i] > 0) {
      total_duration += last_point->durations[i];
      break;
    }
  }

  if (elapsed_time >= total_duration) {
    traj_ctrl.trajectory_active = false;
    trajectory_point_count = 0;
    current_trajectory_point = 0;
    printf("Complete trajectory sequence finished after %d ms\n", elapsed_time);
  }
}

/****************************************************************************/
// Feedback data reading functions
/****************************************************************************/

void read_joint_feedback_data() {
  // Read joint positions
  uint8_t joint_pos[12];
  memcpy(joint_pos, domain_data + off_in, 12);

  // Read joint speeds
  uint8_t joint_spd[12];
  memcpy(joint_spd, domain_data + off_in + 12, 12);

  // Read joint currents
  uint8_t joint_cur[12];
  memcpy(joint_cur, domain_data + off_in + 24, 12);

  // Read joint status
  uint8_t joint_status[12];
  memcpy(joint_status, domain_data + off_in + 36, 12);

  // Print feedback data
  printf("=== Joint Feedback Data ===\n");
  printf("Position: ");
  print_hex(joint_pos, 12);
  printf("Speed:    ");
  print_hex(joint_spd, 12);
  printf("Current:  ");
  print_hex(joint_cur, 12);
  printf("Status:   ");
  print_hex(joint_status, 12);
  printf("===========================\n");
}

/****************************************************************************/
// Tactile feedback data reading functions
// ● 5 finger normal forces (force_normal) → UINT16 ×5
// ● 5 finger tangential forces (force_tangential) → INT16 ×5
// ● 5 finger tangential force directions (force_direction) → INT16 ×5
// ● 5 finger proximity (proximity) → UINT32 ×5
// ● 5 finger tactile status (touch_status) → UINT16 ×5
/****************************************************************************/

void read_touch_feedback_data() {
  // Normal forces
  uint8_t force_normal[10];
  memcpy(force_normal, domain_data + off_in + 48, 10);
  // Tangential forces
  uint8_t force_tangential[10];
  memcpy(force_tangential, domain_data + off_in + 58, 10);
  // Tangential force directions
  uint8_t force_direction[10];
  memcpy(force_direction, domain_data + off_in + 68, 10);
  // Proximity
  uint8_t proximity[20];
  memcpy(proximity, domain_data + off_in + 78, 20);
  // Tactile status
  uint8_t touch_status[20];
  memcpy(touch_status, domain_data + off_in + 98, 20);

  // Print feedback data
  printf("===========================\n");
  printf("Force Normal: ");
  print_hex(force_normal, 10);
  printf("Force Tangential: ");
  print_hex(force_tangential, 10);
  printf("Force Direction: ");
  print_hex(force_direction, 10);
  printf("Proximity: ");
  print_hex(proximity, 20);
  printf("Touch Status: \n");

  // Print tactile status (10 bytes total, 2 bytes per sensor data)
  for (int i = 0; i < 10; i += 2) {
    uint8_t status = touch_status[i];      // Low byte: status
    uint8_t seq    = touch_status[i + 1];  // High byte: sequence number
    printf("Sensor %d -> Status: 0x%02X, Seq: %u\n", i / 2, (unsigned)status, (unsigned)seq);
  }
  printf("===========================\n");
}
/****************************************************************************/
// Main loop task
/****************************************************************************/

void cyclic_task() {
  struct timespec wakeupTime, time;
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

  // Initialize trajectory control
  init_trajectory();

  bool initial_command_sent = false;
  int demo_stage = 0;
  int demo_counter = 0;

  while (1) {
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // Write application time to master
    ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain);

    // check process data state (optional)
    check_domain_state();

    if (counter) {
      counter--;
    } else {
      counter = FREQUENCY;
      check_master_state();
    }

    // Only perform read/write operations when slave is in OPERATIONAL state
    if (master_state.al_states == 0x08) {
      // Demonstrate different control modes and gestures
      if (!initial_command_sent) {
        printf("=== Starting Joint Control Demo ===\n");
        initial_command_sent = true;
      }

      // Switch demo mode every 20 seconds
      if (demo_counter++ % (FREQUENCY * 20) == 0) {
        switch (demo_stage % 8) { // Added gesture demonstrations
        case 0: {
          // set_single_joint_position_time((finger_index_t)PINKY, 1000, 100);
          // set_single_joint_position_time((finger_index_t)PINKY, 0, 100);
          // break;

          // Position and time control mode
          uint16_t positions[6] = {400, 400, 1000, 1000, 1000, 1000};
          uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
          set_position_and_time_mode(positions, durations);
          break;
        }
        case 1: {
          // Position and speed control mode
          uint16_t positions[6] = {400, 400, 0, 0, 0, 0};
          uint16_t speeds[6] = {300, 300, 300, 300, 300, 300};
          set_position_and_speed_mode(positions, speeds);
          break;
        }
        case 2: {
          // Speed control mode
          int16_t speeds[6] = {0, 0, 300, 300, 300, 300};
          set_speed_mode(speeds);
          break;
        }
        case 3: {
          // Current control mode
          int16_t currents[6] = {-100, -100, -500, -500, -500, -500};
          set_current_mode(currents);
          break;
        }
        case 4: {
          // PWM control mode
          uint16_t pwm_values[6] = {200, 100, 500, 500, 500, 500};
          set_pwm_mode(pwm_values);
          break;
        }
        case 5: {
          // Fist gesture
          set_fist_gesture(400);
          break;
        }
        case 6: {
          // Open hand
          set_open_hand_gesture(500);
          break;
        }
        case 7: {
          // OK gesture
          set_ok_gesture(400);
          break;
        }
        }
        demo_stage++;
      }

      // Update trajectory control
      update_trajectory();

      // Periodically print feedback data
      static int print_counter = 0;
      if (print_counter++ % 10 == 0) {
        read_joint_feedback_data();
        read_touch_feedback_data();
      }
    }

    if (sync_ref_counter) {
      sync_ref_counter--;
    } else {
      sync_ref_counter = 1;
      clock_gettime(CLOCK_TO_USE, &time);
      ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
    }
    ecrt_master_sync_slave_clocks(master);

    // send process data
    ecrt_domain_queue(domain);
    ecrt_master_send(master);
  }
}

/****************************************************************************/
// Signal handling and main function
/****************************************************************************/

void handler(int sig) {
  void *array[10];
  size_t size;

  size = backtrace(array, 10);
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char **argv) {
  signal(SIGSEGV, handler);
  signal(SIGABRT, handler);

  // 1. Memory locking
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    perror("mlockall failed");
    return -1;
  }

  // 2. Set real-time scheduling policy
  struct sched_param param = {};
  param.sched_priority = 49;
  // param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  printf("Using priority %i.\n", param.sched_priority);
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    perror("sched_setscheduler failed");
  }

  // 3. Initialize EtherCAT master
  printf("Requesting master...\n");
  master = ecrt_request_master(0);
  if (!master)
    return -1;

  printf("Creating domain...\n");
  domain = ecrt_master_create_domain(master);
  if (!domain)
    return -1;

  // 4. Create slave configuration
  printf("Requesting slave...\n");
  ec_slave_config_t *sc;
  if (!(sc = ecrt_master_slave_config(master, 0, 0, STARK_VENDOR_ID,
                                      STARK_PRODUCT_CODE))) {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return -1;
  }

  printf("Configuring PDOs...\n");
  if (ecrt_slave_config_pdos(sc, EC_END, slave_syncs)) {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return -1;
  }

  printf("Registering PDO entries...\n");
  if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
    fprintf(stderr, "PDO entry registration failed!\n");
    return -1;
  }
  printf("PDO entry registration successful, off_in: %d, off_out: %d\n", off_in,
         off_out);

  printf("Activating master...\n");
  if (ecrt_master_activate(master))
    return -1;

  if (!(domain_data = ecrt_domain_data(domain))) {
    return -1;
  }
  // printf("Domain data pointer: %p, size: %ld\n", domain_data,
  // ecrt_domain_size(domain));

  printf("Starting cyclic function.\n");
  cyclic_task();
  return 0;
}
