/**
 * @file ec_types.h
 * @brief EtherCAT type definitions
 *
 * This header provides shared enums and structs used across multiple
 * EtherCAT example programs.
 */

#ifndef EC_TYPES_H
#define EC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
// Basic enums
/****************************************************************************/

// Hand type enum
typedef enum { LEFT_HAND = 0, RIGHT_HAND = 1 } hand_type_t;

// Switch state enum
typedef enum { SWITCH_OFF = 0, SWITCH_ON = 1 } switch_state_t;

// Unit mode enum
typedef enum {
  NORMAL = 0,  // Dimensionless
  PHYSICAL = 1 // Physical units
} unit_mode_t;

// Control mode enum (shared across all examples)
typedef enum {
  PositionWithDuration = 1, // Renamed from PositionWithDuration for consistency
  PositionWithSpeed = 2,
  Speed = 3,
  Current = 4,
  Pwm = 5
} FingerCtrlMode;

// Joint index enum (shared across all examples)
typedef enum {
  THUMB_FLEX = 0,    // Thumb tip
  THUMB_ABDUCT = 1,  // Thumb base
  INDEX_FINGER = 2,  // Index finger
  MIDDLE_FINGER = 3, // Middle finger
  RING_FINGER = 4,   // Ring finger
  PINKY = 5          // Pinky finger
} finger_index_t;

// PDO configuration types
typedef enum {
  PDO_CONFIG_BASIC,         // Basic joint control only (revo2_pdo.cpp)
  PDO_CONFIG_TOUCH,         // Basic + touch data (revo2_touch_pdo.cpp)
  PDO_CONFIG_TOUCH_PRESSURE // Basic + pressure-sensitive touch
                            // (revo2_touch_pressure_pdo.cpp)
} pdo_config_type_t;

/****************************************************************************/
// Callback function types
/****************************************************************************/

/**
 * @brief Standard callback function type for void(void) functions
 *
 * This type is used for:
 * - Cyclic task functions
 * - Demo functions
 * - Feedback reading callbacks
 * - Any other void(void) callback
 */
typedef void (*ec_callback_t)(void);

// Pressure-sensitive data type enum
typedef enum {
  RAW = 0,        // Raw ADC value
  CALIBRATED = 1, // Calibrated value (pressure/kPa)
  FORCE = 2       // Converted to force value (unit: mN)
} pressure_data_type_t;

/****************************************************************************/
// Configuration structures
/****************************************************************************/

// Protection current configuration struct
typedef struct {
  uint16_t thumb_flex_pro_cur; // Thumb fingertip protection current
  uint16_t thumb_aux_pro_cur;  // Thumb base protection current
  uint16_t index_pro_cur;      // Index finger protection current
  uint16_t middle_pro_cur;     // Middle finger protection current
  uint16_t ring_pro_cur;       // Ring finger protection current
  uint16_t pinky_pro_cur;      // Pinky finger protection current
} protection_current_config_t;

// General configuration struct
typedef struct {
  uint8_t hand_type;        // Hand type
  uint8_t led_switch;       // LED switch
  uint8_t buzzer_switch;    // Buzzer switch
  uint8_t vibrator_switch;  // Vibrator switch
  uint8_t unit_mode;        // Unit mode
  uint8_t auto_calibration; // Auto calibration
  uint8_t turbo_mode;       // Turbo mode
  uint32_t turbo_param;     // Turbo parameter (interval and duration)
} general_config_t;

// Touch configuration struct
typedef struct {
  uint8_t vendor_id;                // Vendor information
  char thumb_touch_fw_version[21];  // Thumb touch firmware version, 20 bytes +
                                    // null terminator
  char index_touch_fw_version[21];  // Index finger touch firmware version, 20
                                    // bytes + null terminator
  char middle_touch_fw_version[21]; // Middle finger touch firmware version, 20
                                    // bytes + null terminator
  char ring_touch_fw_version[21];   // Ring finger touch firmware version, 20
                                    // bytes + null terminator
  char pinky_touch_fw_version[21];  // Pinky finger touch firmware version, 20
                                    // bytes + null terminator
  char palm_touch_fw_version[21];   // Palm touch firmware version, 20 bytes +
                                    // null terminator
} touch_config_t;

// Pressure-sensitive touch configuration struct
typedef struct {
  uint8_t vendor_id;                  // Vendor information
  uint8_t data_type;                  // Data type
  char thumb_pressure_fw_version[21]; // Thumb pressure firmware version, 20
                                      // bytes + null terminator
  char index_pressure_fw_version[21]; // Index finger pressure firmware version,
                                      // 20 bytes + null terminator
  char middle_pressure_fw_version[21]; // Middle finger pressure firmware
                                       // version, 20 bytes + null terminator
  char ring_pressure_fw_version[21];   // Ring finger pressure firmware version,
                                       // 20 bytes + null terminator
  char pinky_pressure_fw_version[21]; // Pinky finger pressure firmware version,
                                      // 20 bytes + null terminator
  char palm_pressure_fw_version[21]; // Palm pressure firmware version, 20 bytes
                                     // + null terminator
} pressure_touch_config_t;

// Firmware information struct
typedef struct {
  char ctrl_wrist_fw_version[21]; // Controller board firmware version, 20 bytes
                                  // + null terminator
  char
      ctrl_sn[19]; // Controller board serial number, 18 bytes + null terminator
  char wrist_fw_version[21]; // Wrist board firmware version, 20 bytes + null
                             // terminator
  char wrist_sn[19]; // Wrist board serial number, 18 bytes + null terminator
} firmware_info_t;

/****************************************************************************/
// Trajectory control structures
/****************************************************************************/

// Trajectory control structure
typedef struct {
  uint16_t positions[6]; // Positions of 6 joints (2 bytes per joint)
  uint16_t durations[6]; // Durations of 6 joints (2 bytes per joint)
  bool trajectory_active;
  uint32_t start_time_ms;
} TrajectoryControl;

// Trajectory point structure
typedef struct {
  uint16_t positions[6];      // Target positions for all joints
  uint16_t durations[6];      // Durations for each joint (ms)
  uint32_t start_time_offset; // Time offset relative to trajectory start
} TrajectoryPoint;

#ifdef __cplusplus
}
#endif

#endif // EC_TYPES_H
