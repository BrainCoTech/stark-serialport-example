/**
 * @file hand_demo.cpp
 * @brief Comprehensive Hand Demo - Complete Feature Showcase
 *
 * Auto-detects device and protocol, then demonstrates ALL features.
 * Supports: Modbus (RS485), CAN 2.0, CANFD, SocketCAN (Linux)
 * Devices: Revo1 (all variants), Revo2 (all variants)
 *
 * Initialization modes:
 * - Auto-detect (default): Scans all protocols and ports
 * - Manual Modbus: -m <port> <baudrate> <slave_id>
 * - Manual CAN 2.0: -c <port> <baudrate> <slave_id>
 * - Manual CANFD: -f <port> <arb_baud> <data_baud> <slave_id>
 * - SocketCAN (can_common.cpp): -s <iface> <slave_id> / -S <iface> <slave_id>
 * - SocketCAN (SDK built-in): -b <iface> <slave_id> / -B <iface> <slave_id>
 * - ZLG CAN: -z <slave_id> / -Z <slave_id>
 *
 * Features demonstrated:
 * - Device auto-detection and info
 * - Position control (single/multiple fingers)
 * - Speed control (single/multiple fingers)
 * - Current control (single/multiple fingers)
 * - PWM control (Revo2 only)
 * - Position + time/speed control (Revo2 only)
 * - Finger parameter configuration (Revo2 only)
 * - Action sequences
 * - Motor status reading
 * - Configuration reading (baudrate, motor params, force levels)
 * - Touch sensor (if available)
 * - Multi-device control (left+right hand)
 * - Custom Modbus/CAN callbacks
 *
 * Build: make hand_demo.exe
 * Run:   ./hand_demo.exe -h  (for help)
 */

#include "stark-sdk.h"
#include "../common/stark_common.h"
#include "../common/can_common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile int keep_running = 1;
static volatile int signal_received = 0;

void signal_handler(int signum) {
    if (signal_received) {
        // 第二次 Ctrl+C，强制退出
        printf("\n[INFO] Force exit...\n");
        exit(1);
    }
    signal_received = 1;
    printf("\n[INFO] Stopping...\n");
    keep_running = 0;
}

// Interruptible sleep - returns early if keep_running becomes false
void interruptible_sleep(useconds_t usec) {
    useconds_t chunk = 50000; // 50ms chunks
    while (usec > 0 && keep_running) {
        useconds_t to_sleep = (usec < chunk) ? usec : chunk;
        usleep(to_sleep);
        usec -= to_sleep;
    }
}

//=============================================================================
// Demo Functions
//=============================================================================

/**
 * Demo 1: Basic position control (Revo1 & Revo2)
 */
void demo_basic_position(DeviceHandler *handle, uint8_t slave_id) {
    printf("\n=== Demo 1: Basic Position Control ===\n");

    useconds_t delay = 1000 * 1000; // 1000ms

    // Fist gesture
    printf("[Demo] Performing fist gesture...\n");
    uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000};
    stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Open hand
    printf("[Demo] Performing open hand...\n");
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Single finger control
    printf("[Demo] Moving single finger (middle)...\n");
    stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_MIDDLE, 800);
    interruptible_sleep(delay);
    if (!keep_running) return;
    stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_MIDDLE, 0);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Get motor status
    printf("[Demo] Reading motor status...\n");
    CMotorStatusData *status = stark_get_motor_status(handle, slave_id);
    if (status != NULL) {
        printf("  Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
               status->positions[0], status->positions[1],
               status->positions[2], status->positions[3],
               status->positions[4], status->positions[5]);
        printf("  Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n",
               status->speeds[0], status->speeds[1],
               status->speeds[2], status->speeds[3],
               status->speeds[4], status->speeds[5]);
        printf("  Currents: %hd, %hd, %hd, %hd, %hd, %hd\n",
               status->currents[0], status->currents[1],
               status->currents[2], status->currents[3],
               status->currents[4], status->currents[5]);
        printf("  States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n",
               status->states[0], status->states[1],
               status->states[2], status->states[3],
               status->states[4], status->states[5]);
        free_motor_status_data(status);
    }
}

/**
 * Demo 2: Speed and Current control (Revo2 primarily, partial Revo1)
 */
void demo_speed_current(DeviceHandler *handle, uint8_t slave_id, bool uses_revo2_api) {
    printf("\n=== Demo 2: Speed & Current Control ===\n");

    useconds_t delay = 1000 * 1000; // 1000ms

    // Speed control - single finger
    printf("[Demo] Speed control - single finger (middle)...\n");
    stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 500);  // Close
    interruptible_sleep(delay);
    if (!keep_running) return;
    stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, -500); // Open
    interruptible_sleep(delay);
    if (!keep_running) return;
    stark_set_finger_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 0);    // Stop

    // Speed control - multiple fingers
    printf("[Demo] Speed control - all fingers...\n");
    int16_t speeds_close[] = {100, 100, 500, 500, 500, 500};
    stark_set_finger_speeds(handle, slave_id, speeds_close, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;
    int16_t speeds_open[] = {-100, -100, -500, -500, -500, -500};
    stark_set_finger_speeds(handle, slave_id, speeds_open, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;
    int16_t speeds_stop[] = {0, 0, 0, 0, 0, 0};
    stark_set_finger_speeds(handle, slave_id, speeds_stop, 6);

    // Current control - single finger
    printf("[Demo] Current control - single finger...\n");
    stark_set_finger_current(handle, slave_id, STARK_FINGER_ID_MIDDLE, 300);  // Close
    interruptible_sleep(delay);
    if (!keep_running) return;
    stark_set_finger_current(handle, slave_id, STARK_FINGER_ID_MIDDLE, -300); // Open
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Current control - multiple fingers
    printf("[Demo] Current control - all fingers...\n");
    int16_t currents_close[] = {200, 200, 300, 300, 300, 300};
    stark_set_finger_currents(handle, slave_id, currents_close, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;
    int16_t currents_open[] = {-200, -200, -300, -300, -300, -300};
    stark_set_finger_currents(handle, slave_id, currents_open, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // PWM control (Revo2 only)
    if (uses_revo2_api) {
        printf("[Demo] PWM control (Revo2 only)...\n");
        stark_set_finger_pwm(handle, slave_id, STARK_FINGER_ID_MIDDLE, 700);
        interruptible_sleep(delay);
        if (!keep_running) return;
        stark_set_finger_pwm(handle, slave_id, STARK_FINGER_ID_MIDDLE, -700);
        interruptible_sleep(delay);
        if (!keep_running) return;

        int16_t pwms[] = {100, 100, 700, 700, 700, 700};
        stark_set_finger_pwms(handle, slave_id, pwms, 6);
        interruptible_sleep(delay);
        if (!keep_running) return;
    }

    // Reset to open
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    interruptible_sleep(delay);
}

/**
 * Demo 3: Advanced control (Revo2 only)
 * - Position + time
 * - Position + speed
 * - Finger parameter configuration
 * - Unit mode
 */
void demo_advanced_revo2(DeviceHandler *handle, uint8_t slave_id, StarkProtocolType protocol) {
    printf("\n=== Demo 3: Advanced Control (Revo2 Only) ===\n");

    useconds_t delay = 1500 * 1000; // 1500ms

    // Unit mode
    printf("[Demo] Setting unit mode to Normalized...\n");
    stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);
    FingerUnitMode mode = stark_get_finger_unit_mode(handle, slave_id);
    printf("  Current mode: %s\n",
           mode == FINGER_UNIT_MODE_NORMALIZED ? "Normalized" : "Physical");
    if (!keep_running) return;

    // Position + time (single finger)
    printf("[Demo] Position + time control (single finger)...\n");
    stark_set_finger_position_with_millis(handle, slave_id, STARK_FINGER_ID_MIDDLE, 1000, 1000);
    interruptible_sleep(delay);
    if (!keep_running) return;
    stark_set_finger_position_with_millis(handle, slave_id, STARK_FINGER_ID_MIDDLE, 0, 1000);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Position + speed (single finger)
    printf("[Demo] Position + speed control (single finger)...\n");
    stark_set_finger_position_with_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 1000, 50);
    interruptible_sleep(delay);
    if (!keep_running) return;
    stark_set_finger_position_with_speed(handle, slave_id, STARK_FINGER_ID_MIDDLE, 0, 50);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Position + duration (multiple fingers)
    // Supported protocols: Modbus, CAN 2.0, CAN FD
    printf("[Demo] Position + duration control (all fingers)...\n");
    uint16_t positions[] = {300, 300, 500, 500, 500, 500};
    uint16_t durations[] = {500, 500, 500, 500, 500, 500};
    stark_set_finger_positions_and_durations(handle, slave_id, positions, durations, 6);
    interruptible_sleep(delay);
    if (!keep_running) return;

    // Position + speed (multiple fingers)
    // Supported protocols: Modbus, CAN FD only (NOT CAN 2.0)
    // Note: CAN 2.0 only supports position+duration due to frame size limitations
    if (protocol != STARK_PROTOCOL_TYPE_CAN) {
        printf("[Demo] Position + speed control (all fingers)...\n");
        uint16_t positions2[] = {100, 100, 800, 800, 800, 800};
        uint16_t speeds[] = {300, 300, 300, 300, 300, 300};
        stark_set_finger_positions_and_speeds(handle, slave_id, positions2, speeds, 6);
        interruptible_sleep(delay);
        if (!keep_running) return;
    } else {
        printf("[SKIP] Position + speed not supported on CAN 2.0, using duration instead...\n");
        uint16_t positions2[] = {100, 100, 800, 800, 800, 800};
        uint16_t durations2[] = {500, 500, 500, 500, 500, 500};
        stark_set_finger_positions_and_durations(handle, slave_id, positions2, durations2, 6);
        interruptible_sleep(delay);
        if (!keep_running) return;
    }

    // Read finger parameters
    printf("[Demo] Reading finger parameters...\n");
    StarkFingerId finger = STARK_FINGER_ID_MIDDLE;
    uint16_t max_pos = stark_get_finger_max_position(handle, slave_id, finger);
    uint16_t min_pos = stark_get_finger_min_position(handle, slave_id, finger);
    uint16_t max_speed = stark_get_finger_max_speed(handle, slave_id, finger);
    uint16_t max_current = stark_get_finger_max_current(handle, slave_id, finger);
    uint16_t prot_current = stark_get_finger_protected_current(handle, slave_id, finger);
    printf("  Middle finger params:\n");
    printf("    Max position: %hu\n", max_pos);
    printf("    Min position: %hu\n", min_pos);
    printf("    Max speed: %hu\n", max_speed);
    printf("    Max current: %hu\n", max_current);
    printf("    Protected current: %hu\n", prot_current);
    if (!keep_running) return;

    // Reset to open
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
    interruptible_sleep(delay);
}

/**
 * Demo 4: Action sequences
 */
void demo_action_sequences(DeviceHandler *handle, uint8_t slave_id) {
    printf("\n=== Demo 4: Action Sequences ===\n");

    useconds_t delay = 1500 * 1000; // 1500ms

    printf("[Demo] Running built-in gestures...\n");

    printf("  Open hand...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_OPEN);
    interruptible_sleep(delay);
    if (!keep_running) return;

    printf("  Fist...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_FIST);
    interruptible_sleep(delay);
    if (!keep_running) return;

    printf("  Two-finger pinch...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_TWO);
    interruptible_sleep(delay);
    if (!keep_running) return;

    printf("  Three-finger pinch...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_THREE);
    interruptible_sleep(delay);
    if (!keep_running) return;

    printf("  Side pinch...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_PINCH_SIDE);
    interruptible_sleep(delay);
    if (!keep_running) return;

    printf("  Point...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_POINT);
    interruptible_sleep(delay);
    if (!keep_running) return;

    printf("  Open hand (reset)...\n");
    stark_run_action_sequence(handle, slave_id, ACTION_SEQUENCE_ID_DEFAULT_GESTURE_OPEN);
    interruptible_sleep(delay);
}

/**
 * Demo 5: Device info and configuration
 * Reads various device configuration parameters
 */
void demo_device_info(DeviceHandler *handle, uint8_t slave_id, bool uses_revo2_api) {
    printf("\n=== Demo 5: Device Info & Configuration ===\n");

    // Communication settings
    printf("\n[Config] Communication:\n");
    uint32_t rs485_baud = stark_get_rs485_baudrate(handle, slave_id);
    printf("  RS485 Baudrate: %u\n", rs485_baud);

    if (uses_revo2_api) {
        uint32_t canfd_baud = stark_get_canfd_baudrate(handle, slave_id);
        printf("  CANFD Baudrate: %u\n", canfd_baud);
    }

    // System settings
    printf("\n[Config] System:\n");
    bool turbo = stark_get_turbo_mode_enabled(handle, slave_id);
    printf("  Turbo mode: %s\n", turbo ? "enabled" : "disabled");

    bool auto_cal = stark_get_auto_calibration(handle, slave_id);
    printf("  Auto calibration: %s\n", auto_cal ? "enabled" : "disabled");

    if (uses_revo2_api) {
        FingerUnitMode unit_mode = stark_get_finger_unit_mode(handle, slave_id);
        printf("  Unit mode: %s\n", unit_mode == FINGER_UNIT_MODE_NORMALIZED ? "Normalized" : "Physical");

        // Motor parameters (read from middle finger as example)
        printf("\n[Config] Motor Parameters (Middle finger):\n");
        StarkFingerId finger = STARK_FINGER_ID_MIDDLE;

        uint16_t max_pos = stark_get_finger_max_position(handle, slave_id, finger);
        uint16_t min_pos = stark_get_finger_min_position(handle, slave_id, finger);
        printf("  Position range: %hu - %hu\n", min_pos, max_pos);

        uint16_t max_speed = stark_get_finger_max_speed(handle, slave_id, finger);
        printf("  Max speed: %hu\n", max_speed);

        uint16_t max_current = stark_get_finger_max_current(handle, slave_id, finger);
        uint16_t prot_current = stark_get_finger_protected_current(handle, slave_id, finger);
        printf("  Max current: %hu, Protected: %hu\n", max_current, prot_current);
    }
}

/**
 * Demo 6: Touch sensor (if available)
 * Supports both capacitive (Revo1/Revo2) and pressure (Revo2 Modulus) touch sensors
 *
 * Capacitive touch sensor differences:
 * - Revo1 Touch API (Revo1Touch, Revo1AdvancedTouch): Different sensor counts per finger
 *   - Thumb/Pinky: 2 force groups + 1 self-proximity
 *   - Index/Middle/Ring: 3 force groups + 2 self-proximity + 1 mutual-proximity
 * - Revo2 Touch API (Revo2Touch): Uniform structure for all fingers
 *   - All fingers: 1 normal force + 1 tangential force + 1 tangential direction + 1 proximity
 */
void demo_touch_sensor(DeviceHandler *handle, uint8_t slave_id, TouchSensorType touch_type,
                       bool is_revo1_touch, bool uses_revo2_api_touch) {
    printf("\n=== Demo 6: Touch Sensor ===\n");
    printf("[Info] Touch type: %s\n", get_touch_type_name_str(touch_type));

    if (touch_type == TOUCH_TYPE_CAPACITIVE) {
        // Capacitive touch sensor (Revo1 Touch, Revo1 Advanced Touch, Revo2 Touch)
        printf("[Demo] Enabling capacitive touch sensors...\n");
        stark_enable_touch_sensor(handle, slave_id, 0x1F);
        usleep(1000 * 1000); // Wait for sensors to be ready

        printf("[Demo] Reading capacitive touch status...\n");
        CTouchFingerData *touch = stark_get_touch_status(handle, slave_id);
        if (touch != NULL) {
            const char* finger_names[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

            if (is_revo1_touch) {
                // Revo1 Touch API (Revo1Touch, Revo1AdvancedTouch): Different sensor configuration per finger
                printf("[Info] Revo1 Touch API (variable sensors per finger)\n\n");

                for (int i = 0; i < 5; i++) {
                    CTouchFingerItem *item = &touch->items[i];
                    printf("  %s:\n", finger_names[i]);

                    // Thumb (i=0) and Pinky (i=4): 2 force groups + 1 self-proximity
                    // Index/Middle/Ring (i=1,2,3): 3 force groups + 2 self-proximity + 1 mutual-proximity
                    bool is_thumb_or_pinky = (i == 0 || i == 4);

                    if (is_thumb_or_pinky) {
                        // Thumb/Pinky: 2 force groups
                        printf("    Normal Force: %hu, %hu (2 groups)\n",
                               item->normal_force1, item->normal_force2);
                        printf("    Tangential Force: %hu, %hu\n",
                               item->tangential_force1, item->tangential_force2);
                        printf("    Tangential Direction: %hu, %hu\n",
                               item->tangential_direction1, item->tangential_direction2);
                        printf("    Self Proximity: %u (1 group)\n",
                               item->self_proximity1);
                    } else {
                        // Index/Middle/Ring: 3 force groups + full proximity
                        printf("    Normal Force: %hu, %hu, %hu (3 groups)\n",
                               item->normal_force1, item->normal_force2, item->normal_force3);
                        printf("    Tangential Force: %hu, %hu, %hu\n",
                               item->tangential_force1, item->tangential_force2, item->tangential_force3);
                        printf("    Tangential Direction: %hu, %hu, %hu\n",
                               item->tangential_direction1, item->tangential_direction2, item->tangential_direction3);
                        printf("    Self Proximity: %u, %u (2 groups)\n",
                               item->self_proximity1, item->self_proximity2);
                        printf("    Mutual Proximity: %u\n",
                               item->mutual_proximity);
                    }
                    printf("    Status: %hu\n", item->status);
                }
            } else if (uses_revo2_api_touch) {
                // Revo2 Touch API: Uniform structure for all fingers
                printf("[Info] Revo2 Touch API (uniform sensors per finger)\n\n");

                for (int i = 0; i < 5; i++) {
                    CTouchFingerItem *item = &touch->items[i];
                    printf("  %s:\n", finger_names[i]);
                    printf("    Normal Force: %hu\n", item->normal_force1);
                    printf("    Tangential Force: %hu\n", item->tangential_force1);
                    printf("    Tangential Direction: %hu\n", item->tangential_direction1);
                    printf("    Proximity: %u\n", item->self_proximity1);
                    printf("    Status: %hu\n", item->status);
                }
            } else {
                // Unknown touch type, print all fields
                printf("[Info] Unknown touch API, printing all fields\n\n");
                for (int i = 0; i < 5; i++) {
                    CTouchFingerItem *item = &touch->items[i];
                    printf("  %s:\n", finger_names[i]);
                    printf("    Normal Force: %hu, %hu, %hu\n",
                           item->normal_force1, item->normal_force2, item->normal_force3);
                    printf("    Tangential Force: %hu, %hu, %hu\n",
                           item->tangential_force1, item->tangential_force2, item->tangential_force3);
                    printf("    Proximity: %u, %u, %u\n",
                           item->self_proximity1, item->self_proximity2, item->mutual_proximity);
                    printf("    Status: %hu\n", item->status);
                }
            }
            free_touch_finger_data(touch);
        } else {
            printf("  [ERROR] Failed to read capacitive touch status\n");
        }
    } else if (touch_type == TOUCH_TYPE_PRESSURE) {
        // Pressure touch sensor (Revo2 Modulus)
        printf("[Demo] Pressure touch sensor (Modulus) detected.\n");
        printf("[Info] Pressure touch provides:\n");
        printf("       - Summary mode: 6 values (5 fingers + palm pressure sum)\n");
        printf("       - Detailed mode: Per-sensor point data (9 points/finger, 46 points/palm)\n");
        printf("\n");
        printf("[Demo] Creating pressure summary buffer for quick read...\n");

        // Create buffers
        CMotorStatusBuffer *motor_buf = motor_buffer_new(100);
        CPressureSummaryBuffer *pressure_buf = pressure_summary_buffer_new(100);

        if (motor_buf && pressure_buf) {
            // Create data collector in pressure summary mode
            CDataCollector *collector = data_collector_new_pressure_summary(
                handle, motor_buf, pressure_buf, slave_id,
                10,  // motor_frequency: 10Hz (low for demo)
                10,  // touch_frequency: 10Hz
                0    // enable_stats: false
            );

            if (collector) {
                printf("[Demo] Starting data collection for 2 seconds...\n");
                data_collector_start(collector);
                usleep(2000 * 1000); // Collect for 2 seconds
                data_collector_stop(collector);
                data_collector_wait(collector);

                // Read pressure summary data
                const char* part_names[] = {"Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"};
                uint16_t *out_data[6];
                uintptr_t max_counts[6] = {100, 100, 100, 100, 100, 100};
                uintptr_t out_counts[6];

                // Allocate output arrays
                for (int i = 0; i < 6; i++) {
                    out_data[i] = (uint16_t*)malloc(100 * sizeof(uint16_t));
                    out_counts[i] = 0;
                }

                if (pressure_summary_buffer_pop_all(pressure_buf, out_data, max_counts, out_counts) == 0) {
                    printf("[Demo] Pressure summary data:\n");
                    for (int i = 0; i < 6; i++) {
                        if (out_counts[i] > 0) {
                            // Show latest value
                            printf("  %s: %hu (samples: %zu)\n",
                                   part_names[i], out_data[i][out_counts[i]-1], out_counts[i]);
                        } else {
                            printf("  %s: no data\n", part_names[i]);
                        }
                    }
                }

                // Free output arrays
                for (int i = 0; i < 6; i++) {
                    free(out_data[i]);
                }

                data_collector_free(collector);
            } else {
                printf("  [ERROR] Failed to create data collector\n");
            }
        }

        // Cleanup buffers
        if (motor_buf) motor_buffer_free(motor_buf);
        if (pressure_buf) pressure_summary_buffer_free(pressure_buf);

        printf("\n[Info] For continuous pressure data collection, use motor_collector.exe:\n");
        printf("       ./motor_collector.exe summary   # Summary mode (6 values)\n");
        printf("       ./motor_collector.exe detailed  # Detailed mode (per-sensor)\n");
    } else {
        printf("[WARN] No touch sensor or unknown touch type\n");
    }
}

/**
 * Interactive loop demo
 */
void demo_interactive_loop(DeviceHandler *handle, uint8_t slave_id) {
    printf("\n=== Interactive Loop Demo ===\n");
    printf("[INFO] Press Ctrl+C to stop\n\n");

    uint16_t positions[6] = {0, 0, 0, 0, 0, 0};
    int direction = 1;
    int step = 50;
    int max_pos = 1000;

    while (keep_running) {
        // Update positions
        for (int i = 0; i < 6; i++) {
            positions[i] += direction * step;
        }

        // Reverse direction at limits
        if (positions[0] >= max_pos || positions[0] <= 0) {
            direction = -direction;
        }

        // Send position command
        stark_set_finger_positions(handle, slave_id, positions, 6);

        // Read and display status
        CMotorStatusData *status = stark_get_motor_status(handle, slave_id);
        if (status != NULL) {
            printf("\r[Motor] Cmd=[%4hu,%4hu,%4hu,%4hu,%4hu,%4hu] "
                   "Pos=[%4hu,%4hu,%4hu,%4hu,%4hu,%4hu]",
                   positions[0], positions[1], positions[2],
                   positions[3], positions[4], positions[5],
                   status->positions[0], status->positions[1],
                   status->positions[2], status->positions[3],
                   status->positions[4], status->positions[5]);
            fflush(stdout);
            free_motor_status_data(status);
        }

        usleep(50 * 1000); // 50ms = 20Hz
    }

    printf("\n\n[INFO] Resetting to zero position...\n");
    memset(positions, 0, sizeof(positions));
    stark_set_finger_positions(handle, slave_id, positions, 6);
    usleep(500 * 1000);
}

/**
 * Demo 8: Multi-device control
 * Controls multiple devices on the same bus simultaneously.
 *
 * This demo uses stark_auto_detect(scan_all=true) to find all devices,
 * then controls them together.
 *
 * Typical setup:
 * - Revo1: Left hand (slave_id=1) + Right hand (slave_id=2)
 * - Revo2: Left hand (slave_id=126/0x7E) + Right hand (slave_id=127/0x7F)
 */
void demo_multi_device(DeviceHandler *handle, uint8_t primary_slave_id) {
    printf("\n=== Demo 8: Multi-Device Control ===\n");

    // Use SDK's auto-detect to find all devices
    printf("[Demo] Scanning for all devices using stark_auto_detect...\n");

    CDetectedDeviceList* device_list = stark_auto_detect(
        true,   // scan_all: find ALL devices
        NULL,   // port: scan all ports
        0       // protocol: auto (try all)
    );

    if (device_list == NULL || device_list->count == 0) {
        printf("[WARN] No devices found via auto-detect.\n");
        printf("[TIP] For multi-device control, ensure:\n");
        printf("      1. Multiple hands connected to the same bus\n");
        printf("      2. Each device has a unique slave ID\n");
        if (device_list) free_detected_device_list(device_list);
        return;
    }

    printf("\n[Demo] Found %zu device(s):\n", device_list->count);
    for (size_t i = 0; i < device_list->count; i++) {
        CDetectedDevice* dev = &device_list->devices[i];
        printf("  [%zu] %s - %s, slave_id=%d, port=%s\n",
               i + 1,
               get_hardware_type_name_str(dev->hardware_type),
               get_protocol_name_str(dev->protocol),
               dev->slave_id,
               dev->port_name);
    }

    if (device_list->count < 2) {
        printf("\n[WARN] Multi-device demo requires at least 2 devices.\n");
        printf("       Found %zu device(s). Skipping demo.\n", device_list->count);
        free_detected_device_list(device_list);
        return;
    }

    // Initialize handlers for all devices
    printf("\n[Demo] Initializing handlers for %zu devices...\n", device_list->count);

    DeviceHandler* handlers[8];
    uint8_t slave_ids[8];
    uint8_t protocols[8];
    size_t device_count = 0;

    for (size_t i = 0; i < device_list->count && device_count < 8; i++) {
        CDetectedDevice* dev = &device_list->devices[i];

        DeviceHandler* h = init_from_detected(dev);
        if (h != NULL) {
            handlers[device_count] = h;
            slave_ids[device_count] = dev->slave_id;
            protocols[device_count] = dev->protocol;
            device_count++;
            printf("  Initialized device %zu: slave_id=%d\n", device_count, dev->slave_id);
        } else {
            printf("  [WARN] Failed to initialize device at slave_id=%d\n", dev->slave_id);
        }
    }

    free_detected_device_list(device_list);

    if (device_count < 2) {
        printf("\n[WARN] Could not initialize at least 2 devices. Skipping demo.\n");
        for (size_t i = 0; i < device_count; i++) {
            close_device_handler(handlers[i], protocols[i]);
        }
        return;
    }

    printf("\n[Demo] Controlling %zu devices simultaneously...\n", device_count);

    useconds_t delay = 1000 * 1000; // 1000ms
    uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000};
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};

    // Synchronized fist gesture
    printf("\n[Demo] All devices: Fist gesture...\n");
    for (size_t i = 0; i < device_count; i++) {
        stark_set_finger_positions(handlers[i], slave_ids[i], positions_fist, 6);
    }
    interruptible_sleep(delay);

    // Synchronized open hand
    printf("[Demo] All devices: Open hand...\n");
    for (size_t i = 0; i < device_count; i++) {
        stark_set_finger_positions(handlers[i], slave_ids[i], positions_open, 6);
    }
    interruptible_sleep(delay);

    // Alternating gestures (mirror mode)
    printf("[Demo] Alternating gestures (mirror mode)...\n");
    for (int cycle = 0; cycle < 3 && keep_running; cycle++) {
        // Device 1 fist, Device 2 open
        stark_set_finger_positions(handlers[0], slave_ids[0], positions_fist, 6);
        stark_set_finger_positions(handlers[1], slave_ids[1], positions_open, 6);
        interruptible_sleep(delay);

        // Device 1 open, Device 2 fist
        stark_set_finger_positions(handlers[0], slave_ids[0], positions_open, 6);
        stark_set_finger_positions(handlers[1], slave_ids[1], positions_fist, 6);
        interruptible_sleep(delay);
    }

    // Read status from all devices
    printf("\n[Demo] Reading motor status from all devices...\n");
    for (size_t i = 0; i < device_count; i++) {
        CMotorStatusData *status = stark_get_motor_status(handlers[i], slave_ids[i]);
        if (status != NULL) {
            printf("  Device[%d] Pos: [%hu, %hu, %hu, %hu, %hu, %hu]\n",
                   slave_ids[i],
                   status->positions[0], status->positions[1],
                   status->positions[2], status->positions[3],
                   status->positions[4], status->positions[5]);
            free_motor_status_data(status);
        }
    }

    // Reset all to open
    printf("\n[Demo] Resetting all devices to open...\n");
    for (size_t i = 0; i < device_count; i++) {
        stark_set_finger_positions(handlers[i], slave_ids[i], positions_open, 6);
    }
    interruptible_sleep(delay);

    // Cleanup handlers (except the primary one which is managed by caller)
    printf("[Demo] Cleaning up device handlers...\n");
    for (size_t i = 0; i < device_count; i++) {
        // Don't close the primary handler - it's managed by the caller
        if (handlers[i] != handle) {
            close_device_handler(handlers[i], protocols[i]);
        }
    }
}

//=============================================================================
// Custom Callbacks Setup
//=============================================================================

/**
 * Setup custom Modbus callbacks
 * Use this when you want to intercept/log Modbus communication
 * or implement your own transport layer
 */
void setup_modbus_callbacks() {
    printf("[Callback] Setting up custom Modbus callbacks...\n");

    set_modbus_read_holding_callback(
        [](uint8_t slave_id, uint16_t register_address, uint16_t *data_out,
           uint16_t count) -> int {
            printf("[Modbus RX] Read holding: slave=%d, addr=%d, count=%d\n",
                   slave_id, register_address, count);
            // TODO: Implement your own Modbus read logic here
            // Return 0 for success, non-zero for failure
            return -1; // Return -1 to use default SDK implementation
        });

    set_modbus_read_input_callback(
        [](uint8_t slave_id, uint16_t register_address, uint16_t *data_out,
           uint16_t count) -> int {
            printf("[Modbus RX] Read input: slave=%d, addr=%d, count=%d\n",
                   slave_id, register_address, count);
            // TODO: Implement your own Modbus read logic here
            return -1; // Return -1 to use default SDK implementation
        });

    set_modbus_write_callback(
        [](uint8_t slave_id, uint16_t register_address, const uint16_t *data_in,
           uint16_t count) -> int {
            printf("[Modbus TX] Write: slave=%d, addr=%d, count=%d\n",
                   slave_id, register_address, count);
            // TODO: Implement your own Modbus write logic here
            return -1; // Return -1 to use default SDK implementation
        });
}

/**
 * Setup custom CAN/CANFD callbacks
 * Use this when you have your own CAN adapter that SDK doesn't support
 */
void setup_custom_can_callbacks() {
    printf("[Callback] Setting up custom CAN/CANFD callbacks...\n");

    set_can_tx_callback(
        [](uint8_t slave_id, uint32_t can_id, const uint8_t *data,
           uintptr_t data_len) -> int {
            printf("[CAN TX] slave=%d, can_id=0x%X, len=%zu\n",
                   slave_id, can_id, data_len);
            // TODO: Send data to your CAN adapter
            // Return 0 for success, non-zero for failure
            return -1; // Return -1 to indicate not implemented
        });

    set_can_rx_callback(
        [](uint8_t slave_id, uint32_t expected_can_id, uint8_t expected_frames,
           uint32_t *can_id_out, uint8_t *data_out, uintptr_t *data_len_out) -> int {
            printf("[CAN RX] slave=%d, expected_can_id=0x%X, expected_frames=%d\n",
                   slave_id, expected_can_id, expected_frames);
            // TODO: Read data from your CAN adapter
            // Use expected_can_id to filter responses
            // Use expected_frames for multi-frame handling (0=auto-detect)
            // Return 0 for success, non-zero for failure
            return -1; // Return -1 to indicate not implemented
        });
}

//=============================================================================
// Initialization Modes
//=============================================================================

/**
 * Mode 1: Auto-detect (default)
 * Scans all protocols and ports to find devices
 */
bool init_auto_detect(CollectorContext *ctx, bool require_touch) {
    printf("\n[Init] Mode: Auto-detect\n");
    return auto_detect_and_init(ctx, require_touch);
}

/**
 * Mode 2: Manual Modbus initialization
 * Specify port, baudrate, and slave_id directly
 */
bool init_manual_modbus(CollectorContext *ctx, const char *port, uint32_t baudrate, uint8_t slave_id) {
    printf("\n[Init] Mode: Manual Modbus\n");
    printf("  Port: %s, Baudrate: %u, Slave ID: %d\n", port, baudrate, slave_id);

    ctx->handle = modbus_open(port, baudrate);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to open Modbus port\n");
        return false;
    }

    ctx->slave_id = slave_id;
    ctx->protocol = STARK_PROTOCOL_TYPE_MODBUS;
    ctx->baudrate = baudrate;
    strncpy(ctx->port_name, port, sizeof(ctx->port_name) - 1);

    // Get device info to determine type
    CDeviceInfo *info = stark_get_device_info(ctx->handle, slave_id);
    if (info != NULL) {
        ctx->hw_type = (StarkHardwareType)info->hardware_type;
        if (info->serial_number) {
            strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
        }
        free_device_info(info);
    }

    ctx->motor_freq = 20;
    ctx->touch_freq = 10;

    return true;
}

/**
 * Mode 3: Manual CAN 2.0 initialization (ZQWL adapter)
 * For Revo1 devices
 */
bool init_manual_can(CollectorContext *ctx, const char *port, uint32_t baudrate, uint8_t slave_id) {
    printf("\n[Init] Mode: Manual CAN 2.0\n");
    printf("  Port: %s, Baudrate: %u, Slave ID: %d\n", port, baudrate, slave_id);

    // Initialize ZQWL CAN adapter
    if (init_zqwl_can(port, baudrate) != 0) {
        printf("[ERROR] Failed to initialize ZQWL CAN adapter\n");
        return false;
    }

    ctx->handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN, 1);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        close_zqwl();
        return false;
    }

    ctx->slave_id = slave_id;
    ctx->protocol = STARK_PROTOCOL_TYPE_CAN;
    ctx->baudrate = baudrate;
    strncpy(ctx->port_name, port, sizeof(ctx->port_name) - 1);
    ctx->motor_freq = 20;
    ctx->touch_freq = 10;

    return true;
}

/**
 * Mode 4: Manual CANFD initialization (ZQWL adapter)
 * For Revo2 devices
 */
bool init_manual_canfd(CollectorContext *ctx, const char *port, uint32_t arb_baudrate,
                       uint32_t data_baudrate, uint8_t slave_id) {
    printf("\n[Init] Mode: Manual CANFD\n");
    printf("  Port: %s, Arb Baudrate: %u, Data Baudrate: %u, Slave ID: %d\n",
           port, arb_baudrate, data_baudrate, slave_id);

    // Initialize ZQWL CANFD adapter
    if (init_zqwl_canfd(port, arb_baudrate, data_baudrate) != 0) {
        printf("[ERROR] Failed to initialize ZQWL CANFD adapter\n");
        return false;
    }

    ctx->handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN_FD, 1);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        close_zqwl();
        return false;
    }

    ctx->slave_id = slave_id;
    ctx->protocol = STARK_PROTOCOL_TYPE_CAN_FD;
    ctx->baudrate = arb_baudrate;
    strncpy(ctx->port_name, port, sizeof(ctx->port_name) - 1);
    ctx->motor_freq = 50;
    ctx->touch_freq = 20;

    return true;
}

/**
 * Mode 5: Custom callback mode
 * Use your own CAN/Modbus adapter with custom callbacks
 */
bool init_custom_callback(CollectorContext *ctx, StarkProtocolType protocol, uint8_t slave_id) {
    printf("\n[Init] Mode: Custom Callback\n");
    printf("  Protocol: %s, Slave ID: %d\n",
           protocol == STARK_PROTOCOL_TYPE_MODBUS ? "Modbus" :
           protocol == STARK_PROTOCOL_TYPE_CAN ? "CAN" : "CANFD", slave_id);

    // Setup callbacks based on protocol
    if (protocol == STARK_PROTOCOL_TYPE_MODBUS) {
        setup_modbus_callbacks();
    } else {
        setup_custom_can_callbacks();
    }

    ctx->handle = init_device_handler(protocol, 1);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        return false;
    }

    ctx->slave_id = slave_id;
    ctx->protocol = protocol;
    ctx->motor_freq = 20;
    ctx->touch_freq = 10;

    return true;
}

#ifdef __linux__
/**
 * Mode 6: SocketCAN initialization (Linux only)
 * Use Linux kernel CAN interface
 */
bool init_socketcan(CollectorContext *ctx, const char *iface, uint8_t slave_id, bool is_canfd) {
    printf("\n[Init] Mode: SocketCAN %s\n", is_canfd ? "(CANFD)" : "(CAN 2.0)");
    printf("  Interface: %s, Slave ID: %d\n", iface, slave_id);

    // Set environment variables for can_common.cpp
    setenv("STARK_CAN_BACKEND", "socketcan", 1);
    setenv("STARK_SOCKETCAN_IFACE", iface, 1);

    // Initialize SocketCAN via can_common
    bool success = is_canfd ? setup_canfd() : setup_can();
    if (!success) {
        printf("[ERROR] Failed to initialize SocketCAN\n");
        printf("[TIP] Make sure the interface is up:\n");
        printf("      sudo ip link set %s type can bitrate 1000000\n", iface);
        printf("      sudo ip link set %s up\n", iface);
        return false;
    }

    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    ctx->handle = init_device_handler(protocol, 1);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        cleanup_can_resources();
        return false;
    }

    ctx->slave_id = slave_id;
    ctx->protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    strncpy(ctx->port_name, iface, sizeof(ctx->port_name) - 1);
    CDeviceInfo *info = stark_get_device_info(ctx->handle, slave_id);
    if (info != NULL) {
        ctx->hw_type = (StarkHardwareType)info->hardware_type;
        free_device_info(info);
    }
    ctx->motor_freq = is_canfd ? 100 : 50;
    ctx->touch_freq = is_canfd ? 50 : 20;

    return true;
}

/**
 * Mode 7: ZLG USB-CANFD initialization (Linux only)
 * Use Zhou Ligong USB-CANFD adapter
 */
bool init_zlg(CollectorContext *ctx, uint8_t slave_id, bool is_canfd) {
    printf("\n[Init] Mode: ZLG USB-CANFD %s\n", is_canfd ? "(CANFD)" : "(CAN 2.0)");
    printf("  Slave ID: %d\n", slave_id);

    // Initialize ZLG via can_common (default backend when STARK_USE_ZLG=1)
    bool success = is_canfd ? setup_canfd() : setup_can();
    if (!success) {
        printf("[ERROR] Failed to initialize ZLG CAN device\n");
        printf("[TIP] Make sure ZLG USB-CANFD is connected and libusbcanfd.so is installed\n");
        return false;
    }

    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    ctx->handle = init_device_handler(protocol, 1);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        cleanup_can_resources();
        return false;
    }

    ctx->slave_id = slave_id;
    ctx->protocol = protocol;
    strncpy(ctx->port_name, "zlg", sizeof(ctx->port_name) - 1);
    CDeviceInfo *info = stark_get_device_info(ctx->handle, slave_id);
    if (info != NULL) {
        ctx->hw_type = (StarkHardwareType)info->hardware_type;
        free_device_info(info);
    }
    ctx->motor_freq = is_canfd ? 100 : 50;
    ctx->touch_freq = is_canfd ? 50 : 20;

    return true;
}
#endif

//=============================================================================
// Main
//=============================================================================

void print_usage(const char *prog_name) {
    printf("Usage: %s [options] [demo_id]\n\n", prog_name);
    printf("Initialization modes:\n");
    printf("  (default)                    Auto-detect device and protocol\n");
    printf("  -m <port> <baud> <id>        Manual Modbus\n");
    printf("  -c <port> <baud> <id>        Manual CAN 2.0 (ZQWL)\n");
    printf("  -f <port> <arb> <data> <id>  Manual CANFD (ZQWL)\n");
#ifdef __linux__
    printf("  -s <iface> <id>              SocketCAN CAN 2.0\n");
    printf("  -S <iface> <id>              SocketCAN CANFD\n");
    printf("  -z <id>                      ZLG USB-CANFD CAN 2.0\n");
    printf("  -Z <id>                      ZLG USB-CANFD CANFD\n");
#endif
    printf("  -x <protocol> <id>           Custom callback mode (protocol: modbus/can/canfd)\n");
    printf("\n");
    printf("Demo IDs:\n");
    printf("    1 - Basic position control\n");
    printf("    2 - Speed & current control\n");
    printf("    3 - Advanced control (Revo2 only)\n");
    printf("    4 - Action sequences\n");
    printf("    5 - Device info & peripherals\n");
    printf("    6 - Touch sensor\n");
    printf("    7 - Interactive loop\n");
    printf("    8 - Multi-device control\n");
    printf("    0 - Run all demos (default)\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                           # Auto-detect, run all demos\n", prog_name);
    printf("  %s 1                         # Auto-detect, run demo 1\n", prog_name);
    printf("  %s -m /dev/ttyUSB0 460800 127 1  # Manual Modbus, run demo 1\n", prog_name);
#ifdef __linux__
    printf("  %s -s can0 1                 # SocketCAN CAN 2.0\n", prog_name);
    printf("  %s -z 1                      # ZLG CAN 2.0\n", prog_name);
    printf("  %s -Z 127                    # ZLG CANFD\n", prog_name);
#endif
}

int main(int argc, char const *argv[]) {
    setup_signal_handlers();
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== Universal Motor Control - Complete Demo ===\n\n");

    // Initialize logging
    init_logging(LOG_LEVEL_DEBUG);
    // init_logging(LOG_LEVEL_INFO);

    CollectorContext ctx;
    memset(&ctx, 0, sizeof(ctx));

    int demo_id = 0;
    int arg_idx = 1;
    bool init_success = false;

    // Parse initialization mode
    if (argc > 1 && argv[1][0] == '-') {
        char mode = argv[1][1];

        switch (mode) {
            case 'm': // Manual Modbus: -m <port> <baud> <slave_id> [demo_id]
                if (argc < 5) {
                    printf("[ERROR] -m requires: <port> <baudrate> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_manual_modbus(&ctx, argv[2], atoi(argv[3]), atoi(argv[4]));
                arg_idx = 5;
                break;

            case 'c': // Manual CAN: -c <port> <baud> <slave_id> [demo_id]
                if (argc < 5) {
                    printf("[ERROR] -c requires: <port> <baudrate> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_manual_can(&ctx, argv[2], atoi(argv[3]), atoi(argv[4]));
                arg_idx = 5;
                break;

            case 'f': // Manual CANFD: -f <port> <arb_baud> <data_baud> <slave_id> [demo_id]
                if (argc < 6) {
                    printf("[ERROR] -f requires: <port> <arb_baudrate> <data_baudrate> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_manual_canfd(&ctx, argv[2], atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
                arg_idx = 6;
                break;

            case 'x': // Custom callback: -x <protocol> <slave_id> [demo_id]
                if (argc < 4) {
                    printf("[ERROR] -x requires: <protocol> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                {
                    StarkProtocolType protocol = STARK_PROTOCOL_TYPE_MODBUS;
                    if (strcmp(argv[2], "can") == 0) {
                        protocol = STARK_PROTOCOL_TYPE_CAN;
                    } else if (strcmp(argv[2], "canfd") == 0) {
                        protocol = STARK_PROTOCOL_TYPE_CAN_FD;
                    }
                    init_success = init_custom_callback(&ctx, protocol, atoi(argv[3]));
                }
                arg_idx = 4;
                break;

#ifdef __linux__
            case 's': // SocketCAN CAN 2.0: -s <iface> <slave_id> [demo_id]
                if (argc < 4) {
                    printf("[ERROR] -s requires: <interface> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_socketcan(&ctx, argv[2], atoi(argv[3]), false);
                arg_idx = 4;
                break;

            case 'S': // SocketCAN CANFD: -S <iface> <slave_id> [demo_id]
                if (argc < 4) {
                    printf("[ERROR] -S requires: <interface> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_socketcan(&ctx, argv[2], atoi(argv[3]), true);
                arg_idx = 4;
                break;

            case 'z': // ZLG CAN 2.0: -z <slave_id> [demo_id]
                if (argc < 3) {
                    printf("[ERROR] -z requires: <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_zlg(&ctx, atoi(argv[2]), false);
                arg_idx = 3;
                break;

            case 'Z': // ZLG CANFD: -Z <slave_id> [demo_id]
                if (argc < 3) {
                    printf("[ERROR] -Z requires: <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_zlg(&ctx, atoi(argv[2]), true);
                arg_idx = 3;
                break;
#endif

            case 'h': // Help
                print_usage(argv[0]);
                return 0;

            default:
                printf("[ERROR] Unknown option: %s\n", argv[1]);
                print_usage(argv[0]);
                return -1;
        }
    } else {
        // Default: auto-detect
        init_success = init_auto_detect(&ctx, false);
    }

    if (!init_success) {
        printf("[ERROR] Device initialization failed\n");
        return -1;
    }

    // Parse demo_id from remaining arguments
    if (arg_idx < argc) {
        demo_id = atoi(argv[arg_idx]);
        if (demo_id < 0 || demo_id > 8) {
            print_usage(argv[0]);
            cleanup_collector_context(&ctx);
            return -1;
        }
    }

    // Get device info
    get_and_print_device_info(ctx.handle, ctx.slave_id);

    bool uses_revo2_api = !stark_uses_revo1_motor_api(ctx.hw_type);
    printf("\n[INFO] Motor API: %s\n", uses_revo2_api ? "Revo2" : "Revo1");
    printf("[INFO] Touch type: %s\n", get_touch_type_name_str(get_touch_sensor_type(ctx.hw_type)));

    // Run selected demo(s)
    switch (demo_id) {
        case 1:
            demo_basic_position(ctx.handle, ctx.slave_id);
            break;
        case 2:
            demo_speed_current(ctx.handle, ctx.slave_id, uses_revo2_api);
            break;
        case 3:
            if (uses_revo2_api) {
                demo_advanced_revo2(ctx.handle, ctx.slave_id, ctx.protocol);
            } else {
                printf("\n[WARN] Demo 3 (Advanced) is Revo2 only. Skipping.\n");
            }
            break;
        case 4:
            demo_action_sequences(ctx.handle, ctx.slave_id);
            break;
        case 5:
            demo_device_info(ctx.handle, ctx.slave_id, uses_revo2_api);
            break;
        case 6:
            if (stark_is_touch_device(ctx.hw_type)) {
                demo_touch_sensor(ctx.handle, ctx.slave_id, get_touch_sensor_type(ctx.hw_type),
                                  stark_uses_revo1_touch_api(ctx.hw_type), stark_uses_revo2_touch_api(ctx.hw_type));
            } else {
                printf("\n[WARN] Device has no touch sensor. Skipping demo 6.\n");
            }
            break;
        case 7:
            demo_interactive_loop(ctx.handle, ctx.slave_id);
            break;
        case 8:
            demo_multi_device(ctx.handle, ctx.slave_id);
            break;
        case 0:
        default:
            // Run all applicable demos
            demo_basic_position(ctx.handle, ctx.slave_id);
            if (!keep_running) break;

            demo_speed_current(ctx.handle, ctx.slave_id, uses_revo2_api);
            if (!keep_running) break;

            if (uses_revo2_api) {
                demo_advanced_revo2(ctx.handle, ctx.slave_id, ctx.protocol);
                if (!keep_running) break;
            }

            demo_action_sequences(ctx.handle, ctx.slave_id);
            if (!keep_running) break;

            demo_device_info(ctx.handle, ctx.slave_id, uses_revo2_api);
            if (!keep_running) break;

            if (stark_is_touch_device(ctx.hw_type)) {
                demo_touch_sensor(ctx.handle, ctx.slave_id, get_touch_sensor_type(ctx.hw_type),
                                  stark_uses_revo1_touch_api(ctx.hw_type), stark_uses_revo2_touch_api(ctx.hw_type));
            }
            break;
    }

    cleanup_collector_context(&ctx);
    printf("\n[INFO] Done!\n");

    return 0;
}
