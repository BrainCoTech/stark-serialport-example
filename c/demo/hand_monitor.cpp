/**
 * @file hand_monitor.cpp
 * @brief Real-time Hand Data Monitor
 *
 * Auto-detects device and protocol, monitors motor/touch/pressure data in real-time.
 * Supports: Modbus (RS485), CAN 2.0, CANFD, SocketCAN (Linux)
 * Devices: All Revo1 and Revo2 variants
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
 * Monitor Modes:
 *   motor     - Motor status only (all devices)
 *   touch     - Motor + Capacitive touch (Revo1/Revo2 Touch)
 *   summary   - Motor + Pressure summary (Revo2 Modulus)
 *   detailed  - Motor + Pressure detailed (Revo2 Modulus)
 *   dual      - Motor + Pressure summary + detailed (Revo2 Modulus)
 *
 * Build: make hand_monitor.exe
 * Run:   ./hand_monitor.exe [options] [mode]
 */

#include "stark-sdk.h"
#include "../common/stark_common.h"
#include "../common/can_common.h"
#include "../common/trajectory_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

#define ENABLE_CONTROL 1      // Enable trajectory control (0=disabled, 1=enabled)
#define TRAJ_LEN 20           // Number of trajectory points

// Platform-dependent frequencies
#ifdef __linux__
  #define CTRL_FREQUENCY 50   // Linux: 50Hz control
#else
  #define CTRL_FREQUENCY 20   // Windows/macOS: 20Hz control
#endif

// Collection mode enum
typedef enum {
    MODE_MOTOR = 0,
    MODE_TOUCH,
    MODE_SUMMARY,
    MODE_DETAILED,
    MODE_DUAL,
    MODE_AUTO
} CollectionMode;

// Global variable for signal handling
static volatile int keep_running = 1;

void signal_handler(int signum) {
    printf("\n[INFO] Received signal %d, stopping...\n", signum);
    keep_running = 0;
}

// ============================================================================
// Helper Functions
// ============================================================================

CollectionMode parse_mode(const char* mode_str) {
    if (mode_str == NULL) return MODE_AUTO;
    if (strcmp(mode_str, "motor") == 0) return MODE_MOTOR;
    if (strcmp(mode_str, "touch") == 0) return MODE_TOUCH;
    if (strcmp(mode_str, "summary") == 0) return MODE_SUMMARY;
    if (strcmp(mode_str, "detailed") == 0) return MODE_DETAILED;
    if (strcmp(mode_str, "dual") == 0) return MODE_DUAL;
    if (strcmp(mode_str, "auto") == 0) return MODE_AUTO;
    return MODE_AUTO;
}

const char* mode_to_string(CollectionMode mode) {
    switch (mode) {
        case MODE_MOTOR: return "motor";
        case MODE_TOUCH: return "touch";
        case MODE_SUMMARY: return "summary";
        case MODE_DETAILED: return "detailed";
        case MODE_DUAL: return "dual";
        case MODE_AUTO: return "auto";
        default: return "unknown";
    }
}

void print_usage(const char* prog_name) {
    printf("Usage: %s [options] [mode]\n\n", prog_name);
    printf("Initialization (default: auto-detect all protocols):\n");
    printf("  (no options)                 Auto-detect device and protocol\n");
    printf("  -m <port> <baud> <id>        Manual Modbus\n");
    printf("  -c <port> <baud> <id>        Manual CAN 2.0 (ZQWL)\n");
    printf("  -f <port> <arb> <data> <id>  Manual CANFD (ZQWL)\n");
#ifdef __linux__
    printf("  -s <iface> <id>              SocketCAN CAN 2.0\n");
    printf("  -S <iface> <id>              SocketCAN CANFD\n");
    printf("  -z <id>                      ZLG USB-CANFD CAN 2.0\n");
    printf("  -Z <id>                      ZLG USB-CANFD CANFD\n");
#endif
    printf("\n");
    printf("Hardware type override (use before init option):\n");
    printf("  -t <type>                    Override hardware type detection\n");
    printf("                               0=Revo1 ProtoBuf, 1=Revo1 Basic, 2=Revo1 Touch\n");
    printf("                               3=Revo1 Advanced, 4=Revo1 Advanced Touch\n");
    printf("                               5=Revo2 Basic, 6=Revo2 Touch, 7=Revo2 Touch Pressure\n");
    printf("\n");
    printf("Monitor modes:\n");
    printf("    motor    - Motor status only (all devices)\n");
    printf("    touch    - Motor + Capacitive touch (Revo1/Revo2 Touch)\n");
    printf("    summary  - Motor + Pressure summary (Revo2 Modulus)\n");
    printf("    detailed - Motor + Pressure detailed (Revo2 Modulus)\n");
    printf("    dual     - Motor + Pressure summary + detailed (Revo2 Modulus)\n");
    printf("    auto     - Auto-detect based on device type (default)\n");
    printf("\n");
    printf("CAN Backend (runtime selection):\n");
    printf("  SocketCAN (default) - use -s/-S, or STARK_CAN_BACKEND=socketcan\n");
    printf("  ZLG                 - use -z/-Z, or STARK_CAN_BACKEND=zlg\n");
    printf("  ZQWL (SDK built-in) - use -c/-f\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                # Auto-detect, auto mode\n", prog_name);
    printf("  %s motor          # Auto-detect, motor only\n", prog_name);
    printf("  %s touch          # Auto-detect, touch mode\n", prog_name);
#ifdef __linux__
    printf("  %s -z 1 motor     # ZLG CAN 2.0, motor mode\n", prog_name);
    printf("  %s -Z 127 touch   # ZLG CANFD, touch mode\n", prog_name);
    printf("  %s -t 6 -z 2 touch  # ZLG CAN 2.0, force Revo2 Touch type\n", prog_name);
#endif
}

//=============================================================================
// Initialization Functions (use stark_common.h implementations)
//=============================================================================
// All init functions are now in stark_common.cpp:
//   - init_modbus()           for Modbus
//   - init_zqwl_device()      for CAN/CANFD via ZQWL adapter
//   - init_socketcan_device() for SocketCAN (Linux)
//   - init_zlg_device()       for ZLG USB-CANFD (Linux)

// ============================================================================
// Motor-Only Collection Loop
// ============================================================================

void run_motor_collection(
    CDataCollector* collector,
    CMotorStatusBuffer* motor_buffer,
    TrajectoryControl* traj_ctrl
) {
    const size_t MAX_MOTOR_DATA = 1000;
    CMotorStatusData motor_data[MAX_MOTOR_DATA];
    size_t motor_count = 0;
    int loop_count = 0;

    while (keep_running) {
        usleep(100 * 1000); // 100ms
        loop_count++;

        // Read motor status every 2 loops (200ms)
        if (loop_count % 2 == 0) {
            size_t buffer_len = motor_buffer_len(motor_buffer);
            if (buffer_len > 0) {
                motor_buffer_pop_all(motor_buffer, motor_data, MAX_MOTOR_DATA, &motor_count);

                if (motor_count > 0) {
                    auto latest = &motor_data[motor_count - 1];

                    #if ENABLE_CONTROL
                    if (traj_ctrl) {
                        size_t traj_idx = trajectory_control_get_index(traj_ctrl);
                        printf("[Motor] Count=%zu, Traj=%zu/%d, Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                               motor_count, traj_idx, TRAJ_LEN,
                               latest->positions[0], latest->positions[1], latest->positions[2],
                               latest->positions[3], latest->positions[4], latest->positions[5]);
                    } else
                    #endif
                    {
                        printf("[Motor] Count=%zu, Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                               motor_count,
                               latest->positions[0], latest->positions[1], latest->positions[2],
                               latest->positions[3], latest->positions[4], latest->positions[5]);
                    }
                    printf("        Speeds=[%hd,%hd,%hd,%hd,%hd,%hd]\n",
                           latest->speeds[0], latest->speeds[1], latest->speeds[2],
                           latest->speeds[3], latest->speeds[4], latest->speeds[5]);
                    printf("        Currents=[%hd,%hd,%hd,%hd,%hd,%hd]\n\n",
                           latest->currents[0], latest->currents[1], latest->currents[2],
                           latest->currents[3], latest->currents[4], latest->currents[5]);
                }
            }
        }
    }
}

// ============================================================================
// Touch (Capacitive) Collection Loop
// ============================================================================

void run_touch_collection(
    CDataCollector* collector,
    CMotorStatusBuffer* motor_buffer,
    CTouchStatusBuffer* touch_buffer,
    TrajectoryControl* traj_ctrl
) {
    const size_t MAX_MOTOR_DATA = 1000;
    const size_t MAX_TOUCH_DATA = 1000;

    CMotorStatusData motor_data[MAX_MOTOR_DATA];
    CTouchFingerItem touch_data[5][MAX_TOUCH_DATA];
    size_t motor_count = 0;
    size_t touch_counts[5] = {0};

    const char* finger_names[] = {"Thumb ", "Index ", "Middle", "Ring  ", "Pinky "};
    const char* status_names[] = {"OK", "DataErr", "CommErr"};
    int loop_count = 0;

    while (keep_running) {
        usleep(1000 * 1000); // 1 second
        loop_count++;

        // Read motor status
        size_t buffer_len = motor_buffer_len(motor_buffer);
        if (buffer_len > 0) {
            motor_buffer_pop_all(motor_buffer, motor_data, MAX_MOTOR_DATA, &motor_count);
        }

        // Read touch data
        for (int finger = 0; finger < 5; finger++) {
            touch_buffer_pop_finger(
                touch_buffer, finger,
                touch_data[finger], MAX_TOUCH_DATA, &touch_counts[finger]
            );
        }

        // Print summary line
        printf("[%2ds] Motor: %4zu samples | Touch: [%zu, %zu, %zu, %zu, %zu]\n",
               loop_count, motor_count,
               touch_counts[0], touch_counts[1], touch_counts[2],
               touch_counts[3], touch_counts[4]);

        // Print touch data for each finger (Revo1 Advanced Touch format)
        for (int finger = 0; finger < 5; finger++) {
            if (touch_counts[finger] > 0) {
                auto item = &touch_data[finger][touch_counts[finger] - 1];
                const char* status_str = (item->status < 3) ? status_names[item->status] : "Unknown";

                if (finger == 0) {
                    // Thumb: 2 force groups, 1 self-proximity
                    printf("  %s: F1(%4u,%4u) F2(%4u,%4u) Prox=%u [%s]\n",
                           finger_names[finger],
                           item->normal_force1, item->tangential_force1,
                           item->normal_force2, item->tangential_force2,
                           item->self_proximity1, status_str);
                } else if (finger >= 1 && finger <= 3) {
                    // Index/Middle/Ring: 3 force groups, 2 self-proximity, 1 mutual-proximity
                    printf("  %s: F1(%4u,%4u) F2(%4u,%4u) F3(%4u,%4u) Prox=(%u,%u,%u) [%s]\n",
                           finger_names[finger],
                           item->normal_force1, item->tangential_force1,
                           item->normal_force2, item->tangential_force2,
                           item->normal_force3, item->tangential_force3,
                           item->self_proximity1, item->self_proximity2, item->mutual_proximity,
                           status_str);
                } else {
                    // Pinky: 2 force groups, 1 self-proximity, NO mutual-proximity
                    printf("  %s: F1(%4u,%4u) F2(%4u,%4u) Prox=%u [%s]\n",
                           finger_names[finger],
                           item->normal_force1, item->tangential_force1,
                           item->normal_force2, item->tangential_force2,
                           item->self_proximity1,
                           status_str);
                }
            }
        }
    }
}

// ============================================================================
// Pressure Collection Loop
// ============================================================================

void run_pressure_collection(
    CDataCollector* collector,
    CMotorStatusBuffer* motor_buffer,
    CPressureSummaryBuffer* summary_buffer,
    CPressureDetailedBuffer* detailed_buffer,
    TrajectoryControl* traj_ctrl,
    CollectionMode mode
) {
    const size_t MAX_MOTOR_DATA = 1000;
    const size_t MAX_PRESSURE_DATA = 1000;

    CMotorStatusData motor_data[MAX_MOTOR_DATA];
    uint16_t summary_data[6][MAX_PRESSURE_DATA];
    PressureDetailedItem detailed_data[6][MAX_PRESSURE_DATA];
    size_t motor_count = 0;
    size_t summary_counts[6] = {0};
    size_t detailed_counts[6] = {0};

    const char* part_names[] = {"Thumb", "Index", "Middle", "Ring", "Pinky", "Palm"};
    int loop_count = 0;

    while (keep_running) {
        usleep(100 * 1000); // 100ms
        loop_count++;

        // Read motor status every 10 loops (1 second)
        if (loop_count % 10 == 0) {
            size_t buffer_len = motor_buffer_len(motor_buffer);
            if (buffer_len > 0) {
                motor_buffer_pop_all(motor_buffer, motor_data, MAX_MOTOR_DATA, &motor_count);

                if (motor_count > 0) {
                    auto latest = &motor_data[motor_count - 1];

                    #if ENABLE_CONTROL
                    if (traj_ctrl) {
                        size_t traj_idx = trajectory_control_get_index(traj_ctrl);
                        printf("[Motor] Count=%zu, Traj=%zu/%d, Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                               motor_count, traj_idx, TRAJ_LEN,
                               latest->positions[0], latest->positions[1], latest->positions[2],
                               latest->positions[3], latest->positions[4], latest->positions[5]);
                    } else
                    #endif
                    {
                        printf("[Motor] Count=%zu, Pos=[%hu,%hu,%hu,%hu,%hu,%hu]\n",
                               motor_count,
                               latest->positions[0], latest->positions[1], latest->positions[2],
                               latest->positions[3], latest->positions[4], latest->positions[5]);
                    }
                }
            }
        }

        // Read pressure summary data
        if (summary_buffer) {
            bool has_summary_data = false;
            for (int part = 0; part < 6; part++) {
                int result = pressure_summary_buffer_pop_finger(
                    summary_buffer, part,
                    summary_data[part], MAX_PRESSURE_DATA, &summary_counts[part]
                );

                if (result == 0 && summary_counts[part] > 0) {
                    has_summary_data = true;
                }
            }

            int print_interval = (mode == MODE_DUAL) ? 10 : 1;
            if (has_summary_data && loop_count % print_interval == 0) {
                printf("[Summary] Counts: [%zu,%zu,%zu,%zu,%zu,%zu]\n",
                       summary_counts[0], summary_counts[1], summary_counts[2],
                       summary_counts[3], summary_counts[4], summary_counts[5]);

                for (int part = 0; part < 6; part++) {
                    if (summary_counts[part] > 0) {
                        uint16_t pressure = summary_data[part][summary_counts[part] - 1];
                        if (pressure > 50) {
                            printf("  %s: %hu mN [CONTACT]\n", part_names[part], pressure);
                        }
                    }
                }
            }
        }

        // Read pressure detailed data
        if (detailed_buffer) {
            bool has_detailed_data = false;
            for (int part = 0; part < 6; part++) {
                int result = pressure_detailed_buffer_pop_finger(
                    detailed_buffer, part,
                    detailed_data[part], MAX_PRESSURE_DATA, &detailed_counts[part]
                );

                if (result == 0 && detailed_counts[part] > 0) {
                    has_detailed_data = true;
                }
            }

            if (has_detailed_data) {
                printf("[Detailed] Counts: [%zu,%zu,%zu,%zu,%zu,%zu]\n",
                       detailed_counts[0], detailed_counts[1], detailed_counts[2],
                       detailed_counts[3], detailed_counts[4], detailed_counts[5]);

                for (int part = 0; part < 6; part++) {
                    if (detailed_counts[part] > 0) {
                        auto item = &detailed_data[part][detailed_counts[part] - 1];
                        uint8_t sensor_count = item->sensor_count;

                        uint32_t total = 0;
                        uint16_t max_val = 0;
                        int max_idx = 0;

                        for (int i = 0; i < sensor_count; i++) {
                            total += item->sensors_data[i];
                            if (item->sensors_data[i] > max_val) {
                                max_val = item->sensors_data[i];
                                max_idx = i;
                            }
                        }

                        if (max_val > 50) {
                            float avg = sensor_count > 0 ? (float)total / sensor_count : 0.0f;
                            printf("  %s: sensors=%u, total=%u, max=%u(#%d), avg=%.1f [CONTACT]\n",
                                   part_names[part], sensor_count, total, max_val, max_idx, avg);
                        }
                    }
                }
                printf("\n");
            }
        }
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char const *argv[]) {
    setup_signal_handlers();
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("=== Hand Data Monitor ===\n\n");

    // Initialize logging
    // init_logging(LOG_LEVEL_DEBUG);
    init_logging(LOG_LEVEL_INFO);

    DeviceContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    
    CollectionMode mode = MODE_AUTO;
    int arg_idx = 1;
    bool init_success = false;
    
    // Check for -t option first (hardware type override)
    if (argc > 2 && argv[1][0] == '-' && argv[1][1] == 't') {
        int hw_type_val = atoi(argv[2]);
        if (hw_type_val < 0 || hw_type_val > 7) {
            printf("[ERROR] Invalid hardware type: %d (valid: 0-7)\n", hw_type_val);
            print_usage(argv[0]);
            return -1;
        }
        ctx.hw_type_override = (StarkHardwareType)hw_type_val;
        printf("[INFO] Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx.hw_type_override), ctx.hw_type_override);
        
        // Shift arguments
        argc -= 2;
        argv += 2;
        arg_idx = 1;
    }
    
    // Parse initialization mode
    if (argc > 1 && argv[1][0] == '-') {
        char opt = argv[1][1];
        
        switch (opt) {
            case 'm': // Manual Modbus: -m <port> <baud> <slave_id> [mode]
                if (argc < 5) {
                    printf("[ERROR] -m requires: <port> <baudrate> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_modbus(&ctx, argv[2], atoi(argv[3]), atoi(argv[4]));
                arg_idx = 5;
                break;
                
            case 'c': // Manual CAN: -c <port> <baud> <slave_id> [mode]
                if (argc < 5) {
                    printf("[ERROR] -c requires: <port> <baudrate> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_zqwl_device(&ctx, argv[2], atoi(argv[3]), 0, atoi(argv[4]), false);
                arg_idx = 5;
                break;
                
            case 'f': // Manual CANFD: -f <port> <arb_baud> <data_baud> <slave_id> [mode]
                if (argc < 6) {
                    printf("[ERROR] -f requires: <port> <arb_baudrate> <data_baudrate> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_zqwl_device(&ctx, argv[2], atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), true);
                arg_idx = 6;
                break;

#ifdef __linux__
            case 's': // SocketCAN CAN 2.0: -s <iface> <slave_id> [mode]
                if (argc < 4) {
                    printf("[ERROR] -s requires: <interface> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_socketcan_device(&ctx, argv[2], atoi(argv[3]), false);
                arg_idx = 4;
                break;
                
            case 'S': // SocketCAN CANFD: -S <iface> <slave_id> [mode]
                if (argc < 4) {
                    printf("[ERROR] -S requires: <interface> <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_socketcan_device(&ctx, argv[2], atoi(argv[3]), true);
                arg_idx = 4;
                break;

            case 'z': // ZLG CAN 2.0: -z <slave_id> [mode]
                if (argc < 3) {
                    printf("[ERROR] -z requires: <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_zlg_device(&ctx, atoi(argv[2]), false);
                arg_idx = 3;
                break;
                
            case 'Z': // ZLG CANFD: -Z <slave_id> [mode]
                if (argc < 3) {
                    printf("[ERROR] -Z requires: <slave_id>\n");
                    print_usage(argv[0]);
                    return -1;
                }
                init_success = init_zlg_device(&ctx, atoi(argv[2]), true);
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
        bool require_touch = false;
        if (argc > 1) {
            mode = parse_mode(argv[1]);
            require_touch = (mode == MODE_TOUCH || mode == MODE_SUMMARY ||
                            mode == MODE_DETAILED || mode == MODE_DUAL);
            arg_idx = 2;
        }
        init_success = auto_detect_and_init(&ctx, require_touch);
    }
    
    if (!init_success) {
        printf("[ERROR] Device initialization failed\n");
        return -1;
    }
    
    // Parse mode from remaining arguments
    if (arg_idx < argc) {
        mode = parse_mode(argv[arg_idx]);
    }

    // Auto-select mode based on device type
    if (mode == MODE_AUTO) {
        TouchSensorType touch_type = get_touch_sensor_type(ctx.hw_type);
        if (touch_type == TOUCH_TYPE_NONE) {
            mode = MODE_MOTOR;
        } else if (touch_type == TOUCH_TYPE_PRESSURE) {
            // Revo2 Modulus (pressure touch) - use summary mode by default
            mode = MODE_SUMMARY;
        } else {
            // Capacitive touch (Revo1 Touch, Revo2 Touch)
            mode = MODE_TOUCH;
        }
        printf("[INFO] Auto-selected mode: %s\n", mode_to_string(mode));
    }

    // Validate mode against device capabilities
    TouchSensorType touch_type = get_touch_sensor_type(ctx.hw_type);
    if ((mode == MODE_SUMMARY || mode == MODE_DETAILED || mode == MODE_DUAL) &&
        touch_type != TOUCH_TYPE_PRESSURE) {
        printf("[WARN] Pressure modes (summary/detailed/dual) require Revo2 Modulus device.\n");
        printf("[WARN] Current device touch type: %s\n", get_touch_type_name_str(touch_type));
        if (touch_type != TOUCH_TYPE_NONE) {
            printf("[INFO] Falling back to 'touch' mode (capacitive).\n");
            mode = MODE_TOUCH;
        } else {
            printf("[INFO] Falling back to 'motor' mode.\n");
            mode = MODE_MOTOR;
        }
    }

    if (mode == MODE_TOUCH && touch_type == TOUCH_TYPE_NONE) {
        printf("[WARN] Touch mode requires a touch-enabled device.\n");
        printf("[INFO] Falling back to 'motor' mode.\n");
        mode = MODE_MOTOR;
    }

    printf("[INFO] Collection mode: %s\n", mode_to_string(mode));
    printf("[INFO] Device: %s, Touch: %s\n",
           get_hardware_type_name_str(ctx.hw_type),
           get_touch_type_name_str(touch_type));

    // Enable touch sensor if needed
    if (mode != MODE_MOTOR && touch_type != TOUCH_TYPE_NONE) {
        printf("[INFO] Enabling touch sensor...\n");
        uint8_t touch_bits = (mode == MODE_SUMMARY || mode == MODE_DETAILED || mode == MODE_DUAL)
                             ? 0x3F : 0x1F;  // 0x3F includes palm for pressure
        stark_enable_touch_sensor(ctx.handle, ctx.slave_id, touch_bits);
        usleep(1000 * 1000);
    }

    // ========================================================================
    // Create buffers based on mode
    // ========================================================================

    printf("[INFO] Creating buffers...\n");

    auto motor_buffer = motor_buffer_new(1000);
    if (motor_buffer == NULL) {
        fprintf(stderr, "[ERROR] Failed to create motor buffer.\n");
        cleanup_device_context(&ctx);
        return -1;
    }

    CTouchStatusBuffer* touch_buffer = NULL;
    CPressureSummaryBuffer* summary_buffer = NULL;
    CPressureDetailedBuffer* detailed_buffer = NULL;

    if (mode == MODE_TOUCH) {
        touch_buffer = touch_buffer_new(1000);
        if (touch_buffer == NULL) {
            fprintf(stderr, "[ERROR] Failed to create touch buffer.\n");
            motor_buffer_free(motor_buffer);
            cleanup_device_context(&ctx);
            return -1;
        }
    }

    if (mode == MODE_SUMMARY || mode == MODE_DUAL) {
        summary_buffer = pressure_summary_buffer_new(1000);
        if (summary_buffer == NULL) {
            fprintf(stderr, "[ERROR] Failed to create summary buffer.\n");
            motor_buffer_free(motor_buffer);
            cleanup_device_context(&ctx);
            return -1;
        }
    }

    if (mode == MODE_DETAILED || mode == MODE_DUAL) {
        detailed_buffer = pressure_detailed_buffer_new(1000);
        if (detailed_buffer == NULL) {
            fprintf(stderr, "[ERROR] Failed to create detailed buffer.\n");
            if (summary_buffer) pressure_summary_buffer_free(summary_buffer);
            motor_buffer_free(motor_buffer);
            cleanup_device_context(&ctx);
            return -1;
        }
    }

    // ========================================================================
    // Create data collector based on mode
    // ========================================================================

    printf("[INFO] Creating data collector...\n");

    // Platform-dependent frequencies
    #ifdef __linux__
        const uint32_t motor_freq = 100;
        const uint32_t touch_freq = 50;
        const uint32_t summary_freq = 50;
        const uint32_t detailed_freq = 10;
    #else
        const uint32_t motor_freq = 30;
        const uint32_t touch_freq = 10;
        const uint32_t summary_freq = 10;
        const uint32_t detailed_freq = 5;
    #endif

    CDataCollector* collector = NULL;

    switch (mode) {
        case MODE_MOTOR:
            printf("[INFO] Motor frequency: %dHz\n", motor_freq);
            collector = data_collector_new_basic(
                ctx.handle, motor_buffer, ctx.slave_id, motor_freq, 1
            );
            break;

        case MODE_TOUCH:
            printf("[INFO] Motor: %dHz, Touch: %dHz\n", motor_freq, touch_freq);
            collector = data_collector_new_capacitive(
                ctx.handle, motor_buffer, touch_buffer,
                ctx.slave_id, motor_freq, touch_freq, 1
            );
            break;

        case MODE_SUMMARY:
            printf("[INFO] Motor: %dHz, Summary: %dHz\n", motor_freq, summary_freq);
            collector = data_collector_new_pressure_summary(
                ctx.handle, motor_buffer, summary_buffer,
                ctx.slave_id, motor_freq, summary_freq, 1
            );
            break;

        case MODE_DETAILED:
            printf("[INFO] Motor: %dHz, Detailed: %dHz\n", motor_freq, detailed_freq);
            collector = data_collector_new_pressure_detailed(
                ctx.handle, motor_buffer, detailed_buffer,
                ctx.slave_id, motor_freq, detailed_freq, 1
            );
            break;

        case MODE_DUAL:
            printf("[INFO] Motor: %dHz, Summary: %dHz, Detailed: %dHz\n",
                   motor_freq, summary_freq, detailed_freq);
            collector = data_collector_new_pressure_hybrid(
                ctx.handle, motor_buffer, summary_buffer, detailed_buffer,
                ctx.slave_id, motor_freq, summary_freq, detailed_freq, 1
            );
            break;

        default:
            fprintf(stderr, "[ERROR] Invalid mode.\n");
            break;
    }

    if (collector == NULL) {
        fprintf(stderr, "[ERROR] Failed to create data collector.\n");
        if (detailed_buffer) pressure_detailed_buffer_free(detailed_buffer);
        if (summary_buffer) pressure_summary_buffer_free(summary_buffer);
        if (touch_buffer) touch_buffer_free(touch_buffer);
        motor_buffer_free(motor_buffer);
        cleanup_device_context(&ctx);
        return -1;
    }

    // Start data collection
    printf("[INFO] Starting data collector...\n");
    if (data_collector_start(collector) != 0) {
        fprintf(stderr, "[ERROR] Failed to start data collector.\n");
        data_collector_free(collector);
        if (detailed_buffer) pressure_detailed_buffer_free(detailed_buffer);
        if (summary_buffer) pressure_summary_buffer_free(summary_buffer);
        if (touch_buffer) touch_buffer_free(touch_buffer);
        motor_buffer_free(motor_buffer);
        cleanup_device_context(&ctx);
        return -1;
    }

    printf("[INFO] Data collector started!\n");

    // ========================================================================
    // Optional: Start trajectory control
    // ========================================================================

    TrajectoryControl traj_ctrl;
    uint16_t* trajectory = NULL;
    bool traj_started = false;

    #if ENABLE_CONTROL
    printf("[INFO] Initializing trajectory control...\n");
    
    // Set unit mode to Normalized for Revo2 devices
    if (!stark_uses_revo1_motor_api(ctx.hw_type)) {
        printf("[INFO] Setting unit mode to Normalized (Revo2)...\n");
        stark_set_finger_unit_mode(ctx.handle, ctx.slave_id, FINGER_UNIT_MODE_NORMALIZED);
        FingerUnitMode mode = stark_get_finger_unit_mode(ctx.handle, ctx.slave_id);
        printf("  Current mode: %s\n", mode == FINGER_UNIT_MODE_NORMALIZED ? "Normalized" : "Physical");
    }

    trajectory = init_cosine_trajectory(TRAJ_LEN, 0, 1000);
    if (trajectory == NULL) {
        fprintf(stderr, "[WARN] Failed to create trajectory, continuing without control.\n");
    } else {
        trajectory_control_init(
            &traj_ctrl, ctx.handle, ctx.slave_id,
            STARK_FINGER_ID_RING,
            trajectory, TRAJ_LEN, CTRL_FREQUENCY
        );

        if (trajectory_control_start(&traj_ctrl) == 0) {
            traj_started = true;
            printf("[INFO] Trajectory control started (Ring finger, %dHz)\n", CTRL_FREQUENCY);
        } else {
            fprintf(stderr, "[WARN] Failed to start trajectory control.\n");
            free(trajectory);
            trajectory = NULL;
        }
    }
    #endif

    printf("[INFO] Press Ctrl+C to stop...\n\n");

    // ========================================================================
    // Run collection loop based on mode
    // ========================================================================

    TrajectoryControl* traj_ptr = traj_started ? &traj_ctrl : NULL;

    switch (mode) {
        case MODE_MOTOR:
            run_motor_collection(collector, motor_buffer, traj_ptr);
            break;

        case MODE_TOUCH:
            run_touch_collection(collector, motor_buffer, touch_buffer, traj_ptr);
            break;

        case MODE_SUMMARY:
        case MODE_DETAILED:
        case MODE_DUAL:
            run_pressure_collection(collector, motor_buffer, summary_buffer,
                                    detailed_buffer, traj_ptr, mode);
            break;

        default:
            break;
    }

    // ========================================================================
    // Cleanup
    // ========================================================================

    #if ENABLE_CONTROL
    if (traj_started) {
        printf("\n[INFO] Stopping trajectory control...\n");
        trajectory_control_stop(&traj_ctrl);
    }
    if (trajectory) {
        free(trajectory);
    }
    #endif

    printf("[INFO] Stopping data collector...\n");
    data_collector_stop(collector);

    printf("[INFO] Cleaning up...\n");
    data_collector_free(collector);
    if (detailed_buffer) pressure_detailed_buffer_free(detailed_buffer);
    if (summary_buffer) pressure_summary_buffer_free(summary_buffer);
    if (touch_buffer) touch_buffer_free(touch_buffer);
    motor_buffer_free(motor_buffer);
    cleanup_device_context(&ctx);

    printf("[INFO] Done!\n");
    return 0;
}
