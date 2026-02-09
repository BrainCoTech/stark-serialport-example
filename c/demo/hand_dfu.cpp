/**
 * @file hand_dfu.cpp
 * @brief Universal Firmware Upgrade (DFU)
 *
 * Auto-detects device and protocol, performs firmware upgrade.
 * Supports: Modbus (RS485), CAN 2.0, CANFD, SocketCAN, ZLG
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
 * Build: make hand_dfu.exe
 * Run:   ./hand_dfu.exe [options] [firmware_file]
 */

#include "stark-sdk.h"
#include "../common/stark_common.h"
#include "../common/dfu_common.h"
#include <stdio.h>
#include <string.h>
#include <atomic>

static std::atomic<bool> g_dfu_completed(false);
static std::atomic<bool> g_dfu_failed(false);

// Default firmware paths (relative to executable)
static const char* FIRMWARE_REVO1_BASIC = "../ota_bin/modbus/FW_MotorController_Release_SecureOTA_0.1.7.C.ota";
static const char* FIRMWARE_REVO1_TOUCH = "../ota_bin/touch/FW_MotorController_Release_SecureOTA_V1.8.53.F.ota";
static const char* FIRMWARE_REVO1_ADVANCED = "../ota_bin/stark2/Revo1.8_V1.0.3.C_2602031800.bin";
static const char* FIRMWARE_REVO2_485_CANFD = "../ota_bin/stark2/Revo2_V1.0.20.U_2601091030.bin";

/**
 * @brief Select firmware path based on hardware type
 * @param hw_type Hardware type from device detection
 * @return Firmware path or NULL if unknown
 */
const char* select_firmware(StarkHardwareType hw_type) {
    switch (hw_type) {
        case STARK_HARDWARE_TYPE_REVO1_BASIC:
            return FIRMWARE_REVO1_BASIC;
        case STARK_HARDWARE_TYPE_REVO1_TOUCH:
            return FIRMWARE_REVO1_TOUCH;
        case STARK_HARDWARE_TYPE_REVO1_ADVANCED:
        case STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH:
            return FIRMWARE_REVO1_ADVANCED;
        case STARK_HARDWARE_TYPE_REVO2_BASIC:
        case STARK_HARDWARE_TYPE_REVO2_TOUCH:
        case STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE:
            return FIRMWARE_REVO2_485_CANFD;
        default:
            return NULL;
    }
}

void print_usage(const char* prog_name) {
    printf("Usage: %s [options] [firmware_file]\n\n", prog_name);
    print_init_usage(prog_name);
    printf("\nIf no firmware file specified, auto-selects based on detected device.\n");
    printf("\nDefault firmware paths:\n");
    printf("  Revo1 Basic:          %s\n", FIRMWARE_REVO1_BASIC);
    printf("  Revo1 Touch:          %s\n", FIRMWARE_REVO1_TOUCH);
    printf("  Revo1 Advanced/Touch: %s\n", FIRMWARE_REVO1_ADVANCED);
    printf("  Revo2 (485/CANFD):    %s\n", FIRMWARE_REVO2_485_CANFD);
    printf("\nExamples:\n");
    printf("  %s                           # Auto-detect, auto-select firmware\n", prog_name);
    printf("  %s firmware.bin              # Auto-detect, custom firmware\n", prog_name);
    printf("  %s -m /dev/ttyUSB0 460800 127 firmware.bin  # Modbus\n", prog_name);
#ifdef __linux__
    printf("  %s -z 1 firmware.bin         # ZLG CAN 2.0\n", prog_name);
    printf("  %s -Z 127 firmware.bin       # ZLG CANFD\n", prog_name);
#endif
}

int main(int argc, char const *argv[]) {
    printf("=== Universal DFU ===\n\n");
    
    // Initialize logging
    init_logging(LOG_LEVEL_INFO);

    // Parse arguments and initialize device
    DeviceContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    
    int arg_idx = 1;
    
    // Check for help
    if (argc > 1 && (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)) {
        print_usage(argv[0]);
        return 0;
    }
    
    if (!parse_args_and_init(&ctx, argc, argv, &arg_idx)) {
        print_usage(argv[0]);
        return -1;
    }

    // Get firmware path from remaining arguments
    const char* firmware_path = NULL;
    if (arg_idx < argc) {
        firmware_path = argv[arg_idx];
    }

    // Get device info before upgrade
    printf("\n[INFO] Current device info:\n");
    get_and_print_device_info(ctx.handle, ctx.slave_id);

    // Auto-select firmware if not provided
    if (firmware_path == NULL) {
        firmware_path = select_firmware(ctx.hw_type);
        if (firmware_path == NULL) {
            printf("[ERROR] Cannot auto-select firmware for hardware type %d\n", ctx.hw_type);
            print_usage(argv[0]);
            cleanup_device_context(&ctx);
            return -1;
        }
        printf("[INFO] Auto-selected firmware: %s\n", firmware_path);
    }

    // Check if firmware file exists
    FILE* f = fopen(firmware_path, "rb");
    if (f == NULL) {
        printf("[ERROR] Firmware file not found: %s\n", firmware_path);
        cleanup_device_context(&ctx);
        return -1;
    }
    fclose(f);

    // Confirm upgrade
    printf("\n[WARNING] Firmware upgrade will begin.\n");
    printf("Firmware file: %s\n", firmware_path);
    printf("Press Enter to continue or Ctrl+C to cancel...\n");
    getchar();

    // Set DFU callbacks
    set_dfu_state_callback([](uint8_t slave_id, uint8_t state) {
        DfuState dfu_state = static_cast<DfuState>(state);
        const char* state_names[] = {"Idle", "Starting", "Started", "Transfer", "Completed", "Aborted"};
        const char* state_name = (state < 6) ? state_names[state] : "Unknown";
        printf("\n[DFU] Slave %d state: %s (%d)\n", slave_id, state_name, state);

        if (dfu_state == DFU_STATE_COMPLETED) {
            g_dfu_completed = true;
        } else if (dfu_state == DFU_STATE_ABORTED) {
            g_dfu_failed = true;
        }
    });

    set_dfu_progress_callback([](uint8_t slave_id, float progress) {
        printf("\r[DFU] Slave %d progress: %.1f%%", slave_id, progress * 100.0f);
        fflush(stdout);
    });

    // Start DFU
    printf("[INFO] Starting firmware upgrade...\n");
    start_dfu(ctx.handle, ctx.slave_id, firmware_path, 10);

    // Wait for completion
    printf("[INFO] Waiting for DFU to complete...\n");
    while (!g_dfu_completed && !g_dfu_failed) {
        usleep(500 * 1000); // 500ms
    }

    if (g_dfu_completed) {
        printf("\n[INFO] Firmware upgrade completed successfully!\n");
    } else {
        printf("\n[ERROR] Firmware upgrade failed!\n");
    }

    cleanup_device_context(&ctx);
    return g_dfu_completed ? 0 : -1;
}
