/**
 * @file stark_auto_detect.cpp
 * @brief Stark Auto-Detect Example (cross-platform)
 * 
 * Demonstrates auto-detection and basic control using the unified API.
 * Auto-detect supports:
 * - Modbus (RS485)
 * - ZQWL USB-CAN/CANFD (SDK built-in)
 * - SocketCAN CAN/CANFD (Linux, SDK built-in)
 * 
 * Build: make stark_auto_detect.exe
 * Run:   ./stark_auto_detect.exe
 */

#include "stark-sdk.h"
#include "stark_common.h"
#include <stdio.h>
#include <string.h>

void test_finger_control(DeviceHandler *handle, uint8_t slave_id) {
    printf("\n=== Testing Finger Control ===\n");
    
    useconds_t delay = 1000 * 1000; // 1000ms

    // Close pinky
    printf("Closing pinky...\n");
    stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 1000);
    usleep(delay);

    // Open pinky
    printf("Opening pinky...\n");
    stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 0);
    usleep(delay);

    // Get status
    CMotorStatusData *status = stark_get_motor_status(handle, slave_id);
    if (status != NULL) {
        printf("Final positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
               status->positions[0], status->positions[1],
               status->positions[2], status->positions[3],
               status->positions[4], status->positions[5]);
        free_motor_status_data(status);
    }
}

int main(int argc, char const *argv[]) {
    printf("=== Stark Auto-Detect Example ===\n\n");

    init_logging(LOG_LEVEL_INFO);

    // Auto-detect and initialize device
    DeviceContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    
    if (!auto_detect_and_init(&ctx, false)) {
        return -1;
    }

    // Get and print device info
    get_and_print_device_info(ctx.handle, ctx.slave_id);

    // Test finger control
    test_finger_control(ctx.handle, ctx.slave_id);

    // Cleanup
    cleanup_device_context(&ctx);

    printf("\n=== Example completed ===\n");
    return 0;
}
