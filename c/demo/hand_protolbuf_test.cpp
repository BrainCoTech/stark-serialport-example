#include "stark-sdk.h"
#include "platform_compat.h"
// #include <unistd.h>
#include <cstdio>
#include <cstdlib>

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

int main(int argc, char* argv[]) {
    init_logging(LogLevel::LOG_LEVEL_INFO);

    // Step 1: stark_auto_detect
    printf("[Step 1] Calling stark_auto_detect...\n");
    CDetectedDeviceList* list = stark_auto_detect(false, NULL, STARK_PROTOCOL_TYPE_PROTOBUF);
    if (!list || list->count == 0) {
        printf("[Step 1] No Protobuf device found, trying protobuf_open with defaults...\n");
        if (list) free_detected_device_list(list);

        const char* port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
        DeviceHandler* handle = protobuf_open(port, 10, 0);
        if (handle) {
            printf("  protobuf_open succeeded (default slave_id=10, baudrate=115200)\n");
            test_finger_control(handle, 10);
            protobuf_close(handle);
        } else {
            printf("  protobuf_open returned NULL\n");
        }
        printf("Done.\n");
        return 0;
    }

    // Step 2: 用扫描到的参数初始化
    CDetectedDevice& dev = list->devices[0];
    printf("[Step 1] Found device: port=%s, slave_id=%d, baudrate=%u\n",
           dev.port_name, dev.slave_id, dev.baudrate);

    printf("[Step 2] Calling protobuf_open with detected params...\n");
    DeviceHandler* handle = protobuf_open(dev.port_name, dev.slave_id, dev.baudrate);
    if (handle) {
        printf("[Step 2] protobuf_open succeeded\n");
        test_finger_control(handle, dev.slave_id);
        protobuf_close(handle);
    } else {
        printf("[Step 2] protobuf_open returned NULL\n");
    }

    free_detected_device_list(list);
    printf("Done.\n");
    return 0;
}
