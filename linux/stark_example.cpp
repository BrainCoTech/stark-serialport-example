#include <stdio.h>
#include "stark-sdk.h"

int main(int argc, char const *argv[])
{
    initialize_logging(LogLevel::DEBUG);
    // initialize_logging(LogLevel::INFO);
    auto handle = modbus_open("/dev/tty.usbserial-FT9O53VF", 1, 115200);
    // auto handle = modbus_open("/dev/tty.usbserial-21220", 1, 115200);
    auto info = modbus_get_device_info(handle);
    printf("SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", info->sku_type, info->serial_number, info->firmware_version);

    // modbus_get_finger_positions(handle, (uint16_t[]){0, 0, 0, 0, 0, 0}, 6);
    modbus_set_finger_speeds(handle, (int16_t[]){50, 50, 100, 100, 100, 100}, 6);

    auto finger_status = modbus_get_motor_status(handle);
    printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
    printf("Speeds: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
    printf("Currents: %hhd, %hhd, %hhd, %hhd, %hhd, %hhd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
    printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
    free_motor_status_data(finger_status);

    // auto status = modbus_get_touch_status(handle);
    // printf("Touch Sensor Status At Index Finger:\n");
    // printf("Normal Force 1: %hu\n", status[1]->normal_force1);
    // printf("Normal Force 2: %hu\n", status[1]->normal_force2);
    // printf("Normal Force 3: %hu\n", status[1]->normal_force3);
    // printf("Tangential Force 1: %hu\n", status[1]->tangential_force1);
    // printf("Tangential Force 2: %hu\n", status[1]->tangential_force2);
    // printf("Tangential Force 3: %hu\n", status[1]->tangential_force3);
    // printf("Tangential Direction 1: %hu\n", status[1]->tangential_direction1);
    // printf("Tangential Direction 2: %hu\n", status[1]->tangential_direction2);
    // printf("Tangential Direction 3: %hu\n", status[1]->tangential_direction3);
    // printf("Self Proximity 1: %u\n", status[1]->self_proximity1);
    // printf("Self Proximity 2: %u\n", status[1]->self_proximity2);
    // printf("Mutual Proximity: %u\n", status[1]->mutual_proximity);
    // printf("Status: %hu\n", status[1]->status);

    // auto baudrate = modbus_get_baudrate(handle);
    // printf("Baudrate: %d\n", baudrate);
    // auto slave_id = modbus_get_slave_id(handle);
    // printf("Slave ID: %d\n", slave_id);
    // auto force_level = modbus_get_force_level(handle);
    // printf("Force Level: %d\n", force_level);
    // auto voltage = modbus_get_voltage(handle);
    // printf("Voltage: %.2fV\n", voltage / 1000.0);
    // auto led_info = modbus_get_led_info(handle);
    // printf("LED Info: %hhu, %hhu\n", led_info->color, led_info->mode);
    // auto button_event = modbus_get_button_event(handle);
    // printf("Button Event: %d, %d, %hhu\n", button_event->timestamp, button_event->button_id, button_event->press_state);
    free_device_info(info);
    return 0;
}
