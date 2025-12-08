#include "stark-sdk.h"
#include <stdio.h>
#include <unistd.h>

// Function declarations
void get_touch_status(DeviceHandler *handle, uint8_t slave_id);
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);
void get_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[]) {
  // auto port_name = "COM1"; // Windows
  // auto port_name = "/dev/ttyUSB0"; // Linux

  auto cfg = auto_detect_modbus_revo1(NULL, true);
  if (cfg == NULL) {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }

  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  if (cfg != NULL)
    free_device_config(cfg);

  uint8_t slave_id_1 = 1;
  uint8_t slave_id_2 = 2;
  get_device_info(handle, slave_id_1);
  get_device_info(handle, slave_id_2);

  uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000}; // Fist
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};                 // Open hand

  useconds_t delay = 1000 * 1000; // 1000ms
  stark_set_finger_positions(handle, slave_id_1, positions_fist, 6);
  stark_set_finger_positions(handle, slave_id_2, positions_fist, 6);
  usleep(delay);
  stark_set_finger_positions(handle, slave_id_1, positions_open, 6);
  stark_set_finger_positions(handle, slave_id_2, positions_open, 6);
  usleep(delay);

  auto finger_status = stark_get_motor_status(handle, slave_id_1);
  printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
         finger_status->positions[0], finger_status->positions[1],
         finger_status->positions[2], finger_status->positions[3],
         finger_status->positions[4], finger_status->positions[5]);
  printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0],
         finger_status->speeds[1], finger_status->speeds[2],
         finger_status->speeds[3], finger_status->speeds[4],
         finger_status->speeds[5]);
  printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->currents[0],
         finger_status->currents[1], finger_status->currents[2],
         finger_status->currents[3], finger_status->currents[4],
         finger_status->currents[5]);
  printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n",
         finger_status->states[0], finger_status->states[1],
         finger_status->states[2], finger_status->states[3],
         finger_status->states[4], finger_status->states[5]);
  free_motor_status_data(finger_status);

  // get_touch_status(handle, slave_id_1);

  // // Loop to control finger positions, for testing only; too short delay may
  // affect motor lifespan while (1)
  // {
  //   printf("set_positions loop start\n");
  //   stark_set_finger_positions(handle, slave_id_1, positions_fist, 6);
  //   stark_set_finger_positions(handle, slave_id_2, positions_fist, 6);
  //   usleep(delay);
  //   get_touch_status(handle, slave_id_1); // Call get_touch_status
  //   get_touch_status(handle, slave_id_2); // Call get_touch_status
  //   stark_set_finger_positions(handle, slave_id_1, positions_open, 6);
  //   stark_set_finger_positions(handle, slave_id_2, positions_open, 6);
  //   usleep(delay);
  // }

  return 0;
}

// Get device serial number, firmware version and other information
void get_device_info(DeviceHandler *handle, uint8_t slave_id) {
  auto info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf(
        "Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n",
        slave_id, (uint8_t)info->sku_type, info->serial_number,
        info->firmware_version);
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO1_TOUCH ||
        info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH) {
      // Enable all tactile sensors
      stark_enable_touch_sensor(handle, slave_id, 0x1F);
      usleep(1000 * 1000); // wait for tactile sensor to be ready
    }
    free_device_info(info);
  } else {
    printf("Error: Failed to get device info\n");
  }
}

// Get tactile sensor status, 3D force values, self-proximity, mutual-proximity
// capacitance values, and sensor status
void get_touch_status(DeviceHandler *handle, uint8_t slave_id) {
  auto status = stark_get_touch_status(handle, slave_id);
  if (status != NULL) {
    auto data = status->items[1];
    printf("Slave[%hhu] Touch Sensor Status At Index Finger:\n", slave_id);
    printf("Normal Force 1: %hu\n", data.normal_force1);
    printf("Normal Force 2: %hu\n", data.normal_force2);
    printf("Normal Force 3: %hu\n", data.normal_force3);
    printf("Tangential Force 1: %hu\n", data.tangential_force1);
    printf("Tangential Force 2: %hu\n", data.tangential_force2);
    printf("Tangential Force 3: %hu\n", data.tangential_force3);
    printf("Tangential Direction 1: %hu\n", data.tangential_direction1);
    printf("Tangential Direction 2: %hu\n", data.tangential_direction2);
    printf("Tangential Direction 3: %hu\n", data.tangential_direction3);
    printf("Self Proximity 1: %u\n", data.self_proximity1);
    printf("Self Proximity 2: %u\n", data.self_proximity2);
    printf("Mutual Proximity: %u\n", data.mutual_proximity);
    printf("Status: %hu\n", data.status);

    // Free the allocated memory
    free_touch_finger_data(status);
  } else {
    printf("Error: Failed to get tactile sensor status\n");
  }
}

// Get device information: serial baudrate, slave address, voltage, LED info,
// button events
void get_info(DeviceHandler *handle, uint8_t slave_id) {
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);
  auto voltage = stark_get_voltage(handle, slave_id);
  printf("Slave[%hhu] Voltage: %.2fV\n", slave_id, voltage / 1000.0);
  auto led_info = stark_get_led_info(handle, slave_id);
  printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode,
         led_info->color);
  free_led_info(led_info);
  auto button_event = stark_get_button_event(handle, slave_id);
  printf("Slave[%hhu] Button Event: %d, %d, %hhu\n", slave_id,
         button_event->timestamp, button_event->button_id,
         button_event->press_state);
  free_button_event(button_event);
}
