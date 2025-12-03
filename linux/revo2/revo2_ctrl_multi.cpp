#include <stdio.h>
#include <unistd.h>
#include "stark-sdk.h"

// Function declarations
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);
void get_info(DeviceHandler *handle, uint8_t slave_id);

int main(int argc, char const *argv[])
{
  init_cfg(STARK_PROTOCOL_TYPE_MODBUS, LOG_LEVEL_DEBUG); // Initialize configuration
  auto cfg = auto_detect_modbus_revo2("/dev/ttyUSB0", true); // Replace with actual serial port name; passing NULL will try auto-detection
  if (cfg == NULL)
  {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }
  // uint8_t slave_id = cfg->slave_id;
  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  free_device_config(cfg);

  // Option 1: connect multiple devices through a single serial port (each device must have a unique ID)
  uint8_t slave_id_1 = 0x7e; // Default left-hand ID is 0x7e, right-hand ID is 0x7f
  uint8_t slave_id_2 = 0x7f;
  get_device_info(handle, slave_id_1);
  get_device_info(handle, slave_id_2);

  uint16_t positions_fist[] = {400, 400, 1000, 1000, 1000, 1000}; // Fist
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // Open hand

  useconds_t delay = 1000 * 1000; // 1000 ms
  stark_set_finger_positions(handle, slave_id_1, positions_fist, 6);
  stark_set_finger_positions(handle, slave_id_2, positions_fist, 6);
  usleep(delay);
  stark_set_finger_positions(handle, slave_id_1, positions_open, 6);
  stark_set_finger_positions(handle, slave_id_2, positions_open, 6);
  usleep(delay);

  auto finger_status = stark_get_motor_status(handle, slave_id_1);
  printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n", finger_status->positions[0], finger_status->positions[1], finger_status->positions[2], finger_status->positions[3], finger_status->positions[4], finger_status->positions[5]);
  printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0], finger_status->speeds[1], finger_status->speeds[2], finger_status->speeds[3], finger_status->speeds[4], finger_status->speeds[5]);
  printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->currents[0], finger_status->currents[1], finger_status->currents[2], finger_status->currents[3], finger_status->currents[4], finger_status->currents[5]);
  printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n", finger_status->states[0], finger_status->states[1], finger_status->states[2], finger_status->states[3], finger_status->states[4], finger_status->states[5]);
  free_motor_status_data(finger_status);

  // Loop to control finger positions, for testing only; a very short delay may affect motor lifespan
  while (0)
  {
    printf("set_positions loop start\n");
    stark_set_finger_positions(handle, slave_id_1, positions_fist, 6);
    stark_set_finger_positions(handle, slave_id_2, positions_fist, 6);
    usleep(delay);
    stark_set_finger_positions(handle, slave_id_1, positions_open, 6);
    stark_set_finger_positions(handle, slave_id_2, positions_open, 6);
    usleep(delay);
  }

  return 0;
}

// Get device serial number, firmware version and other information
void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  auto info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", slave_id, (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH)
    {
      // Enable all touch sensors
      stark_enable_touch_sensor(handle, slave_id, 0x1F);
      usleep(1000 * 1000); // wait for touch sensor to be ready
    } else {
      printf("Not Revo2 Touch, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  } else
  {
    printf("Error: Failed to get device info\n");
  }
}

// Get device information: serial baudrate, slave address, voltage, LED info, button events
void get_info(DeviceHandler *handle, uint8_t slave_id)
{
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);
  auto led_info = stark_get_led_info(handle, slave_id);
  printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode, led_info->color);
  free_led_info(led_info);
}
