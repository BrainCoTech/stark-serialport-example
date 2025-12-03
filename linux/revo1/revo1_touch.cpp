#include <stdio.h>
#include "stark-sdk.h"
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>

// Function declarations
void get_touch_status(DeviceHandler *handle, uint8_t slave_id);
void get_device_info(DeviceHandler *handleint, uint8_t slave_id);
void get_info(DeviceHandler *handle, uint8_t slave_id);

void handler(int sig)
{
  void *array[10];
  size_t size;

  // Get stack frames
  size = backtrace(array, 10);

  // Print all stack frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char const *argv[])
{
  signal(SIGSEGV, handler); // Install our handler for SIGSEGV (segmentation fault)
  signal(SIGABRT, handler); // Install our handler for SIGABRT (abort signal)

  auto cfg = auto_detect_modbus_revo1(NULL, true);
  if (cfg == NULL)
  {
    fprintf(stderr, "Failed to auto-detect Modbus device configuration.\n");
    return -1;
  }

  auto handle = modbus_open(cfg->port_name, cfg->baudrate);
  uint8_t slave_id = cfg->slave_id;
  get_device_info(handle, slave_id);
  if (cfg != NULL) free_device_config(cfg);

  uint16_t positions_fist[] = {500, 500, 1000, 1000, 1000, 1000}; // Fist
  uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};           // Open hand

  useconds_t delay = 1000 * 1000; // 1000ms
  // stark_set_finger_position(handle, slave_id, STARK_FINGER_ID_PINKY, 100);
  // usleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_fist, 6);
  usleep(delay);
  stark_set_finger_positions(handle, slave_id, positions_open, 6);
  usleep(delay);

  get_touch_status(handle, slave_id);

  // Loop to control finger positions, for testing only; too short delay may affect motor lifespan
  while (0)
  {
    printf("set_positions loop start\n");
    stark_set_finger_positions(handle, slave_id, positions_fist, 6);
    usleep(delay);
    get_touch_status(handle, slave_id); // Call get_touch_status
    stark_set_finger_positions(handle, slave_id, positions_open, 6);
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
    if (info->hardware_type == STARK_HARDWARE_TYPE_REVO1_TOUCH || info->hardware_type == STARK_HARDWARE_TYPE_REVO2_TOUCH)
    {
      // Enable all touch sensors
      // Enable all touch sensors
      stark_enable_touch_sensor(handle, slave_id, 0x1F);
      usleep(1000 * 1000); // wait for touch sensor to be ready
    } else {
      printf("Not Revo1 Touch, hardware type: %hhu\n", info->hardware_type);
      exit(1);
    }
    free_device_info(info);
  }
  else
  {
    printf("Error: Failed to get device info\n");
  }
}

void test_auto_calibration(DeviceHandler *handle, uint8_t slave_id)
{
  stark_set_auto_calibration(handle, slave_id, false); // Set whether to enable automatic calibration after power-up
  // Set whether to enable automatic calibration after power-on
  bool auto_calibration_enabled = stark_get_auto_calibration(handle, slave_id);
  printf("Slave[%hhu] Auto Calibration Enabled: %s\n", slave_id, auto_calibration_enabled ? "true" : "false");
}

// Get touch sensor status, 3D force values, self-proximity, mutual-proximity capacitance values, and sensor status
void get_touch_status(DeviceHandler *handle, uint8_t slave_id)
{
  auto status = stark_get_touch_status(handle, slave_id);
  if (status != NULL)
  {
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
  }
  else
  {
    printf("Error: Failed to get touch sensor status\n");
  }

  auto raw_data = stark_get_touch_raw_data(handle, slave_id);
  if (raw_data != NULL)
  {
    printf("Slave[%hhu] Touch Raw Data:\n", slave_id);
    printf("Thumb: %u, %u, %u, %u, %u, %u, %u\n", raw_data->thumb[0], raw_data->thumb[1], raw_data->thumb[2], raw_data->thumb[3], raw_data->thumb[4], raw_data->thumb[5], raw_data->thumb[6]);
    printf("Index: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", raw_data->index[0], raw_data->index[1], raw_data->index[2], raw_data->index[3], raw_data->index[4], raw_data->index[5], raw_data->index[6], raw_data->index[7], raw_data->index[8], raw_data->index[9], raw_data->index[10]);
    printf("Middle: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", raw_data->middle[0], raw_data->middle[1], raw_data->middle[2], raw_data->middle[3], raw_data->middle[4], raw_data->middle[5], raw_data->middle[6], raw_data->middle[7], raw_data->middle[8], raw_data->middle[9], raw_data->middle[10]);
    printf("Ring: %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", raw_data->ring[0], raw_data->ring[1], raw_data->ring[2], raw_data->ring[3], raw_data->ring[4], raw_data->ring[5], raw_data->ring[6], raw_data->ring[7], raw_data->ring[8], raw_data->ring[9], raw_data->ring[10]);
    printf("Pinky: %u, %u, %u, %u, %u, %u, %u\n", raw_data->pinky[0], raw_data->pinky[1], raw_data->pinky[2], raw_data->pinky[3], raw_data->pinky[4], raw_data->pinky[5], raw_data->pinky[6]);
    free_touch_raw_data(raw_data);
  }
}

// Get device information: serial baudrate, slave address, voltage, LED info, button events
void get_info(DeviceHandler *handle, uint8_t slave_id)
{
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);
  // Touch version deprecated
  // auto force_level = stark_get_force_level(handle, slave_id);
  // printf("Slave[%hhu] Force Level: %d\n", slave_id, force_level);
  auto voltage = stark_get_voltage(handle, slave_id);
  printf("Slave[%hhu] Voltage: %.2fV\n", slave_id, voltage / 1000.0);

  auto led_info = stark_get_led_info(handle, slave_id);
  if (led_info != NULL)
  {
    printf("Slave[%hhu] LED Info: %hhu, %hhu\n", slave_id, led_info->mode, led_info->color);
    free_led_info(led_info);
  }

  auto button_event = stark_get_button_event(handle, slave_id);
  if (button_event != NULL)
  {
    printf("Slave[%hhu] Button Event: %d, %d, %hhu\n", slave_id, button_event->timestamp, button_event->button_id, button_event->press_state);
    free_button_event(button_event);
  }
}
