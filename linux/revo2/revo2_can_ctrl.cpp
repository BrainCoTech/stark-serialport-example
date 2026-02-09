// This example demonstrates basic control of Revo2 with a ZLG USB-CAN FD
// device. USBCANFD-200U USBCANFD-100U USBCANFD-100U-mini You need to download
// the vendor-provided .so https://manual.zlg.cn/web/#/146
#include "can_common.h"
#include "stark-sdk.h"
#include "stark_common.h"
#include <cstring>
#include <execinfo.h>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

// ================== Function declarations ==================
void cleanup_resources();
void get_device_info(DeviceHandler *handle, uint8_t slave_id);

// ================== Main function ==================

int main(int argc, char const *argv[])
{
  // Setup signal handlers for crash debugging
  setup_signal_handlers();

  // Setup CAN (device + channel + callbacks)
  if (!setup_can())
  {
    return -1;
  }

  // Initialize STARK SDK
  init_logging(LOG_LEVEL_INFO);
  auto handle = init_device_handler(STARK_PROTOCOL_TYPE_CAN, 0);
  // Revo2 CAN slave ID: 1 = left hand, 2 = right hand
  // uint8_t slave_id = 1;
  uint8_t slave_id = 2;
  get_device_info(handle, slave_id);

  // Loop through positions from 0 to 1000, incrementing by 50 each time
  // with 5ms interval between each position
  useconds_t delay = 5 * 1000; // 5ms in microseconds

  struct timeval last_time, current_time;
  gettimeofday(&last_time, NULL);

  while (true)
  {
    for (uint16_t pos = 0; pos <= 1000; pos += 50)
    {
      gettimeofday(&current_time, NULL);
      long interval_us = (current_time.tv_sec - last_time.tv_sec) * 1000000 + 
                         (current_time.tv_usec - last_time.tv_usec);
      double interval_ms = interval_us / 1000.0;
      
      uint16_t thumb_pos = pos * 2 / 5;
      uint16_t positions[6] = {thumb_pos, thumb_pos, pos, pos, pos, pos}; // Set all fingers to same position
      printf("positions: %hu, %hu, %hu, %hu, %hu, %hu, interval: %.2f ms\n", 
             positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], interval_ms);
      stark_set_finger_positions(handle, slave_id, positions, 6);
      last_time = current_time;
      usleep(delay);
    }

    for (uint16_t pos = 1000; pos > 0; pos -= 50)
    {
      gettimeofday(&current_time, NULL);
      long interval_us = (current_time.tv_sec - last_time.tv_sec) * 1000000 + 
                         (current_time.tv_usec - last_time.tv_usec);
      double interval_ms = interval_us / 1000.0;
      
      uint16_t thumb_pos = pos * 2 / 5;
      uint16_t positions[6] = {thumb_pos, thumb_pos, pos, pos, pos, pos}; // Set all fingers to same position
      printf("positions: %hu, %hu, %hu, %hu, %hu, %hu, interval: %.2f ms\n", 
             positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], interval_ms);
      stark_set_finger_positions(handle, slave_id, positions, 6);
      last_time = current_time;
      usleep(delay);
    }
  }

  auto finger_id = STARK_FINGER_ID_MIDDLE;
  bool running = true;
  while (running)
  {
    uint16_t durations[6] = {300, 300, 300, 300, 300, 300};
    uint16_t positions_fist_1000[] = {500, 500, 1000, 1000, 1000, 1000}; // Fist
    uint16_t positions_open[] = {0, 0, 0, 0, 0, 0};                      // Open hand
    useconds_t delay = 1000 * 1000;                                      // 1000 ms
    stark_set_finger_positions_and_durations(handle, slave_id,
                                             positions_fist_1000, durations, 6);
    usleep(delay);
    stark_set_finger_positions_and_durations(handle, slave_id, positions_open,
                                             durations, 6);
    usleep(delay);
    stark_set_finger_position_with_millis(handle, slave_id, finger_id, 1000,
                                          300);
    usleep(delay);
    stark_set_finger_position_with_millis(handle, slave_id, finger_id, 0, 300);
    usleep(delay);
    // running = false;

    auto finger_status = stark_get_motor_status(handle, slave_id);
    if (finger_status != NULL)
    {
      printf("Positions: %hu, %hu, %hu, %hu, %hu, %hu\n",
             finger_status->positions[0], finger_status->positions[1],
             finger_status->positions[2], finger_status->positions[3],
             finger_status->positions[4], finger_status->positions[5]);
      printf("Speeds: %hd, %hd, %hd, %hd, %hd, %hd\n", finger_status->speeds[0],
             finger_status->speeds[1], finger_status->speeds[2],
             finger_status->speeds[3], finger_status->speeds[4],
             finger_status->speeds[5]);
      printf("Currents: %hd, %hd, %hd, %hd, %hd, %hd\n",
             finger_status->currents[0], finger_status->currents[1],
             finger_status->currents[2], finger_status->currents[3],
             finger_status->currents[4], finger_status->currents[5]);
      printf("States: %hhu, %hhu, %hhu, %hhu, %hhu, %hhu\n",
             finger_status->states[0], finger_status->states[1],
             finger_status->states[2], finger_status->states[3],
             finger_status->states[4], finger_status->states[5]);
      free_motor_status_data(finger_status);
    }
  }

  // Clean up resources
  cleanup_resources();
  return 0;
}

void cleanup_resources()
{
  cleanup_can_resources(); // Use common cleanup function
  printf("Resources cleaned up.\n");
}

void get_device_info(DeviceHandler *handle, uint8_t slave_id)
{
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL)
  {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
           info->serial_number, info->firmware_version);
    // Check if device uses Revo2 motor API
    if (stark_uses_revo1_motor_api(info->hardware_type))
    {
      printf("Device uses Revo1 motor API, hardware type: %hhu\n", info->hardware_type);
      free_device_info(info);
      exit(1);
    }
    free_device_info(info);
  }
  else
  {
    printf("Error: Failed to get device info\n");
    exit(1);
  }
}
