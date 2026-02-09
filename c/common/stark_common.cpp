/**
 * @file stark_common.c
 * @brief Implementation of common utility functions for Stark SDK C examples
 */

#include "stark_common.h"
#include <stdio.h>
#include <stdlib.h>

// Platform-specific includes are handled by platform_compat.h

// Suppress clang-tidy warnings for this file
// #pragma clang diagnostic push
// #pragma clang diagnostic ignored "-Wunknown-pragmas"
// #pragma clang diagnostic ignored "-Wunused-parameter"

/****************************************************************************/
// Signal handling
/****************************************************************************/

static void signal_handler(int sig) {
  fprintf(stderr, "Error: signal %d:\n", sig);
  print_stack_trace();
  exit(1);
}

void setup_signal_handlers(void) {
  signal_compat(SIGSEGV, signal_handler); // Segmentation fault
  signal_compat(SIGABRT, signal_handler); // Abort signal
}

/****************************************************************************/
// Utility functions
/****************************************************************************/

void print_hex(const unsigned char *data, int len) {
  for (int i = 0; i < len; i++) {
    printf("%02X", data[i]);
    if (i < len - 1) {
      printf(" ");
    }
  }
  printf("\n");
}

/****************************************************************************/
// Device information functions
/****************************************************************************/

bool get_and_print_device_info(DeviceHandler *handle, uint8_t slave_id) {
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info != NULL) {
    printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
           info->serial_number, info->firmware_version);

    // Print hardware type
    const char *hw_type = "Unknown";
    switch (info->hardware_type) {
    case STARK_HARDWARE_TYPE_REVO1_BASIC:
      hw_type = "Revo1 Basic";
      break;
    case STARK_HARDWARE_TYPE_REVO1_TOUCH:
      hw_type = "Revo1 Touch";
      break;
    case STARK_HARDWARE_TYPE_REVO2_BASIC:
      hw_type = "Revo2 Basic";
      break;
    case STARK_HARDWARE_TYPE_REVO2_TOUCH:
      hw_type = "Revo2 Touch (Capacitive)";
      break;
    case STARK_HARDWARE_TYPE_REVO1_ADVANCED:
      hw_type = "Revo1 Advanced";
      break;
    case STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH:
      hw_type = "Revo1 Advanced Touch";
      break;
    case STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE:
      hw_type = "Revo2 Touch (Pressure/Modulus)";
      break;
    default:
      break;
    }
    printf("Hardware Type: %s (%hhu)\n", hw_type, info->hardware_type);

    free_device_info(info);
    return true;
  } else {
    printf("Error: Failed to get device info\n");
    return false;
  }
}

void get_and_print_extended_info(DeviceHandler *handle, uint8_t slave_id) {
  // Get baudrate
  auto baudrate = stark_get_rs485_baudrate(handle, slave_id);
  printf("Slave[%hhu] Baudrate: %d\n", slave_id, baudrate);

  // Get voltage
  auto voltage = stark_get_voltage(handle, slave_id);
  printf("Slave[%hhu] Voltage: %.2fV\n", slave_id, voltage / 1000.0);

  // Get LED info
  auto led_info = stark_get_led_info(handle, slave_id);
  if (led_info != NULL) {
    printf("Slave[%hhu] LED: color=%hhu, mode=%hhu\n", slave_id,
           led_info->color, led_info->mode);
    free_led_info(led_info);
  }
}

/****************************************************************************/
// Hardware type helper functions
/****************************************************************************/

StarkHardwareType get_device_hardware_type(DeviceHandler *handle, uint8_t slave_id) {
  CDeviceInfo *info = stark_get_device_info(handle, slave_id);
  if (info == NULL) {
    printf("Error: Failed to get device info\n");
    return STARK_HARDWARE_TYPE_REVO2_BASIC;  // Use as error indicator
  }

  // Print device info
  printf("Slave[%hhu] Serial Number: %s, FW: %s\n", slave_id,
         info->serial_number, info->firmware_version);
  printf("Hardware Type: %s (%hhu)\n",
         get_hardware_type_name_str(info->hardware_type), info->hardware_type);

  StarkHardwareType hw_type = (StarkHardwareType)info->hardware_type;
  free_device_info(info);
  return hw_type;
}

bool hw_uses_revo1_motor_api(StarkHardwareType hw_type) {
  return (hw_type == STARK_HARDWARE_TYPE_REVO1_PROTOBUF ||
          hw_type == STARK_HARDWARE_TYPE_REVO1_BASIC ||
          hw_type == STARK_HARDWARE_TYPE_REVO1_TOUCH);
}

bool hw_uses_revo2_motor_api(StarkHardwareType hw_type) {
  return !hw_uses_revo1_motor_api(hw_type);
}

bool hw_uses_revo1_touch_api(StarkHardwareType hw_type) {
  return (hw_type == STARK_HARDWARE_TYPE_REVO1_TOUCH ||
          hw_type == STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH);
}

bool hw_uses_revo2_touch_api(StarkHardwareType hw_type) {
  return (hw_type == STARK_HARDWARE_TYPE_REVO2_TOUCH ||
          hw_type == STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE);
}

bool hw_has_touch_sensor(StarkHardwareType hw_type) {
  return (hw_type == STARK_HARDWARE_TYPE_REVO1_TOUCH ||
          hw_type == STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH ||
          hw_type == STARK_HARDWARE_TYPE_REVO2_TOUCH ||
          hw_type == STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE);
}

// Legacy functions for backward compatibility
bool verify_device_is_revo1(DeviceHandler *handle, uint8_t slave_id) {
  StarkHardwareType hw_type = get_device_hardware_type(handle, slave_id);
  if (hw_type == STARK_HARDWARE_TYPE_REVO1_PROTOBUF) return false;  // Error case

  // Revo1 devices: Revo1 Basic, Touch
  bool is_revo1 = (
                   hw_type == STARK_HARDWARE_TYPE_REVO1_PROTOBUF ||
                   hw_type == STARK_HARDWARE_TYPE_REVO1_BASIC ||
                   hw_type == STARK_HARDWARE_TYPE_REVO1_TOUCH);
  if (!is_revo1) {
    printf("Error: Device is not Revo1\n");
  }
  return is_revo1;
}

bool verify_device_is_revo2(DeviceHandler *handle, uint8_t slave_id) {
  return !verify_device_is_revo1(handle, slave_id);
}

/****************************************************************************/
// Time utilities
/****************************************************************************/

int get_current_time_ms(void) {
  return (int)get_time_ms();
}

/****************************************************************************/
// Auto-detect and initialize device
/****************************************************************************/

#include <string.h>

const char* get_hardware_type_name_str(uint8_t hw_type) {
    switch (hw_type) {
        case STARK_HARDWARE_TYPE_REVO1_PROTOBUF: return "Revo1 ProtoBuf";
        case STARK_HARDWARE_TYPE_REVO1_BASIC: return "Revo1 Basic";
        case STARK_HARDWARE_TYPE_REVO1_TOUCH: return "Revo1 Touch";
        case STARK_HARDWARE_TYPE_REVO1_ADVANCED: return "Revo1 Advanced";
        case STARK_HARDWARE_TYPE_REVO1_ADVANCED_TOUCH: return "Revo1 Advanced Touch";
        case STARK_HARDWARE_TYPE_REVO2_BASIC: return "Revo2 Basic";
        case STARK_HARDWARE_TYPE_REVO2_TOUCH: return "Revo2 Touch (Capacitive)";
        case STARK_HARDWARE_TYPE_REVO2_TOUCH_PRESSURE: return "Revo2 Touch (Pressure)";
        default: return "Unknown";
    }
}

const char* get_touch_type_name_str(TouchSensorType touch_type) {
    switch (touch_type) {
        case TOUCH_TYPE_NONE: return "None";
        case TOUCH_TYPE_CAPACITIVE: return "Capacitive";
        case TOUCH_TYPE_PRESSURE: return "Pressure (Modulus)";
        default: return "Unknown";
    }
}

/**
 * Get touch sensor type from hardware type
 */
TouchSensorType get_touch_sensor_type(StarkHardwareType hw_type) {
    return (TouchSensorType)stark_get_touch_sensor_type(hw_type);
}

const char* get_protocol_name_str(uint8_t protocol) {
    switch (protocol) {
        case STARK_PROTOCOL_TYPE_MODBUS: return "Modbus";
        case STARK_PROTOCOL_TYPE_CAN: return "CAN 2.0";
        case STARK_PROTOCOL_TYPE_CAN_FD: return "CANFD";
        default: return "Unknown";
    }
}

bool auto_detect_and_init(DeviceContext* ctx, bool require_touch) {
    printf("[INFO] Auto-detecting devices...\n");

    CDetectedDeviceList* device_list = stark_auto_detect(
        true,   // scan_all: find all devices
        NULL,   // port: scan all ports
        STARK_PROTOCOL_TYPE_AUTO  // protocol: auto-detect all protocols
    );

    if (device_list == NULL || device_list->count == 0) {
        printf("[ERROR] No devices found\n");
        if (device_list) free_detected_device_list(device_list);
        return false;
    }

    // Count and display matching devices
    size_t match_count = 0;

    printf("\n[INFO] Found devices:\n");
    for (size_t i = 0; i < device_list->count; i++) {
        CDetectedDevice* device = &device_list->devices[i];

        // Filter by touch requirement
        if (require_touch && !stark_is_touch_device(device->hardware_type)) {
            continue;
        }

        match_count++;
        printf("\n[%zu] %s\n", match_count, get_hardware_type_name_str(device->hardware_type));
        printf("    Protocol: %s\n", get_protocol_name_str(device->protocol));
        printf("    Port: %s\n", device->port_name);
        printf("    Slave ID: 0x%02X (%d)\n", device->slave_id, device->slave_id);
        if (device->serial_number) {
            printf("    Serial Number: %s\n", device->serial_number);
        }
        if (device->firmware_version) {
            printf("    Firmware: %s\n", device->firmware_version);
        }
    }

    if (match_count == 0) {
        printf("[ERROR] No %sdevices found\n", require_touch ? "touch sensor " : "");
        free_detected_device_list(device_list);
        return false;
    }

    // Select device
    size_t selected_idx = 0;
    if (match_count > 1) {
        printf("\nSelect device [1-%zu]: ", match_count);
        if (scanf("%zu", &selected_idx) != 1 || selected_idx < 1 || selected_idx > match_count) {
            printf("[ERROR] Invalid selection\n");
            free_detected_device_list(device_list);
            return false;
        }
    } else {
        selected_idx = 1;
        printf("\n[INFO] Using the only available device\n");
    }

    // Find the selected device
    size_t current_idx = 0;
    CDetectedDevice* selected_device = NULL;

    for (size_t i = 0; i < device_list->count; i++) {
        CDetectedDevice* device = &device_list->devices[i];

        if (require_touch && !stark_is_touch_device(device->hardware_type)) {
            continue;
        }

        current_idx++;
        if (current_idx == selected_idx) {
            selected_device = device;
            break;
        }
    }

    if (selected_device == NULL) {
        printf("[ERROR] Failed to find selected device\n");
        free_detected_device_list(device_list);
        return false;
    }

    // Initialize using SDK API
    ctx->handle = init_from_detected(selected_device);

    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to initialize device\n");
        free_detected_device_list(device_list);
        return false;
    }

    // Copy device info
    ctx->slave_id = selected_device->slave_id;
    ctx->hw_type = (StarkHardwareType)selected_device->hardware_type;

    // Copy serial number for touch type detection
    if (selected_device->serial_number) {
        strncpy(ctx->serial_number, selected_device->serial_number, sizeof(ctx->serial_number) - 1);
        ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
    } else {
        ctx->serial_number[0] = '\0';
    }

    // Get protocol type from device handler
    StarkProtocolType protocol = stark_get_protocol_type(ctx->handle);

    free_detected_device_list(device_list);

    // Determine optimal frequencies based on protocol and platform
    #ifdef __linux__
        switch (protocol) {
            case STARK_PROTOCOL_TYPE_MODBUS:
                ctx->motor_freq = 100;
                ctx->touch_freq = 50;
                break;
            case STARK_PROTOCOL_TYPE_CAN:
                ctx->motor_freq = 50;
                ctx->touch_freq = 20;
                break;
            case STARK_PROTOCOL_TYPE_CAN_FD:
                ctx->motor_freq = 100;
                ctx->touch_freq = 50;
                break;
            default:
                ctx->motor_freq = 50;
                ctx->touch_freq = 20;
                break;
        }
    #else
        switch (protocol) {
            case STARK_PROTOCOL_TYPE_MODBUS:
                ctx->motor_freq = 20;
                ctx->touch_freq = 10;
                break;
            case STARK_PROTOCOL_TYPE_CAN:
                ctx->motor_freq = 20;
                ctx->touch_freq = 10;
                break;
            case STARK_PROTOCOL_TYPE_CAN_FD:
                ctx->motor_freq = 50;
                ctx->touch_freq = 20;
                break;
            default:
                ctx->motor_freq = 20;
                ctx->touch_freq = 10;
                break;
        }
    #endif

    printf("[INFO] Device: %s\n", get_hardware_type_name_str(ctx->hw_type));
    printf("[INFO] Touch: %s\n", get_touch_type_name_str(get_touch_sensor_type(ctx->hw_type)));
    printf("[INFO] Protocol: %s, Motor: %dHz, Touch: %dHz\n",
           get_protocol_name_str(protocol), ctx->motor_freq, ctx->touch_freq);

    return true;
}

void cleanup_device_context(DeviceContext* ctx) {
    if (ctx == NULL || ctx->handle == NULL) {
        return;
    }

    // Use SDK's unified close function
    StarkProtocolType protocol = stark_get_protocol_type(ctx->handle);
    close_device_handler(ctx->handle, protocol);
    ctx->handle = NULL;
    printf("[INFO] Device connection closed\n");
}

/****************************************************************************/
// Manual initialization functions
/****************************************************************************/

bool init_modbus(DeviceContext* ctx, const char* port, uint32_t baudrate, uint8_t slave_id) {
    printf("\n[Init] Mode: Modbus\n");
    printf("  Port: %s, Baudrate: %u, Slave ID: %d\n", port, baudrate, slave_id);
    
    ctx->handle = modbus_open(port, baudrate);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to open Modbus port\n");
        return false;
    }
    
    ctx->slave_id = slave_id;
    
    // Print override info if set
    if (ctx->hw_type_override != 0) {
        printf("  Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx->hw_type_override), ctx->hw_type_override);
        ctx->hw_type = ctx->hw_type_override;
    }
    
    // Always get device info for serial number and firmware version
    CDeviceInfo* info = stark_get_device_info(ctx->handle, slave_id);
    if (info != NULL) {
        if (ctx->hw_type_override == 0) {
            ctx->hw_type = (StarkHardwareType)info->hardware_type;
        }
        printf("  Device: %s, SN: %s, FW: %s\n", 
               get_hardware_type_name_str(info->hardware_type),
               info->serial_number, info->firmware_version);
        
        // Copy serial number
        strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
        ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
        
        free_device_info(info);
    }
    
    #ifdef __linux__
        ctx->motor_freq = 100;
        ctx->touch_freq = 50;
    #else
        ctx->motor_freq = 20;
        ctx->touch_freq = 10;
    #endif
    return true;
}

bool init_protobuf(DeviceContext* ctx, const char* port, uint8_t slave_id) {
    printf("\n[Init] Mode: Protobuf\n");
    printf("  Port: %s, Slave ID: %d (baudrate: 115200)\n", port, slave_id);
    
    // Validate slave_id range for Protobuf (10-254)
    if (slave_id < 10 || slave_id > 254) {
        printf("[ERROR] Protobuf slave_id must be 10-254, got %d\n", slave_id);
        return false;
    }
    
    ctx->handle = protobuf_open(port, slave_id, 0);
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to open Protobuf port\n");
        return false;
    }
    
    ctx->slave_id = slave_id;
    ctx->hw_type = STARK_HARDWARE_TYPE_REVO1_PROTOBUF;
    
    // Try to get device info (optional for Protobuf)
    CDeviceInfo* info = stark_get_device_info(ctx->handle, slave_id);
    if (info != NULL) {
        printf("  Device: %s, SN: %s, FW: %s\n", 
               get_hardware_type_name_str(info->hardware_type),
               info->serial_number, info->firmware_version);
        if (info->serial_number) {
            strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
            ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
        }
        free_device_info(info);
    }
    
    // Protobuf has lower frequency due to serial protocol overhead
    ctx->motor_freq = 50;
    ctx->touch_freq = 20;
    return true;
}

bool init_zqwl_device(DeviceContext* ctx, const char* port, uint32_t arb_baudrate, 
                      uint32_t data_baudrate, uint8_t slave_id, bool is_canfd) {
    printf("\n[Init] Mode: ZQWL %s\n", is_canfd ? "CANFD" : "CAN 2.0");
    if (is_canfd) {
        printf("  Port: %s, Arb: %u, Data: %u, Slave ID: %d\n", port, arb_baudrate, data_baudrate, slave_id);
    } else {
        printf("  Port: %s, Baudrate: %u, Slave ID: %d\n", port, arb_baudrate, slave_id);
    }
    
    int result = is_canfd ? init_zqwl_canfd(port, arb_baudrate, data_baudrate) 
                          : init_zqwl_can(port, arb_baudrate);
    if (result != 0) {
        printf("[ERROR] Failed to initialize ZQWL %s adapter\n", is_canfd ? "CANFD" : "CAN");
        return false;
    }
    
    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    
    // Use hw_type override if set
    if (ctx->hw_type_override != 0) {
        printf("  Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx->hw_type_override), ctx->hw_type_override);
        ctx->handle = init_device_handler_can_with_hw_type(protocol, 1, slave_id, arb_baudrate, data_baudrate, ctx->hw_type_override);
        ctx->hw_type = ctx->hw_type_override;
    } else {
        ctx->handle = init_device_handler_can(protocol, 1, arb_baudrate, data_baudrate);
    }
    
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        close_zqwl();
        return false;
    }
    
    ctx->slave_id = slave_id;
    
    // Always get device info for serial number and firmware version
    CDeviceInfo* info = stark_get_device_info(ctx->handle, slave_id);
    if (info != NULL) {
        if (ctx->hw_type_override == 0) {
            ctx->hw_type = (StarkHardwareType)info->hardware_type;
        }
        printf("  Device: %s, SN: %s, FW: %s\n", 
               get_hardware_type_name_str(info->hardware_type),
               info->serial_number, info->firmware_version);
        
        // Copy serial number
        if (info->serial_number) {
            strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
            ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
        }
        
        free_device_info(info);
    }
    
    ctx->motor_freq = is_canfd ? 50 : 20;
    ctx->touch_freq = is_canfd ? 20 : 10;
    return true;
}

#ifdef __linux__
#include "can_common.h"

bool init_socketcan_device(DeviceContext* ctx, const char* iface, uint8_t slave_id, bool is_canfd) {
    printf("\n[Init] Mode: SocketCAN %s\n", is_canfd ? "(CANFD)" : "(CAN 2.0)");
    printf("  Interface: %s, Slave ID: %d\n", iface, slave_id);
    
    setenv("STARK_CAN_BACKEND", "socketcan", 1);
    setenv("STARK_SOCKETCAN_IFACE", iface, 1);
    
    bool success = is_canfd ? setup_canfd() : setup_can();
    if (!success) {
        printf("[ERROR] Failed to initialize SocketCAN\n");
        printf("[TIP] Make sure the interface is up:\n");
        printf("      sudo ip link set %s type can bitrate 1000000\n", iface);
        printf("      sudo ip link set %s up\n", iface);
        return false;
    }
    
    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    
    // SocketCAN baudrates are configured at OS level, but we store typical values for query APIs
    uint32_t arb_baudrate = 1000000;  // 1 Mbps (typical for both CAN 2.0 and CANFD)
    uint32_t data_baudrate = is_canfd ? 5000000 : 1000000;  // 5 Mbps for CANFD, 1 Mbps for CAN 2.0
    
    // Use hw_type override if set
    if (ctx->hw_type_override != 0) {
        printf("  Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx->hw_type_override), ctx->hw_type_override);
        ctx->handle = init_device_handler_can_with_hw_type(protocol, 1, slave_id, arb_baudrate, data_baudrate, ctx->hw_type_override);
        ctx->hw_type = ctx->hw_type_override;
    } else {
        ctx->handle = init_device_handler_can(protocol, 1, arb_baudrate, data_baudrate);
    }
    
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        cleanup_can_resources();
        return false;
    }
    
    ctx->slave_id = slave_id;
    
    // Get device info if no override
    if (ctx->hw_type_override == 0) {
        CDeviceInfo* info = stark_get_device_info(ctx->handle, slave_id);
        if (info != NULL) {
            ctx->hw_type = (StarkHardwareType)info->hardware_type;
            
            // Copy serial number
            if (info->serial_number) {
                strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
                ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
            }
            
            free_device_info(info);
        }
    }
    
    ctx->motor_freq = is_canfd ? 100 : 50;
    ctx->touch_freq = is_canfd ? 50 : 20;
    return true;
}

bool init_socketcan_device_builtin(DeviceContext* ctx, const char* iface, uint8_t slave_id, bool is_canfd) {
    const char* iface_name = iface ? iface : "can0";
    printf("\n[Init] Mode: SocketCAN %s (SDK built-in)\n", is_canfd ? "(CANFD)" : "(CAN 2.0)");
    printf("  Interface: %s, Slave ID: %d\n", iface_name, slave_id);
    
    // Use SDK built-in SocketCAN API
    int result = is_canfd ? init_socketcan_canfd(iface_name) : init_socketcan_can(iface_name);
    if (result != 0) {
        printf("[ERROR] Failed to initialize SocketCAN (SDK built-in)\n");
        printf("[TIP] Make sure the interface is up:\n");
        printf("      sudo ip link set %s type can bitrate 1000000\n", iface_name);
        if (is_canfd) {
            printf("      sudo ip link set %s type can dbitrate 5000000 fd on\n", iface_name);
        }
        printf("      sudo ip link set %s up\n", iface_name);
        return false;
    }
    
    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    
    // SocketCAN baudrates are configured at OS level, but we store typical values for query APIs
    uint32_t arb_baudrate = 1000000;  // 1 Mbps (typical for both CAN 2.0 and CANFD)
    uint32_t data_baudrate = is_canfd ? 5000000 : 1000000;  // 5 Mbps for CANFD, 1 Mbps for CAN 2.0
    
    // Use hw_type override if set
    if (ctx->hw_type_override != 0) {
        printf("  Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx->hw_type_override), ctx->hw_type_override);
        ctx->handle = init_device_handler_can_with_hw_type(protocol, 1, slave_id, arb_baudrate, data_baudrate, ctx->hw_type_override);
        ctx->hw_type = ctx->hw_type_override;
    } else {
        ctx->handle = init_device_handler_can(protocol, 1, arb_baudrate, data_baudrate);
    }
    
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        close_socketcan();
        return false;
    }
    
    ctx->slave_id = slave_id;
    
    // Get device info if no override
    if (ctx->hw_type_override == 0) {
        CDeviceInfo* info = stark_get_device_info(ctx->handle, slave_id);
        if (info != NULL) {
            ctx->hw_type = (StarkHardwareType)info->hardware_type;
            
            // Copy serial number
            if (info->serial_number) {
                strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
                ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
            }
            
            free_device_info(info);
        }
    }
    
    ctx->motor_freq = is_canfd ? 100 : 50;
    ctx->touch_freq = is_canfd ? 50 : 20;
    return true;
}

bool init_zlg_device(DeviceContext* ctx, uint8_t slave_id, bool is_canfd) {
    printf("\n[Init] Mode: ZLG USB-CANFD %s\n", is_canfd ? "(CANFD)" : "(CAN 2.0)");
    printf("  Slave ID: %d\n", slave_id);
    
    bool success = is_canfd ? setup_canfd() : setup_can();
    if (!success) {
        printf("[ERROR] Failed to initialize ZLG CAN device\n");
        printf("[TIP] Make sure ZLG USB-CANFD is connected and libusbcanfd.so is installed\n");
        return false;
    }
    
    StarkProtocolType protocol = is_canfd ? STARK_PROTOCOL_TYPE_CAN_FD : STARK_PROTOCOL_TYPE_CAN;
    
    // ZLG baudrates are configured internally by setup_can/setup_canfd, store typical values for query APIs
    uint32_t arb_baudrate = 1000000;  // 1 Mbps (typical for both CAN 2.0 and CANFD)
    uint32_t data_baudrate = is_canfd ? 5000000 : 1000000;  // 5 Mbps for CANFD, 1 Mbps for CAN 2.0
    
    // Use hw_type override if set
    if (ctx->hw_type_override != 0) {
        printf("  Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx->hw_type_override), ctx->hw_type_override);
        ctx->handle = init_device_handler_can_with_hw_type(protocol, 1, slave_id, arb_baudrate, data_baudrate, ctx->hw_type_override);
        ctx->hw_type = ctx->hw_type_override;
    } else {
        ctx->handle = init_device_handler_can(protocol, 1, arb_baudrate, data_baudrate);
    }
    
    if (ctx->handle == NULL) {
        printf("[ERROR] Failed to create device handler\n");
        cleanup_can_resources();
        return false;
    }
    
    ctx->slave_id = slave_id;
    
    // Get device info if no override
    if (ctx->hw_type_override == 0) {
        CDeviceInfo* info = stark_get_device_info(ctx->handle, slave_id);
        if (info != NULL) {
            ctx->hw_type = (StarkHardwareType)info->hardware_type;
            
            // Copy serial number
            if (info->serial_number) {
                strncpy(ctx->serial_number, info->serial_number, sizeof(ctx->serial_number) - 1);
                ctx->serial_number[sizeof(ctx->serial_number) - 1] = '\0';
            }
            
            free_device_info(info);
        }
    }
    
    ctx->motor_freq = is_canfd ? 100 : 50;
    ctx->touch_freq = is_canfd ? 50 : 20;
    return true;
}
#endif

void print_init_usage(const char* prog_name) {
    printf("Initialization options:\n");
    printf("  (default)                    Auto-detect device and protocol\n");
    printf("  -m <port> <baud> <id>        Modbus\n");
    printf("  -p <port> [id]               Protobuf (115200 baud, default id=10)\n");
    printf("  -c <port> <baud> <id>        CAN 2.0 (ZQWL)\n");
    printf("  -f <port> <arb> <data> <id>  CANFD (ZQWL)\n");
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
}

bool parse_args_and_init(DeviceContext* ctx, int argc, const char* argv[], int* arg_idx) {
    *arg_idx = 1;
    
    // Initialize hw_type_override to 0 (auto-detect)
    ctx->hw_type_override = (StarkHardwareType)0;
    
    // Check for -t option first (hardware type override)
    if (argc > 2 && argv[1][0] == '-' && argv[1][1] == 't') {
        int hw_type_val = atoi(argv[2]);
        if (hw_type_val < 0 || hw_type_val > 7) {
            printf("[ERROR] Invalid hardware type: %d (valid: 0-7)\n", hw_type_val);
            return false;
        }
        ctx->hw_type_override = (StarkHardwareType)hw_type_val;
        printf("[INFO] Hardware type override: %s (%d)\n", 
               get_hardware_type_name_str(ctx->hw_type_override), ctx->hw_type_override);
        
        // Shift arguments
        argc -= 2;
        argv += 2;
        *arg_idx = 3;
    }
    
    if (argc > 1 && argv[1][0] == '-') {
        char opt = argv[1][1];
        
        switch (opt) {
            case 'm': // Modbus: -m <port> <baud> <slave_id>
                if (argc < 5) {
                    printf("[ERROR] -m requires: <port> <baudrate> <slave_id>\n");
                    return false;
                }
                if (!init_modbus(ctx, argv[2], atoi(argv[3]), atoi(argv[4]))) {
                    return false;
                }
                *arg_idx += 4;
                break;
            
            case 'p': // Protobuf: -p <port> [slave_id]
                if (argc < 3) {
                    printf("[ERROR] -p requires: <port> [slave_id]\n");
                    return false;
                }
                {
                    uint8_t proto_slave_id = 10;  // Default slave_id for Protobuf
                    if (argc >= 4 && argv[3][0] != '-') {
                        proto_slave_id = atoi(argv[3]);
                        *arg_idx += 3;
                    } else {
                        *arg_idx += 2;
                    }
                    if (!init_protobuf(ctx, argv[2], proto_slave_id)) {
                        return false;
                    }
                }
                break;
                
            case 'c': // CAN 2.0: -c <port> <baud> <slave_id>
                if (argc < 5) {
                    printf("[ERROR] -c requires: <port> <baudrate> <slave_id>\n");
                    return false;
                }
                if (!init_zqwl_device(ctx, argv[2], atoi(argv[3]), 0, atoi(argv[4]), false)) {
                    return false;
                }
                *arg_idx += 4;
                break;
                
            case 'f': // CANFD: -f <port> <arb_baud> <data_baud> <slave_id>
                if (argc < 6) {
                    printf("[ERROR] -f requires: <port> <arb_baudrate> <data_baudrate> <slave_id>\n");
                    return false;
                }
                if (!init_zqwl_device(ctx, argv[2], atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), true)) {
                    return false;
                }
                *arg_idx += 5;
                break;

#ifdef __linux__
            case 's': // SocketCAN CAN 2.0: -s <iface> <slave_id>
                if (argc < 4) {
                    printf("[ERROR] -s requires: <interface> <slave_id>\n");
                    return false;
                }
                if (!init_socketcan_device(ctx, argv[2], atoi(argv[3]), false)) {
                    return false;
                }
                *arg_idx += 3;
                break;
                
            case 'S': // SocketCAN CANFD: -S <iface> <slave_id>
                if (argc < 4) {
                    printf("[ERROR] -S requires: <interface> <slave_id>\n");
                    return false;
                }
                if (!init_socketcan_device(ctx, argv[2], atoi(argv[3]), true)) {
                    return false;
                }
                *arg_idx += 3;
                break;

            case 'b': // SocketCAN CAN 2.0 (SDK built-in): -b <iface> <slave_id>
                if (argc < 4) {
                    printf("[ERROR] -b requires: <interface> <slave_id>\n");
                    return false;
                }
                if (!init_socketcan_device_builtin(ctx, argv[2], atoi(argv[3]), false)) {
                    return false;
                }
                *arg_idx += 3;
                break;
                
            case 'B': // SocketCAN CANFD (SDK built-in): -B <iface> <slave_id>
                if (argc < 4) {
                    printf("[ERROR] -B requires: <interface> <slave_id>\n");
                    return false;
                }
                if (!init_socketcan_device_builtin(ctx, argv[2], atoi(argv[3]), true)) {
                    return false;
                }
                *arg_idx += 3;
                break;

            case 'z': // ZLG CAN 2.0: -z <slave_id>
                if (argc < 3) {
                    printf("[ERROR] -z requires: <slave_id>\n");
                    return false;
                }
                if (!init_zlg_device(ctx, atoi(argv[2]), false)) {
                    return false;
                }
                *arg_idx += 2;
                break;
                
            case 'Z': // ZLG CANFD: -Z <slave_id>
                if (argc < 3) {
                    printf("[ERROR] -Z requires: <slave_id>\n");
                    return false;
                }
                if (!init_zlg_device(ctx, atoi(argv[2]), true)) {
                    return false;
                }
                *arg_idx += 2;
                break;
#endif
                
            case 'h': // Help
                return false;
                
            default:
                printf("[ERROR] Unknown option: %s\n", argv[1]);
                return false;
        }
    } else {
        // Default: auto-detect
        if (!auto_detect_and_init(ctx, false)) {
            return false;
        }
    }
    
    return true;
}
