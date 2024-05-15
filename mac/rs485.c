#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libserialport.h>
#include <stark_sdk.h>

#ifdef __APPLE__
#define SERIAL_PORT "/dev/tty.usbserial-14220"
#elif __linux__
// #define SERIAL_PORT "/dev/ttyCH341USB0"
#define SERIAL_PORT "/dev/ttyUSB0"
#endif

#define BAUD_RATE 115200

/* The ports we will use. */
struct sp_port *port = NULL;
struct sp_event_set *rx_event_set = NULL;

/* Helper function for error handling. */
int check(enum sp_return result);
void printData(const uint8_t *data, size_t size, bool format);

// 发送数据到 RS485
int send_data(const char* device_id, const uint8_t *data, int size) {
    int result;

	/* Send data. */
	printf("Sending '%s' (%d bytes).\n", data, size);
    printData(data, size, true);

    if (port == NULL) {
        printf("send_data, while Port is NULL.\n");
        return -1;
    }

    /* We'll allow a 1 second timeout for send. */
	unsigned int timeout = 1000;
    result = check(sp_blocking_write(port, data, size, timeout));

	/* Check whether we sent all of the data. */
	if (result == size)
		printf("Sent %d bytes successfully.\n", size);
	else
		printf("Timed out, %d/%d bytes sent.\n", result, size);

    return 0;
}

// 从 RS485 接收数据
void receive_data(StarkDevice* device) {
    if (port == NULL) {
        printf("receive_data, while Port is NULL.\n");
        return;
    }

    usleep(100000); // wait 100ms for return data

    /* Now we can call sp_wait() to await any event in the set.
    * It will return when an event occurs, or the timeout elapses. */
    printf("Waiting up to 1 seconds for RX on any port...\n");
    check(sp_wait(rx_event_set, 1000));
    struct sp_port *rx_port = port;
    int bytes_waiting = check(sp_input_waiting(rx_port));
    if (bytes_waiting <= 0) {
        printf("No data waiting, result:%d.\n", bytes_waiting);
        return;
    }

    printf("Receiving %d bytes data.\n", bytes_waiting);

    /* Allocate a buffer to receive data. */
    int size = bytes_waiting;
	uint8_t *buf = malloc(size);

    /* We'll allow a 100ms timeout for receive. */
    unsigned int timeout = 100;
	int result = check(sp_blocking_read(rx_port, buf, size, timeout));

	/* Check whether we received the number of bytes we wanted. */
	if (result > 0) {
        printf("Received %d bytes successfully.\n", result);
        // printData(buf, result, true);
        stark_did_receive_data(device, buf, result);
    } else {
        printf("Timed out, none data received, result: %d\n", result);
    }

	/* Free receive buffer. */
	free(buf);
}

void open_serial() {
    /* Open and configure port. */
	check(sp_get_port_by_name(SERIAL_PORT, &port));

	printf("Opening port.\n");
	check(sp_open(port, SP_MODE_READ_WRITE));

	printf("Setting port to %u 8N1, no flow control.\n", BAUD_RATE);
	check(sp_set_baudrate(port, BAUD_RATE));
	check(sp_set_bits(port, 8));
	check(sp_set_parity(port, SP_PARITY_NONE));
	check(sp_set_stopbits(port, 1));
	check(sp_set_flowcontrol(port, SP_FLOWCONTROL_NONE));

    printf("Adding port RX event to event set.\n");
    check(sp_new_event_set(&rx_event_set));
    check(sp_add_port_events(rx_event_set, port, SP_EVENT_RX_READY));
}

void close_serial() {
    /* Close ports and free resources. */
    if (rx_event_set) {
        printf("Freeing event set.\n");
        sp_free_event_set(rx_event_set);
    }
    if (port == NULL) return;
    printf("Closing port.\n");
	check(sp_close(port));
	sp_free_port(port);
}

void on_error(const char *device_id, int error) {
    if (device_id == NULL) {
        printf("on_error, error: %d\n", error);
        return;
    }
    printf("on_error, device_id: %s, error: %d\n", device_id, error);
}

void on_serialport_cfg(const char *device_id, SerialPortCfg *cfg) {
    printf("on_serialport_cfg, device_id: %s, serial_device_id: %d, baudrate: %d\n", device_id, cfg->serial_device_id, cfg->baudrate);
}

void on_hand_type(const char *device_id, int hand_type) {
    printf("on_hand_type, device_id: %s, hand_type: %d\n", device_id, hand_type);
}

void on_force_level(const char *device_id, int force_level) {
    printf("on_force_level, device_id: %s, force_level: %d\n", device_id, force_level);
}

void on_motorboard_info(const char *device_id, MotorboardInfo *info) {
    printf("on_motorboard_info, device_id: %s, hand_type: %d, sn: %s, fw_version: %s\n", device_id, info->hand_type, info->sn, info->fw_version);
}

void on_voltage(const char *device_id, float voltage) {
    printf("on_voltage, device_id: %s, voltage: %f\n", device_id, voltage);
}

void on_limit_current(const char *device_id, int limit_current) {
    printf("on_limit_current, device_id: %s, limit_current: %d\n", device_id, limit_current);
}

void on_finger_status(const char *device_id, StarkFingerStatusData *finger_status) {
    printf("on_finger_status, device_id: %s, finger_status: %p\n", device_id, finger_status);
}

void on_finger_movement_status(const char *device_id, StarkFingerMovementData *finger_movement_status) {
    printf("on_finger_movement_status, device_id: %s, finger_movement_status: %p\n", device_id, finger_movement_status);
}

void on_button_event(const char *device_id, ButtonPressEvent *event) {
    printf("on_button_event, device_id: %s, button_event: %p\n", device_id, event);
}

// 主函数
int main(int argc, char **argv) {
    printf("----------------------Main begin----------------------\n");
    printf("StarkSDK version: v%s\n", stark_get_sdk_version());
    stark_set_write_data_callback(send_data);

    open_serial();

    const char* device_id = "Stark_OK";
    const int serial_device_id = 10;
    // const int serial_device_id = 254; // boardcast id
    StarkDevice* device = stark_create_device(device_id, serial_device_id);
    stark_set_error_callback(device, on_error);

    // setters
    factory_set_device_sn(device, "stark-key", "stark-sn");
    factory_set_hand_type(device, "stark-key", MEDIUM_LEFT);
    stark_set_serialport_cfg(device, BAUD_RATE);
    stark_set_serial_device_id(device, 99);
    stark_set_force_level(device, STARK_FORCE_LEVEL_NORMAL);
    stark_set_max_current(device, 2000); // 2000mA
    stark_reset_finger_positions(device);
    stark_set_finger_position(device, 66);
    const int finger_positions[6] = {50, 40, 30, 20, 50, 50};
    stark_set_finger_positions(device, finger_positions);
    stark_set_finger_speed(device, -69);
    const int finger_speeds[6] = {30, 30, 30, 30, -30, 30};
    stark_set_finger_speeds(device, finger_speeds);
    stark_set_led_info(device, LED_MODE_BLINK, LED_COLOR_R);
    stark_group_set_finger_speeds(finger_speeds);
    stark_group_set_finger_positions(finger_positions);

    // getters
    stark_get_serialport_cfg(device, on_serialport_cfg);
    receive_data(device);
    stark_get_hand_type(device, on_hand_type);
    receive_data(device);
    stark_get_force_level(device, on_force_level);
    receive_data(device);
    stark_get_motorboard_info(device, on_motorboard_info);
    receive_data(device);
    stark_get_voltage(device, on_voltage);
    receive_data(device);
    stark_get_max_current(device, on_limit_current);
    receive_data(device);
    stark_get_finger_status(device, on_finger_status);
    receive_data(device);
    stark_get_finger_movement_status(device, on_finger_movement_status);
    receive_data(device);
    stark_get_button_event(device, on_button_event);
    receive_data(device);

    close_serial();
    return 0;
}

/* Helper function for error handling. */
int check(enum sp_return result)
{
	/* For this example we'll just exit on any error by calling abort(). */
	char *error_message;

	switch (result) {
	case SP_ERR_ARG:
		printf("Error: Invalid argument.\n");
		abort();
	case SP_ERR_FAIL:
		error_message = sp_last_error_message();
		printf("Error: Failed: %s\n", error_message);
		sp_free_error_message(error_message);
		abort();
	case SP_ERR_SUPP:
		printf("Error: Not supported.\n");
		abort();
	case SP_ERR_MEM:
		printf("Error: Couldn't allocate memory.\n");
		abort();
	case SP_OK:
	default:
		return result;
	}
}

void printData(const uint8_t *data, size_t size, bool format) {
    for (int i = 0; i < size; ++i) {
        if (format) {
            printf("0x%02X", data[i]); // 打印当前字节的十六进制表示
        } else {
            printf("%02X", data[i]); // 打印当前字节的十六进制表示
        }
        if (i != size - 1) {
            if (format) printf(","); // 除了最后一个字节外，每个字节后面加逗号
        } else {
            printf("\n"); // 最后一个字节后面加换行符
        }
    }
}
