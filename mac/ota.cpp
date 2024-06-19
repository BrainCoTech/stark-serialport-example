#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <execinfo.h>
#include <signal.h>
#include <libserialport.h>
#include <stark_sdk.h>
#include <uv.h>
#include <thread>
#include <future>

#ifdef __APPLE__
#define SERIAL_PORT "/dev/tty.usbserial-14310"
#elif __linux__
#define SERIAL_PORT "/dev/ttyUSB0"
#endif

#define BAUD_RATE 115200

/* The ports we will use. */
struct sp_port *port = NULL;
static StarkDfuState dfu_state = STARK_DFU_STATE_IDLE;

/* Helper function for error handling. */
int check(enum sp_return result);
void printData(const uint8_t *data, int size, bool format);

// 发送数据到 RS485
int send_data(const char* device_id, const uint8_t *data, int size) {
    if (port == NULL) {
        printf("send_data, while Port is NULL.\n");
        return -1;
    }

	printf("Sending (%d bytes).\n", size);
    // printData(data, size, true);

    /* We'll allow a 1 second timeout for send. */
	unsigned int timeout = 1000;
    int result = check(sp_blocking_write(port, data, size, timeout));
	if (result == size) {
        printf("Sent %d bytes successfully.\n", size);
    } else {
        printf("Timed out, %d/%d bytes sent.\n", result, size);
    }
    return 0;
}

// 等待串口数据的辅助函数
int wait_for_data(int timeout) {
    int size = 0;
    bool more_data = false;
    time_t start = time(NULL);

    while (time(NULL) - start <= timeout) {
        int available = sp_input_waiting(port);
        if ((more_data && available == 0) || (more_data && available == size)) {
            break;
        }
        if (available > 0 && available != size) {
            more_data = true;
            size = available;
            printf("Data waiting: %d bytes.\n", size);
        }
        usleep(10000); // 等待10毫秒
    }

    return size;
}

// 接收数据的主函数
void receive_data(StarkDevice *device, int timeout) {
    if (port == NULL) {
        printf("receive_data: Port is NULL.\n");
        return;
    }

    printf("Waiting for data...\n");
    int bytes_waiting = wait_for_data(timeout); // 等待最多timeout秒钟

    if (bytes_waiting <= 0) {
        printf("No data waiting, bytes_waiting: %d.\n", bytes_waiting);
        return;
    }

    printf("Receiving %d bytes data.\n", bytes_waiting);

    // 分配缓冲区接收数据
    uint8_t *buf = (uint8_t *)malloc(bytes_waiting);
    if (buf == NULL) {
        perror("malloc failed");
        return;
    }

    // 设置超时时间为100毫秒
    int result = sp_blocking_read(port, buf, bytes_waiting, 100);
    if (result > 0) {
        printf("Received %d bytes successfully.\n", result);
        printData(buf, result, true);
        stark_did_receive_data(device, buf, result);
    } else {
        printf("Error receiving data: %s\n", sp_last_error_message());
    }

    free(buf); // 释放接收缓冲区
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

    sp_flush(port, SP_BUF_BOTH);
}

void close_serial() {
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

void on_motorboard_info(const char *device_id, MotorboardInfo *info) {
    printf("on_motorboard_info, device_id: %s, hand_type: %d, sn: %s, fw_version: %s\n", device_id, info->hand_type, info->sn, info->fw_version);
}

void async_dfu_read(uv_timer_t* timer_handle) {
    printf("async_dfu_read\n");
    if (timer_handle == NULL) {
        fprintf(stderr, "async_dfu_read: Invalid timer handle\n");
        return;
    }

    const char *device_id = (const char *)timer_handle->data;
    if (device_id == NULL) {
        fprintf(stderr, "async_dfu_read: Timer data (device_id) is NULL\n");
        uv_timer_stop(timer_handle);
        free(timer_handle);
        return;
    }

    StarkDevice *device = stark_get_device(device_id);
    if (device == NULL) {
        fprintf(stderr, "async_dfu_read: Failed to retrieve device for id: %s\n", device_id);
        uv_timer_stop(timer_handle);
        free(timer_handle);
        return;
    }

    uv_timer_stop(timer_handle);
    free(timer_handle);
}

void on_dfu_read(const char *device_id) {
    printf("on_dfu_read\n");
    StarkDevice *device = stark_get_device(device_id);
    if (device == NULL) {
        fprintf(stderr, "async_dfu_read: Failed to retrieve device for id: %s\n", device_id);
        return;
    }
    std::thread([device]() {
        receive_data(device, 3);
    }).detach();
}

void on_dfu_state(const char *device_id, int state) {
    printf("on_ota_state, device_id: %s, dfu_state: %s\n", device_id, stark_dfu_state_to_string(state));
    dfu_state = (StarkDfuState)state;
}

void on_dfu_progress(const char *device_id, float progress) {
    printf("on_ota_progress, device_id: %s, progress: %.1f\n", device_id, progress * 100.0);
}

int modbus_write_registers(const char *device_id, const uint16_t *data, int register_address, int count) {
    printf("modbus_write_registers, device_id: %s, register_address: %d, count: %d\n", device_id, register_address, count);
    if (register_address == 900) {
        // send enter ota cmd
        uint8_t data[] = {0x0, 0x10, 0x3, 0x84, 0x0, 0x1, 0x2, 0x0, 0x1, 0x47, 0x44};
        send_data(device_id, data, sizeof(data));
    }
    return 0;
}

void timer_close_cb(uv_handle_t* handle){
    if(handle){
        free(handle);
        handle = NULL;
    }
}

void loop_timer_cb(uv_timer_t* handle) {
    // printf("Timer callback called\n");
}

void start_loop() {
    uv_loop_t *loop = uv_default_loop();

    // 分配内存给定时器句柄
    uv_timer_t *timer_handle = (uv_timer_t*)malloc(sizeof(uv_timer_t));
    if (timer_handle == NULL) {
        fprintf(stderr, "Failed to allocate memory for timer handle\n");
        return;
    }

    // 初始化定时器
    uv_timer_init(loop, timer_handle);

    // 启动定时器，每2000ms触发一次回调
    int result = uv_timer_start(timer_handle, loop_timer_cb, 1000, 2000);
    if (result != 0) {
        fprintf(stderr, "Failed to start timer: %s\n", uv_strerror(result));
        free(timer_handle);
        return;
    }

    // 运行事件循环
    uv_run(loop, UV_RUN_DEFAULT);

    // 停止和关闭定时器
    uv_timer_stop(timer_handle);
    uv_close((uv_handle_t*)timer_handle, timer_close_cb);

    printf("Event loop finished\n");
}

void handler(int sig) {
    void *array[10];
    size_t size;

    // 获取堆栈帧
    size = backtrace(array, 10);

    // 打印所有堆栈帧到 stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int main(int argc, char *argv[]) {
    signal(SIGSEGV, handler);   // Install our handler for SIGSEGV (segmentation fault)
    signal(SIGABRT, handler);   // Install our handler for SIGABRT (abort signal)

    printf("----------------------Main begin----------------------\n");
    printf("StarkSDK version: v%s\n", stark_get_sdk_version());
    stark_set_write_data_callback(send_data);
    
    open_serial();

    const char* device_id = "Stark_OK";

    // proto协议固件
    // const int serial_device_id = 10; // 10 ~ 254, 9.2.7固件
    const int serial_device_id = 2; // 1.2.4 固件
    StarkDevice* device = stark_create_serial_device(device_id, serial_device_id);
    printf("serial_device: %p\n", device);

    // Modbus协议固件
    // const int slave_id = 0; # 0 ~ 254
    // StarkDevice* device = stark_create_modbus_device(device_id, slave_id);
    // modbus_set_write_registers_callback(device, modbus_write_registers);

    // start ota
    stark_set_dfu_read_callback(device, on_dfu_read);
    stark_set_dfu_state_callback(device, on_dfu_state);
    stark_set_dfu_progress_callback(device, on_dfu_progress);

    dfu_state = STARK_DFU_STATE_IDLE;
    // stark_start_dfu(device, "../ota_bin/FW_MotorController_Release_SecureOTA_485_1.2.4.ota");
    stark_start_dfu(device, "../ota_bin/FW_MotorController_Release_SecureOTA_485_9.2.7.ota");

    start_loop();
    close_serial();
    return 0;
}

/* Helper function for error handling. */
int check(enum sp_return result) {
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

void printData(const uint8_t *data, int size, bool format) {
    for (int i = 0; i < size; i++) {
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
