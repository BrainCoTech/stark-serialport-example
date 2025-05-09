#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros2_stark_interfaces.srv import GetDeviceInfo, SetMotorMulti, SetMotorSingle
import sys
import time

class StarkClientNode(Node):
    def __init__(self, target_slave_id=1):
        """初始化Stark客户端节点"""
        super().__init__('stark_node_client')
        
        # 设置目标设备ID
        self.target_slave_id = target_slave_id
        self.node_id_str = f'Stark client node[{self.target_slave_id}]'
        self.log_info(f'Creating client for device')
        
        # 创建服务客户端
        slave_id_str = str(self.target_slave_id)
        self.get_device_info_client = self.create_client(
            GetDeviceInfo, f'get_device_info_{slave_id_str}')
        self.set_motor_multi_client = self.create_client(
            SetMotorMulti, f'set_motor_multi_{slave_id_str}')
        self.set_motor_single_client = self.create_client(
            SetMotorSingle, f'set_motor_single_{slave_id_str}')
        
        # 等待服务可用
        self.log_info('Waiting for services to be available...')
        self.wait_for_services()
        self.log_info('All services are available.')
        self.log_info('initialized successfully')
    
    def log_info(self, message):
        """统一的日志信息格式"""
        self.get_logger().info(f'{self.node_id_str} {message}')
        
    def log_error(self, message):
        """统一的错误日志格式"""
        self.get_logger().error(f'{self.node_id_str} {message}')
    
    def wait_for_services(self):
        """等待所有服务可用"""
        services = [
            (self.get_device_info_client, "GetDeviceInfo"),
            (self.set_motor_multi_client, "SetMotorMulti"),
            (self.set_motor_single_client, "SetMotorSingle")
        ]
        
        for client, name in services:
            self.log_info(f'Waiting for {name} service...')
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    self.log_error('Interrupted while waiting for service')
                    return False
                self.log_info(f'{name} service not available, waiting...')
            
            self.log_info(f'{name} service is now available')
        
        return True
    
    def get_device_info(self):
        """获取设备信息"""
        request = GetDeviceInfo.Request()
        # request.get_sku_type = True
        # request.get_serial_number = True
        # request.get_firmware_version = True
        request.get_voltage = True
        request.get_turbo_mode = True
        
        self.get_logger().info('Sending get_device_info request...')
        future = self.get_device_info_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                if response is not None:
                    self.get_logger().info(
                        f"Device Info: SKU Type: {response.sku_type}, "
                        f"Serial Number: {response.serial_number}, "
                        f"Firmware Version: {response.firmware_version}, "
                        f"Voltage: {response.voltage}, "
                        f"Turbo Mode: {response.turbo_mode}"
                    )
                    return response
                else:
                    self.get_logger().error('Received empty response')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        else:
            self.get_logger().error('Future was not completed')
        
        return None
    
    def set_all_finger_positions(self, positions):
        """设置所有手指位置
        
        Args:
            positions: 长度为6的列表，表示6个电机的目标位置
        """
        if len(positions) != 6:
            self.get_logger().error(f'Expected 6 positions, got {len(positions)}')
            return False
            
        request = SetMotorMulti.Request()
        request.mode = 1  # 位置模式
        request.positions = positions
        
        self.get_logger().info(f'Setting all fingers to positions: {positions}')
        future = self.set_motor_multi_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Set positions result: {response.success}")
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        
        return False
    
    def set_all_finger_speeds(self, speeds):
        """设置所有手指速度
        
        Args:
            speeds: 长度为6的列表，表示6个电机的目标速度
        """
        if len(speeds) != 6:
            self.get_logger().error(f'Expected 6 speeds, got {len(speeds)}')
            return False
            
        request = SetMotorMulti.Request()
        request.mode = 2  # 速度模式
        request.speeds = speeds
        
        self.get_logger().info(f'Setting all fingers to speeds: {speeds}')
        future = self.set_motor_multi_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Set speeds result: {response.success}")
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        
        return False
    
    def set_all_finger_currents(self, currents):
        """设置所有手指电流
        
        Args:
            currents: 长度为6的列表，表示6个电机的目标电流
        """
        if len(currents) != 6:
            self.get_logger().error(f'Expected 6 currents, got {len(currents)}')
            return False
            
        request = SetMotorMulti.Request()
        request.mode = 3
        request.currents = currents
        self.get_logger().info(f'Setting all fingers to currents: {currents}')
        future = self.set_motor_multi_client.call_async(request)
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Set currents result: {response.success}")
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        return False
    
    def set_all_finger_pwms(self, pwms):
        """设置所有手指PWM
        
        Args:
            pwms: 长度为6的列表，表示6个电机的目标PWM
        """
        if len(pwms) != 6:
            self.get_logger().error(f'Expected 6 PWMs, got {len(pwms)}')
            return False
            
        request = SetMotorMulti.Request()
        request.mode = 4
        request.pwms = pwms
        self.get_logger().info(f'Setting all fingers to PWMs: {pwms}')
        future = self.set_motor_multi_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Set PWMs result: {response.success}")
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        
        return False
    def set_all_finger_positions_with_mills(self, positions, durations):
        """设置所有手指位置+期望时间
        
        Args:
            positions: 长度为6的列表，表示6个电机的目标位置
            durations: 长度为6的列表，表示6个电机的期望时间（毫秒）
        """
        if len(positions) != 6:
            self.get_logger().error(f'Expected 6 positions, got {len(positions)}')
            return False
        if len(durations) != 6:
            self.get_logger().error(f'Expected 6 durations, got {len(durations)}')
            return False
            
        request = SetMotorMulti.Request()
        request.mode = 5 # 位置+期望时间
        request.positions = positions
        request.durations = durations
        self.get_logger().info(f'Setting all fingers to positions: {positions} with time: {durations}')
        future = self.set_motor_multi_client.call_async(request)
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Set positions with time result: {response.success}")
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        return False
    
    def set_all_finger_positions_with_speeds(self, positions, speeds):
        """设置所有手指位置+速度
        
        Args:
            positions: 长度为6的列表，表示6个电机的目标位置
            speed: 长度为6的列表，表示6个电机的期望速度
        """
        if len(positions) != 6:
            self.get_logger().error(f'Expected 6 positions, got {len(positions)}')
            return False
        if len(speeds) != 6:
            self.get_logger().error(f'Expected 6 speeds, got {len(speeds)}')
            return False
            
        request = SetMotorMulti.Request()
        request.mode = 6 # 位置+速度
        request.positions = positions
        request.speeds = speeds
        self.get_logger().info(f'Setting all fingers to positions: {positions} with speeds: {speeds}')
        future = self.set_motor_multi_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f"Set positions with speed result: {response.success}")
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        
        return False
    
    def set_single_finger_position(self, finger_id, position):
        """设置单个手指位置
        
        Args:
            finger_id: 电机ID (1~6)
            position: 目标位置
        """
        if finger_id < 1 or finger_id > 6:
            self.get_logger().error(f'Invalid finger_id: {finger_id}, must be 1~6')
            return False
            
        request = SetMotorSingle.Request()
        request.mode = 1  # 位置模式
        request.motor_id = finger_id
        request.position = position
        
        self.get_logger().info(f'Setting finger {finger_id} to position: {position}')
        future = self.set_motor_single_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Set finger {finger_id} position to {position}: {response.success}"
                )
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        
        return False
    
    def set_single_finger_speed(self, finger_id, speed):
        """设置单个手指速度
        
        Args:
            finger_id: 电机ID (1~6)
            speed: 目标速度
        """
        if finger_id < 1 or finger_id > 6:
            self.get_logger().error(f'Invalid finger_id: {finger_id}, must be 1~6')
            return False
            
        request = SetMotorSingle.Request()
        request.mode = 2  # 速度模式
        request.motor_id = finger_id
        request.speed = speed
        
        self.get_logger().info(f'Setting finger {finger_id} to speed: {speed}')
        future = self.set_motor_single_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Set finger {finger_id} speed to {speed}: {response.success}"
                )
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        
        return False
    
    def set_single_finger_current(self, finger_id, current):
        """设置单个手指电流
        
        Args:
            finger_id: 电机ID (1~6)
            current: 目标电流
        """
        if finger_id < 1 or finger_id > 6:
            self.get_logger().error(f'Invalid finger_id: {finger_id}, must be 1~6')
            return False
            
        request = SetMotorSingle.Request()
        request.mode = 3
        request.motor_id = finger_id
        request.current = current
        self.get_logger().info(f'Setting finger {finger_id} to current: {current}')

        future = self.set_motor_single_client.call_async(request)

        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Set finger {finger_id} current to {current}: {response.success}"
                )
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

        return False

    def set_single_finger_pwm(self, finger_id, pwm):
        """设置单个手指PWM
        
        Args:
            finger_id: 电机ID (1~6)
            pwm: 目标PWM
        """
        if finger_id < 1 or finger_id > 6:
            self.get_logger().error(f'Invalid finger_id: {finger_id}, must be 1~6')
            return False
            
        request = SetMotorSingle.Request()
        request.mode = 4
        request.motor_id = finger_id
        request.pwm = pwm
        self.get_logger().info(f'Setting finger {finger_id} to PWM: {pwm}')

        future = self.set_motor_single_client.call_async(request)

        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Set finger {finger_id} PWM to {pwm}: {response.success}"
                )
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

        return False

    def set_single_finger_position_with_mills(self, finger_id, position, mills):
        """设置单个手指位置+期望时间
        
        Args:
            finger_id: 电机ID (1~6)
            position: 目标位置
            mills: 期望时间（毫秒）
        """
        if finger_id < 1 or finger_id > 6:
            self.get_logger().error(f'Invalid finger_id: {finger_id}, must be 1~6')
            return False
            
        request = SetMotorSingle.Request()
        request.mode = 5 # 位置+期望时间
        request.motor_id = finger_id
        request.position = position
        request.duration = mills
        self.get_logger().info(f'Setting finger {finger_id} to position: {position} with time: {mills}')

        future = self.set_motor_single_client.call_async(request)

        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Set finger {finger_id} position to {position} with time {mills}: {response.success}"
                )
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

        return False
    
    def set_single_finger_position_with_speed(self, finger_id, position, speed):
        """设置单个手指位置+速度
        
        Args:
            finger_id: 电机ID (1~6)
            position: 目标位置
            speed: 期望速度
        """
        if finger_id < 1 or finger_id > 6:
            self.get_logger().error(f'Invalid finger_id: {finger_id}, must be 1~6')
            return False
            
        request = SetMotorSingle.Request()
        request.mode = 6 # 位置+速度
        request.motor_id = finger_id
        request.position = position
        request.speed = speed
        self.get_logger().info(f'Setting finger {finger_id} to position: {position} with speed: {speed}')

        future = self.set_motor_single_client.call_async(request)

        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(
                    f"Set finger {finger_id} position to {position} with speed {speed}: {response.success}"
                )
                return response.success
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')

        return False

def main(args=None):
    rclpy.init(args=args)
    
    # 解析命令行参数，获取目标从机ID
    slave_id = 1  # 默认值
    if len(sys.argv) > 1:
        try:
            slave_id = int(sys.argv[1])
        except ValueError:
            print(f"Invalid slave_id: {sys.argv[1]}, using default: 1")
    
    print(f"slave_id: {slave_id}")
    client_node = StarkClientNode(slave_id)
    
    try:
        # 示例: 获取设备信息
        client_node.get_device_info()
        
        # 示例: 设置所有手指位置到中间位置
        positions = [50] * 6  # 所有手指位置设置为50
        client_node.set_all_finger_positions(positions)
        
        # 等待一秒
        time.sleep(1)
        
        # 示例: 设置最后一个手指位置到100
        client_node.set_single_finger_position(6, 100)
        
        # 提示用户程序已完成
        print("\nDemonstration complete! Press Ctrl+C to exit.")
        rclpy.spin(client_node)
        
    except KeyboardInterrupt:
        print("User interrupted, shutting down...")
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()