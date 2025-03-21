#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import argparse
import time

class StarkPositionControl(Node):
    def __init__(self):
        super().__init__('stark_position_control')
        self.publisher_ = self.create_publisher(JointState, '/joint_commands', 10)
        self.joint_names = ['thumb', 'thumb_aux', 'index', 'middle', 'ring', 'pinky']

    def publish_position(self, positions, rate=None):
        """发布关节位置消息"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions  # 期望 6 个值，范围 0.0-1.0

        if rate is None:
            # 单次发布
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published positions: {positions}")
        else:
            # 持续发布
            period = 1.0 / rate
            while rclpy.ok():
                msg.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published positions: {positions}")
                time.sleep(period)

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Control Stark robotic hand positions')
    parser.add_argument('--positions', type=float, nargs=6, default=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
                        help='6 joint positions (0.0-1.0): thumb, thumb_aux, index, middle, ring, pinky')
    parser.add_argument('--rate', type=float, default=None,
                        help='Publishing rate in Hz (omit for single publish)')
    args = parser.parse_args()

    # 验证位置范围
    for pos in args.positions:
        if not 0.0 <= pos <= 1.0:
            print(f"Error: Positions must be between 0.0 and 1.0, got {pos}")
            return

    # 初始化 ROS 2
    rclpy.init()
    node = StarkPositionControl()

    try:
        node.publish_position(args.positions, args.rate)
    except KeyboardInterrupt:
        pass

    # 清理
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()