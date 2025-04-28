#ifndef STARK_NODE_HPP
#define STARK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include "ros2_stark_interfaces/msg/motor_status.hpp"
// #include "ros2_stark_interfaces/msg/touch_status_item.hpp"
#include "ros2_stark_interfaces/msg/touch_status.hpp"
#include "ros2_stark_controller/stark-sdk.h"

class StarkNode : public rclcpp::Node {
public:
    StarkNode();
    ~StarkNode();

private:
    // ROS 2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<ros2_stark_interfaces::msg::MotorStatus>::SharedPtr motor_status_pub_;
    rclcpp::Publisher<ros2_stark_interfaces::msg::TouchStatus>::SharedPtr touch_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr turbo_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr voltage_pub_;

    // ROS 2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr force_level_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr turbo_mode_sub_;

    // ROS 2 Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr info_timer_;

    // Stark SDK handle
    ModbusHandle* handle_;
    uint8_t slave_id_;

    // Configuration parameters
    std::string port_;
    uint32_t baudrate_;
    StarkFirmwareType fw_type_;
    StarkProtocolType protocol_type_;
    LogLevel log_level_;

    // Callback functions
    void timer_callback();
    void info_timer_callback();
    void joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void force_level_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void turbo_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Helper functions
    bool initialize_modbus();
    void publish_joint_state();
    void publish_device_status();
    void publish_motor_status();
    void publish_touch_status();
};

#endif // STARK_NODE_HPP

