#ifndef STARK_NODE_HPP
#define STARK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include "ros2_stark_msgs/srv/get_device_info.hpp"
#include "ros2_stark_msgs/srv/set_motor_multi.hpp"
#include "ros2_stark_msgs/srv/set_motor_single.hpp"
#include "ros2_stark_msgs/msg/motor_status.hpp"
#include "ros2_stark_msgs/msg/touch_status.hpp"
#include "ros2_stark_msgs/msg/set_motor_multi.hpp"
#include "ros2_stark_msgs/msg/set_motor_single.hpp"
#include "ros2_stark_controller/stark-sdk.h"
// #include "ros2_stark_controller/zcan.h"

class StarkNode : public rclcpp::Node {
public:
    StarkNode();
    ~StarkNode();

private:
    // ROS 2 Services
    rclcpp::Service<ros2_stark_msgs::srv::GetDeviceInfo>::SharedPtr get_device_info_service_;
    rclcpp::Service<ros2_stark_msgs::srv::SetMotorMulti>::SharedPtr set_motor_multi_service_;
    rclcpp::Service<ros2_stark_msgs::srv::SetMotorSingle>::SharedPtr set_motor_single_service_;
    
    // ROS 2 Publishers
    rclcpp::Publisher<ros2_stark_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_;
    rclcpp::Publisher<ros2_stark_msgs::msg::TouchStatus>::SharedPtr touch_status_pub_;

    // ROS 2 Subscribers
    rclcpp::Subscription<ros2_stark_msgs::msg::SetMotorMulti>::SharedPtr motor_multi_sub_;
    rclcpp::Subscription<ros2_stark_msgs::msg::SetMotorSingle>::SharedPtr motor_single_sub_;

    // ROS 2 Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr info_timer_;

    // Stark SDK handle
    DeviceHandler* handle_;
    uint8_t slave_id_;

    // Configuration parameters
    std::string port_;
    uint32_t baudrate_;
    StarkHardwareType fw_type_;
    StarkProtocolType protocol_type_;
    LogLevel log_level_;

    // device information
    SkuType sku_type_;
    std::string sn_;
    std::string fw_version_;
    uint32_t voltage_;
    bool is_turbo_mode_enabled_;

    // Callback functions for timers
    void timer_callback();
    // void info_timer_callback();

    // Callback functions for services
    void handle_get_device_info(
        const std::shared_ptr<ros2_stark_msgs::srv::GetDeviceInfo::Request> request,
        std::shared_ptr<ros2_stark_msgs::srv::GetDeviceInfo::Response> response);
    void handle_set_motor_multi(
        const std::shared_ptr<ros2_stark_msgs::srv::SetMotorMulti::Request> request,
        std::shared_ptr<ros2_stark_msgs::srv::SetMotorMulti::Response> response);
    void handle_set_motor_single(
        const std::shared_ptr<ros2_stark_msgs::srv::SetMotorSingle::Request> request,
        std::shared_ptr<ros2_stark_msgs::srv::SetMotorSingle::Response> response);

    // Callback functions for subscribers
    void handle_motor_multi_command(
        const std::shared_ptr<ros2_stark_msgs::msg::SetMotorMulti> msg);
    void handle_motor_single_command(
        const std::shared_ptr<ros2_stark_msgs::msg::SetMotorSingle> msg);

    // Helper functions
    bool initialize_stark_handler();
    // void publish_device_status();
    void publish_motor_status();
    void publish_touch_status();
};

#endif // STARK_NODE_HPP

