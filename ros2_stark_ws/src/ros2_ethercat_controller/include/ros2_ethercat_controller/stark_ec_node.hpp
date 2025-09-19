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

enum LogLevel : uint8_t {
  LOG_LEVEL_ERROR = 0,
  LOG_LEVEL_WARN = 1,
  LOG_LEVEL_INFO = 2,
  LOG_LEVEL_DEBUG = 3,
  LOG_LEVEL_TRACE = 4,
};

enum SkuType : uint8_t {
  SKU_TYPE_MEDIUM_RIGHT = 1,
  SKU_TYPE_MEDIUM_LEFT = 2,
  SKU_TYPE_SMALL_RIGHT = 3,
  SKU_TYPE_SMALL_LEFT = 4,
};

enum StarkFingerId : uint8_t {
  STARK_FINGER_ID_THUMB = 1,
  STARK_FINGER_ID_THUMB_AUX = 2,
  STARK_FINGER_ID_INDEX = 3,
  STARK_FINGER_ID_MIDDLE = 4,
  STARK_FINGER_ID_RING = 5,
  STARK_FINGER_ID_PINKY = 6,
};

// enum TouchSensorStatus : uint8_t {
//   TOUCH_SENSOR_STATUS_NORMAL = 0,
//   TOUCH_SENSOR_STATUS_ABNORMAL = 1,
//   TOUCH_SENSOR_STATUS_COMMUNICATION_ERROR = 2,
//   TOUCH_SENSOR_STATUS_UNKNOWN = 255,
// };

class StarkNode : public rclcpp::Node {
public:
    StarkNode();
    ~StarkNode();

private:
    // ROS 2 Services
    rclcpp::Service<ros2_stark_msgs::srv::GetDeviceInfo>::SharedPtr get_device_info_service_left;
    rclcpp::Service<ros2_stark_msgs::srv::GetDeviceInfo>::SharedPtr get_device_info_service_right;
    rclcpp::Service<ros2_stark_msgs::srv::SetMotorMulti>::SharedPtr set_motor_multi_service_left;
    rclcpp::Service<ros2_stark_msgs::srv::SetMotorMulti>::SharedPtr set_motor_multi_service_right;
    rclcpp::Service<ros2_stark_msgs::srv::SetMotorSingle>::SharedPtr set_motor_single_service_left;
    rclcpp::Service<ros2_stark_msgs::srv::SetMotorSingle>::SharedPtr set_motor_single_service_right;
    
    // ROS 2 Publishers
    rclcpp::Publisher<ros2_stark_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_left;
    rclcpp::Publisher<ros2_stark_msgs::msg::MotorStatus>::SharedPtr motor_status_pub_right;
    rclcpp::Publisher<ros2_stark_msgs::msg::TouchStatus>::SharedPtr touch_status_pub_left;
    rclcpp::Publisher<ros2_stark_msgs::msg::TouchStatus>::SharedPtr touch_status_pub_right;

    // ROS 2 Subscribers
    rclcpp::Subscription<ros2_stark_msgs::msg::SetMotorMulti>::SharedPtr motor_multi_sub_left;
    rclcpp::Subscription<ros2_stark_msgs::msg::SetMotorMulti>::SharedPtr motor_multi_sub_right;
    rclcpp::Subscription<ros2_stark_msgs::msg::SetMotorSingle>::SharedPtr motor_single_sub_left;
    rclcpp::Subscription<ros2_stark_msgs::msg::SetMotorSingle>::SharedPtr motor_single_sub_right;

    // ROS 2 Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr info_timer_;

    // Configuration parameters
    LogLevel log_level_;
    uint8_t slave_pos_left; // 从站pos
    uint8_t slave_pos_right; 

    // device information
    // StarkHardwareType fw_type_left;
    // StarkHardwareType fw_type_right;
    // SkuType sku_type_left;
    // SkuType sku_type_right;
    // std::string sn_left;
    // std::string sn_right;
    // std::string fw_version_left;
    // std::string fw_version_right;
    // bool is_turbo_mode_enabled_left;
    // bool is_turbo_mode_enabled_right;

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
    void queue_motor_multi_command(
        const std::shared_ptr<ros2_stark_msgs::msg::SetMotorMulti> msg);
    void queue_motor_single_command(
        const std::shared_ptr<ros2_stark_msgs::msg::SetMotorSingle> msg);

    // Helper functions
    bool initialize_stark_handler();
    // void publish_device_status();
    void publish_motor_status();
    void publish_touch_status();
};

#endif // STARK_NODE_HPP