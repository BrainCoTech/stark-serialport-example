#include "ros2_stark_controller/stark_node.hpp"

StarkNode::StarkNode() : Node("stark_node"), handle_(nullptr) {
  // Declare and Get parameters
  port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
  baudrate_ = this->declare_parameter<int>("baudrate", 115200);
  slave_id_ = this->declare_parameter<int>("slave_id", 1);
  fw_type_ = static_cast<StarkHardwareType>(this->declare_parameter<int>("firmware_type", 2));
  protocol_type_ = static_cast<StarkProtocolType>(this->declare_parameter<int>("protocol_type", 1));
  log_level_ = static_cast<LogLevel>(this->declare_parameter<int>("log_level", 2));
  RCLCPP_INFO(this->get_logger(),
              "Port: %s, Baudrate: %d, slave_id: %d, firmware_type: %d, protocol_type: %d, log_level: %d",
              port_.c_str(), baudrate_, slave_id_, fw_type_, protocol_type_, log_level_);

  // Initialize Stark SDK
  init_cfg(fw_type_, protocol_type_, log_level_, 1024);
  // list_available_ports();
  if (!initialize_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Modbus");
    throw std::runtime_error("Modbus initialization failed");
  }

  if (fw_type_ == StarkHardwareType::STARK_HARDWARE_TYPE_REVO1_TOUCH || 
      fw_type_ == StarkHardwareType::STARK_HARDWARE_TYPE_REVO2_TOUCH) {
    // 启用全部触觉传感器
    modbus_enable_touch_sensor(handle_, slave_id_, 0x1F);
    // usleep(1000 * 1000); // wait for touch sensor to be ready
  }

  // Initialize services
  std::string slave_id = std::to_string(slave_id_);
  get_device_info_service_ = create_service<ros2_stark_interfaces::srv::GetDeviceInfo>(
      "get_device_info_" + slave_id,
      std::bind(&StarkNode::handle_get_device_info, this, std::placeholders::_1, std::placeholders::_2));
  set_motor_multi_service_ = create_service<ros2_stark_interfaces::srv::SetMotorMulti>(
      "set_motor_multi_" + slave_id,
      std::bind(&StarkNode::handle_set_motor_multi, this, std::placeholders::_1, std::placeholders::_2));
  set_motor_single_service_ = create_service<ros2_stark_interfaces::srv::SetMotorSingle>(
      "set_motor_single_" + slave_id,
      std::bind(&StarkNode::handle_set_motor_single, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize publishers
  motor_status_pub_ = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status", 10);
  touch_status_pub_ = create_publisher<ros2_stark_interfaces::msg::TouchStatus>("touch_status", 10);

  // Initialize subscribers
  motor_multi_sub_ = create_subscription<ros2_stark_interfaces::msg::SetMotorMulti>(
      "set_motor_multi_" + slave_id, 10, std::bind(&StarkNode::handle_motor_multi_command, this, std::placeholders::_1));
  motor_single_sub_ = create_subscription<ros2_stark_interfaces::msg::SetMotorSingle>(
      "set_motor_single_" + slave_id, 10, std::bind(&StarkNode::handle_motor_single_command, this, std::placeholders::_1));

  // Initialize timer
  timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&StarkNode::timer_callback, this));

  // 设备电量等信息，不需要频繁更新
  // info_timer_ = create_wall_timer(
  //     std::chrono::seconds(30),
  //     std::bind(&StarkNode::info_timer_callback, this));
}

StarkNode::~StarkNode() {
  if (handle_) {
    modbus_close(handle_);
  }
}

bool StarkNode::initialize_modbus() {
  handle_ = modbus_open(port_.c_str(), baudrate_);
  if (!handle_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open Modbus on %s", port_.c_str());
    return false;
  }
  auto info = modbus_get_device_info(handle_, slave_id_);
  if (info != NULL) {
    sku_type_ = info->sku_type;
    sn_ = std::string(info->serial_number);
    fw_version_ = std::string(info->firmware_version);
    RCLCPP_INFO(this->get_logger(), "Slave[%hhu] SKU Type: %hhu, Serial Number: %s, Firmware Version: %s\n", slave_id_,
                (uint8_t)info->sku_type, info->serial_number, info->firmware_version);
    free_device_info(info);
  }
  return true;
}

void StarkNode::timer_callback() {
  publish_motor_status();
  if (fw_type_ == StarkHardwareType::STARK_HARDWARE_TYPE_REVO1_TOUCH || 
      fw_type_ == StarkHardwareType::STARK_HARDWARE_TYPE_REVO2_TOUCH) {
    publish_touch_status();
  }
}

// void StarkNode::() {
//     publish_device_status();
// }

void StarkNode::publish_motor_status() {
  auto motor_status = modbus_get_motor_status(handle_, slave_id_);
  if (!motor_status) {
    RCLCPP_WARN(this->get_logger(), "Failed to get motor status");
    return;
  }

  auto msg = ros2_stark_interfaces::msg::MotorStatus();
  msg.slave_id = slave_id_;
  std::copy(motor_status->positions, motor_status->positions + 6, msg.positions.begin());
  std::copy(motor_status->speeds, motor_status->speeds + 6, msg.speeds.begin());
  std::copy(motor_status->currents, motor_status->currents + 6, msg.currents.begin());
  std::copy(motor_status->states, motor_status->states + 6, msg.states.begin());
  // print currents if not zero
  // for (int i = 0; i < 6; i++) {
  //     if (motor_status->currents[i] != 0) {
  //         RCLCPP_INFO(this->get_logger(), "Motor %d current: %d", i, motor_status->currents[i]);
  //     }
  // }

  motor_status_pub_->publish(msg);
  free_motor_status_data(motor_status);
}

void StarkNode::publish_touch_status() {
  auto touch_status = modbus_get_touch_status(handle_, slave_id_);
  if (!touch_status) {
    RCLCPP_WARN(this->get_logger(), "Failed to get touch status");
    return;
  }

  auto msg = ros2_stark_interfaces::msg::TouchStatus();
  msg.slave_id = slave_id_;
  // 填充固定 5 个元素的数组, 代表 5个手指上对应的传感器信息
  for (int i = 0; i < 5; i++) {
    msg.data[i].normal_force1 = touch_status->items[i].normal_force1;
    msg.data[i].normal_force2 = touch_status->items[i].normal_force2;
    msg.data[i].normal_force3 = touch_status->items[i].normal_force3;
    msg.data[i].tangential_force1 = touch_status->items[i].tangential_force1;
    msg.data[i].tangential_force2 = touch_status->items[i].tangential_force2;
    msg.data[i].tangential_force3 = touch_status->items[i].tangential_force3;
    msg.data[i].tangential_direction1 = touch_status->items[i].tangential_direction1;
    msg.data[i].tangential_direction2 = touch_status->items[i].tangential_direction2;
    msg.data[i].tangential_direction3 = touch_status->items[i].tangential_direction3;
    msg.data[i].self_proximity1 = touch_status->items[i].self_proximity1;
    msg.data[i].self_proximity2 = touch_status->items[i].self_proximity2;
    msg.data[i].mutual_proximity = touch_status->items[i].mutual_proximity;
    msg.data[i].status = touch_status->items[i].status;
  }

  touch_status_pub_->publish(msg);
  free_touch_finger_data(touch_status);
}

void StarkNode::handle_get_device_info(
    const std::shared_ptr<ros2_stark_interfaces::srv::GetDeviceInfo::Request> request,
    std::shared_ptr<ros2_stark_interfaces::srv::GetDeviceInfo::Response> response) {
  response->sku_type = static_cast<uint8_t>(sku_type_);
  response->serial_number = sn_;
  response->firmware_version = fw_version_;
  if (request->get_voltage) {
    voltage_ = modbus_get_voltage(handle_, slave_id_);
    response->voltage = voltage_;
  }
  if (request->get_turbo_mode) {
    is_turbo_mode_enabled_ = modbus_get_turbo_mode_enabled(handle_, slave_id_);
    response->turbo_mode = is_turbo_mode_enabled_;
  }
  response->success = true;
}

void StarkNode::handle_set_motor_multi(
    const std::shared_ptr<ros2_stark_interfaces::srv::SetMotorMulti::Request> request,
    std::shared_ptr<ros2_stark_interfaces::srv::SetMotorMulti::Response> response) {
  response->success = true;
  switch (request->mode) {
  case 1: {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(request->positions[i]);
    }
    modbus_set_finger_positions(handle_, slave_id_, positions, 6);
  } break;
  case 2:
    int16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<int16_t>(request->speeds[i]);
    }
    modbus_set_finger_speeds(handle_, slave_id_, speeds, 6);
    break;
  case 3: // 仅支持二代手
    int16_t currents[6];
    for (int i = 0; i < 6; i++) {
      currents[i] = static_cast<int16_t>(request->currents[i]);
    }
    modbus_set_finger_currents(handle_, slave_id_, currents, 6);
    break;
  case 4: // 仅支持二代手
    int16_t pwms[6];
    for (int i = 0; i < 6; i++) {
      pwms[i] = static_cast<int16_t>(request->pwms[i]);
    }
    modbus_set_finger_pwms(handle_, slave_id_, pwms, 6);
    break;
  case 5: // 仅支持二代手
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(request->positions[i]);
    }
    uint16_t durations[6];
    for (int i = 0; i < 6; i++) {
      durations[i] = static_cast<uint16_t>(request->durations[i]);
    }
    modbus_set_finger_positions_and_durations(handle_, slave_id_, positions, durations, 6);
  } break;
  case 6: // 仅支持二代手
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(request->positions[i]);
    }
    uint16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<uint16_t>(request->speeds[i]);
    }
    modbus_set_finger_positions_and_speeds(handle_, slave_id_, positions, speeds, 6);
  } break;
  default:
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Invalid mode: %d", request->mode);
    break;
  }
}

void StarkNode::handle_set_motor_single(
    const std::shared_ptr<ros2_stark_interfaces::srv::SetMotorSingle::Request> request,
    std::shared_ptr<ros2_stark_interfaces::srv::SetMotorSingle::Response> response) {
  StarkFingerId finger_id = static_cast<StarkFingerId>(request->motor_id);
  response->success = true;
  switch (request->mode) {
  case 1:
    modbus_set_finger_position(handle_, slave_id_, finger_id, request->position);
    break;
  case 2:
    modbus_set_finger_speed(handle_, slave_id_, finger_id, request->speed);
    break;
  case 3: // 仅支持二代手
    /// 范围为-1000~1000或-最大电流~最大电流mA
    modbus_set_finger_current(handle_, slave_id_, finger_id, request->current);
    break;
  case 4: // 仅支持二代手
    /// 范围为-1000~1000
    modbus_set_finger_pwm(handle_, slave_id_, finger_id, request->pwm);
    break;
  case 5: // 仅支持二代手
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 期望时间范围为1~2000ms
    modbus_set_finger_position_with_millis(handle_, slave_id_, finger_id, request->position, request->duration);
    break;
  case 6: // 仅支持二代手
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 速度范围为1~1000或最小-最大速度(°/s）
    modbus_set_finger_position_with_speed(handle_, slave_id_, finger_id, request->position, request->speed);
    break;
  default:
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Invalid mode: %d", request->mode);
    break;
  }
}

void StarkNode::handle_motor_multi_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorMulti> msg) { //
  // Handle motor multi command
  switch (msg->mode) {
  case 1: {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(msg->positions[i]);
    }
    modbus_set_finger_positions(handle_, slave_id_, positions, 6);
  } break;
  case 2: {
    int16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<int16_t>(msg->speeds[i]);
    }
    modbus_set_finger_speeds(handle_, slave_id_, speeds, 6);
  } break;
  case 3: // 仅支持二代手
    int16_t currents[6];
    for (int i = 0; i < 6; i++) {
      currents[i] = static_cast<int16_t>(msg->currents[i]);
    }
    modbus_set_finger_currents(handle_, slave_id_, currents, 6);
    break;
  case 4: // 仅支持二代手
    int16_t pwms[6];
    for (int i = 0; i < 6; i++) {
      pwms[i] = static_cast<int16_t>(msg->pwms[i]);
    }
    modbus_set_finger_pwms(handle_, slave_id_, pwms, 6);
    break;
  case 5: // 仅支持二代手
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(msg->positions[i]);
    }
    uint16_t durations[6];
    for (int i = 0; i < 6; i++) {
      durations[i] = static_cast<uint16_t>(msg->durations[i]);
    }
    modbus_set_finger_positions_and_durations(handle_, slave_id_, positions, durations, 6);
  } break;
  case 6: // 仅支持二代手
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(msg->positions[i]);
    }
    uint16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<uint16_t>(msg->speeds[i]);
    }
    modbus_set_finger_positions_and_speeds(handle_, slave_id_, positions, speeds, 6);
  } break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid mode: %d", msg->mode);
    break;
  }
}

void StarkNode::handle_motor_single_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorSingle> msg) {
  // Handle motor single command
  StarkFingerId finger_id = static_cast<StarkFingerId>(msg->motor_id);
  switch (msg->mode) {
  case 1:
    modbus_set_finger_position(handle_, slave_id_, finger_id, msg->position);
    break;
  case 2:
    modbus_set_finger_speed(handle_, slave_id_, finger_id, msg->speed);
    break;
  case 3: // 仅支持二代手
    /// 范围为-1000~1000或-最大电流~最大电流mA
    modbus_set_finger_current(handle_, slave_id_, finger_id, msg->current);
    break;
  case 4: // 仅支持二代手
    /// 范围为-1000~1000
    modbus_set_finger_pwm(handle_, slave_id_, finger_id, msg->pwm);
    break;
  case 5: // 仅支持二代手
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 期望时间范围为1~2000ms
    modbus_set_finger_position_with_millis(handle_, slave_id_, finger_id, msg->position, msg->duration);
    break;
  case 6: // 仅支持二代手
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 速度范围为1~1000或最小-最大速度(°/s）
    modbus_set_finger_position_with_speed(handle_, slave_id_, finger_id, msg->position, msg->speed);
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Invalid mode: %d", msg->mode);
    break;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StarkNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}