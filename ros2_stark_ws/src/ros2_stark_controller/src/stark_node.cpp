#include "ros2_stark_controller/stark_node.hpp"

StarkNode::StarkNode() : Node("stark_node") {
    // Declare parameters
    declare_parameter("port", "/dev/ttyUSB1");
    declare_parameter("baudrate", 115200);
    declare_parameter("slave_id", 1);
    declare_parameter("firmware_type", STARK_FIRMWARE_TYPE_V1_TOUCH);  // Default V1Touch
    declare_parameter("protocol_type", STARK_PROTOCOL_TYPE_MODBUS);  // Default Mobbus
    declare_parameter("log_level", LOG_LEVEL_INFO);      // Default Info

    // Get parameters
    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    slave_id_ = get_parameter("slave_id").as_int();
    fw_type_ = static_cast<StarkFirmwareType>(get_parameter("firmware_type").as_int());
    protocol_type_ = static_cast<StarkProtocolType>(get_parameter("protocol_type").as_int());
    log_level_ = static_cast<LogLevel>(get_parameter("log_level").as_int());

    RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d, slave_id: %d, firmware_type: %d, protocol_type: %d, log_level: %d",
                port_.c_str(), baudrate_, slave_id_, fw_type_, protocol_type_, log_level_);

    // Initialize Stark SDK
    init_cfg(fw_type_, protocol_type_, log_level_);
    // list_available_ports();
    if (!initialize_modbus()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Modbus");
        throw std::runtime_error("Modbus initialization failed");
    }

    // Initialize publishers
    // joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    motor_status_pub_ = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status", 10);
    touch_status_pub_ = create_publisher<ros2_stark_interfaces::msg::TouchStatus>("touch_status", 10);
    turbo_mode_pub_ = create_publisher<std_msgs::msg::Bool>("turbo_mode", 10);
    voltage_pub_ = create_publisher<std_msgs::msg::UInt16>("voltage", 10);

    // Initialize subscribers
    joint_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", 10, std::bind(&StarkNode::joint_cmd_callback, this, std::placeholders::_1));
    force_level_sub_ = create_subscription<std_msgs::msg::Int8>(
        "force_level", 10, std::bind(&StarkNode::force_level_callback, this, std::placeholders::_1));
    turbo_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
        "turbo_mode_cmd", 10, std::bind(&StarkNode::turbo_mode_callback, this, std::placeholders::_1));

    // Initialize timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&StarkNode::timer_callback, this));

    // 设备电量等信息，不需要频繁更新    
    info_timer_ = create_wall_timer(
        std::chrono::seconds(30),
        std::bind(&StarkNode::info_timer_callback, this));
}

StarkNode::~StarkNode() {
    if (handle_) {
        modbus_close(handle_);
    }
}

bool StarkNode::initialize_modbus() {
    handle_ = modbus_open(port_.c_str(), baudrate_, slave_id_);
    if (!handle_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Modbus on %s", port_.c_str());
        return false;
    }
    return true;
}

void StarkNode::timer_callback() {
    // publish_joint_state();
    publish_motor_status();
    publish_touch_status();
}

void StarkNode::info_timer_callback() {
    publish_device_status();
}

void StarkNode::publish_joint_state() {
    auto motor_status = modbus_get_motor_status(handle_, slave_id_);
    if (!motor_status) {
        RCLCPP_WARN(this->get_logger(), "Failed to get motor status");
        return;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"thumb", "thumb_aux", "index", "middle", "ring", "pinky"};
    for (int i = 0; i < 6; i++) {
        msg.position.push_back(motor_status->positions[i] / 100.0);  // Normalize to 0-1
        msg.velocity.push_back(motor_status->speeds[i] / 100.0);     // Normalize to -1-1
        msg.effort.push_back(motor_status->currents[i]);
    }

    joint_state_pub_->publish(msg);
    free_motor_status_data(motor_status);
}

void StarkNode::publish_motor_status() {
    auto motor_status = modbus_get_motor_status(handle_, slave_id_);
    if (!motor_status) {
        RCLCPP_WARN(this->get_logger(), "Failed to get motor status");
        return;
    }

    auto msg = ros2_stark_interfaces::msg::MotorStatus();
    // 使用 std::copy 填充 std::array
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
    // 填充固定 5 个元素的数组, 代表 5个手指上对应的传感器信息
    for (int i = 0; i < 5; i++) {
        msg.data[i].normal_force1 = touch_status->data[i].normal_force1;
        msg.data[i].normal_force2 = touch_status->data[i].normal_force2;
        msg.data[i].normal_force3 = touch_status->data[i].normal_force3;
        msg.data[i].tangential_force1 = touch_status->data[i].tangential_force1;
        msg.data[i].tangential_force2 = touch_status->data[i].tangential_force2;
        msg.data[i].tangential_force3 = touch_status->data[i].tangential_force3;
        msg.data[i].tangential_direction1 = touch_status->data[i].tangential_direction1;
        msg.data[i].tangential_direction2 = touch_status->data[i].tangential_direction2;
        msg.data[i].tangential_direction3 = touch_status->data[i].tangential_direction3;
        msg.data[i].self_proximity1 = touch_status->data[i].self_proximity1;
        msg.data[i].self_proximity2 = touch_status->data[i].self_proximity2;
        msg.data[i].mutual_proximity = touch_status->data[i].mutual_proximity;
        msg.data[i].status = touch_status->data[i].status;
    }

    touch_status_pub_->publish(msg);
    free_touch_status_data(touch_status);
}

void StarkNode::publish_device_status() {
    // 电量
    auto voltage = modbus_get_voltage(handle_, slave_id_);
    auto voltage_msg = std_msgs::msg::UInt16();
    voltage_msg.data = voltage;
    voltage_pub_->publish(voltage_msg);

    // Turbo mode
    // auto turbo_mode = modbus_get_turbo_mode_enabled(handle_, slave_id_);
    // auto turbo_msg = std_msgs::msg::Bool();
    // turbo_msg.data = turbo_mode;
    // turbo_mode_pub_->publish(turbo_msg);
}

void StarkNode::joint_cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received joint commands");
    if (msg->position.size() >= 6) {
        uint16_t positions[6];
        for (int i = 0; i < 6; i++) {
            positions[i] = static_cast<uint16_t>(msg->position[i] * 100);  // Scale to 0-100
        }
        RCLCPP_INFO(this->get_logger(), "Joint positions: %d, %d, %d, %d, %d, %d",
                    positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);
        modbus_set_finger_positions(handle_, slave_id_, positions, 6);
    }
}

void StarkNode::force_level_callback(const std_msgs::msg::Int8::SharedPtr msg) {
    auto level = static_cast<ForceLevel>(msg->data);
    if (level >= FORCE_LEVEL_SMALL && level <= FORCE_LEVEL_FULL) {
        modbus_set_force_level(handle_, slave_id_, level);
    }
}

void StarkNode::turbo_mode_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    modbus_set_turbo_mode_enabled(handle_, slave_id_, msg->data);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StarkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}