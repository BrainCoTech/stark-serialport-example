#include "ros2_ethercat_controller/stark_ec_node.hpp"
#include <sys/mman.h>
#include <ecrt.h>
#include <queue>

/****************************************************************************/
// 宏定义
/****************************************************************************/
#define FREQUENCY 1000 // 1000 Hz
#define CLOCK_TO_USE CLOCK_MONOTONIC

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
                       (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define STARK_VENDOR_ID     0x00bc0000
#define STARK_PRODUCT_CODE  0x00009252

// SDO配置参数索引定义
#define CONFIG_OBJECT_INDEX 0x8000

// 通用配置子索引
#define HAND_TYPE_SUBINDEX 0x01
#define LED_SWITCH_SUBINDEX 0x02
#define BUZZER_SWITCH_SUBINDEX 0x03
#define VIBRATOR_SWITCH_SUBINDEX 0x04
#define UNIT_MODE_SUBINDEX 0x05
#define POS_CALIBRATION_SUBINDEX 0x06
#define AUTO_CALIBRATION_SUBINDEX 0x07
#define TURBO_MODE_SUBINDEX 0x08
#define TURBO_PARAM_SUBINDEX 0x09

// 保护电流子索引
#define THUMB_FLEX_PRO_CUR_SUBINDEX 0x0A
#define THUMB_AUX_PRO_CUR_SUBINDEX 0x0B
#define INDEX_PRO_CUR_SUBINDEX 0x0C
#define MID_PRO_CUR_SUBINDEX 0x0D
#define RING_PRO_CUR_SUBINDEX 0x0E
#define PINK_PRO_CUR_SUBINDEX 0x0F
#define THUMB_AUX_LOCK_CUR_SUBINDEX 0x10

// 固件信息子索引
#define CTRL_WRIST_FW_VERSION_SUBINDEX 0x11
#define CTRL_SN_SUBINDEX 0x12
#define WRIST_FW_VERSION_SUBINDEX 0x13
#define WRIST_SN_SUBINDEX 0x14

/****************************************************************************/
// 枚举和结构体定义
/****************************************************************************/

// 控制模式枚举
typedef enum
{
  PositionAndTime = 1,
  PositionAndSpeed = 2,
  Speed = 3,
  Current = 4,
  Pwm = 5
} FingerCtrlMode;

// 固件信息结构体
typedef struct
{
  char ctrl_wrist_fw_version[21]; // 控制板固件版本, 20字节 + 1结束符
  char ctrl_sn[19];               // 控制板序列号, 18字节 + 1结束符
  char wrist_fw_version[21];      // 手腕板固件版本, 20字节 + 1结束符
  char wrist_sn[19];              // 手腕板序列号, 18字节 + 1结束符
} firmware_info_t;

/****************************************************************************/
// 全局变量
/****************************************************************************/

// EtherCAT 相关
std::thread ethercat_thread_;
// std::atomic<bool> ethercat_running_{false};
// std::atomic<bool> shutdown_requested_{false};
static bool cyclic_running_ = false;
static ec_master_t *master_ = NULL;
static ec_master_state_t master_state_ = {};
static ec_domain_t *domain_ = NULL;
static ec_domain_state_t domain_state_ = {};
static uint8_t *domain_data_ = NULL;
static std::queue<ros2_stark_interfaces::msg::SetMotorMulti> motor_multi_command_queue_;
static std::queue<ros2_stark_interfaces::msg::SetMotorSingle> motor_single_command_queue_;
static std::mutex command_queue_mutex_;
static uint8_t joint_pos_0[12];
static uint8_t joint_spd_0[12];
static uint8_t joint_cur_0[12];
static uint8_t joint_status_0[12];
static uint8_t joint_pos_1[12];
static uint8_t joint_spd_1[12];
static uint8_t joint_cur_1[12];
static uint8_t joint_status_1[12];
void run_motor_multi_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorMulti> msg);
void run_motor_single_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorSingle> msg);

// 计数器和时间
static unsigned int counter = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

// PDO 偏移量
static unsigned int off_out_0;
static unsigned int off_in_0;
static unsigned int off_out_1;
static unsigned int off_in_1;

/****************************************************************************/
// EtherCAT 配置
/****************************************************************************/

// PDO 域注册
// 从站0
const ec_pdo_entry_reg_t domain_regs_0[] = {
  {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x6000, 0x01, &off_in_0, NULL},  // 输入数据
  {0, 0, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x7000, 0x01, &off_out_0, NULL}, // 输出数据
  {}
};

// 从站1
const ec_pdo_entry_reg_t domain_regs_1[] = {
  {1, 1, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x6000, 0x01, &off_in_1, NULL},  // 输入数据
  {1, 1, STARK_VENDOR_ID, STARK_PRODUCT_CODE, 0x7000, 0x01, &off_out_1, NULL}, // 输出数据
  {}
};

// PDO 条目定义
ec_pdo_entry_info_t slave_pdo_entries[] = {
    // RxPDO: 0x1600 (主站发送给从站)
    {0x7000, 0x01, 16}, // mult_joint_ctrl_mode
    {0x7000, 0x02, 96}, // joint_param1 (positions)
    {0x7000, 0x03, 96}, // joint_param2 (durations/speeds)
    {0x7010, 0x01, 8},  // single_joint_ctrl_mode
    {0x7010, 0x02, 8},  // single_joint_id
    {0x7010, 0x03, 16}, // single_joint_param1
    {0x7010, 0x04, 16}, // single_joint_param2

    // TxPDO: 0x1A00 (从站发送给主站)
    {0x6000, 0x01, 96}, // joint_pos
    {0x6000, 0x02, 96}, // joint_spd
    {0x6000, 0x03, 96}, // joint_cur
    {0x6000, 0x04, 96}, // joint_status
};

// PDO 映射
ec_pdo_info_t slave_pdos[] = {
  {0x1600, 3, &slave_pdo_entries[0]}, // RxPDO: 0x7000:01-03
  {0x1601, 4, &slave_pdo_entries[3]}, // RxPDO: 0x7010:01-04
  {0x1A00, 4, &slave_pdo_entries[7]}, // TxPDO: 0x6000:01-04
};

// 同步管理器配置
ec_sync_info_t slave_syncs[] = {
  {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
  {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 2, &slave_pdos[0], EC_WD_ENABLE}, // SM2: RxPDO
  {3, EC_DIR_INPUT, 1, &slave_pdos[2], EC_WD_DISABLE}, // SM3: TxPDO
  {0xff},
};

/****************************************************************************/
// 函数声明
/****************************************************************************/
void print_hex(unsigned char *data, int len);
int get_mills();
struct timespec timespec_add(struct timespec time1, struct timespec time2);
void check_domain_state(void);
void check_master_state(void);
void start_cyclic_task(void);
void run_motor_command(void);

/****************************************************************************/
// 工具函数
/****************************************************************************/

// 获取关节名称字符串
const char *get_joint_name(StarkFingerId joint_id)
{
  switch (joint_id)
  {
  case STARK_FINGER_ID_THUMB:
    return "THUMB_FLEX";
  case STARK_FINGER_ID_THUMB_AUX:
    return "THUMB_ABDUCT";
  case STARK_FINGER_ID_INDEX:
    return "INDEX_FINGER";
  case STARK_FINGER_ID_MIDDLE:
    return "MIDDLE_FINGER";
  case STARK_FINGER_ID_RING:
    return "RING_FINGER";
  case STARK_FINGER_ID_PINKY:
    return "PINKY";
  default:
    return "UNKNOWN";
  }
}

int get_mills()
{
  struct timespec ts;
  clock_gettime(CLOCK_TO_USE, &ts);
  return TIMESPEC2NS(ts) / 1000000;
}

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
  struct timespec result;

  if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC)
  {
    result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
  }
  else
  {
    result.tv_sec = time1.tv_sec + time2.tv_sec;
    result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
  }

  return result;
}

void print_hex(unsigned char *data, int len)
{
  for (int i = 0; i < len; i++)
  {
    printf("0x%02X", data[i]);
    if (i < len - 1)
    {
      printf(" ");
    }
  }
  printf("\n");
}

/****************************************************************************/
// EtherCAT 状态检查函数
/****************************************************************************/

void check_domain_state(void)
{
  ec_domain_state_t ds;

  ecrt_domain_state(domain_, &ds);

  if (ds.working_counter != domain_state_.working_counter)
    printf("domain: WC %u.\n", ds.working_counter);
  if (ds.wc_state != domain_state_.wc_state)
    printf("domain: State %u.\n", ds.wc_state);

  domain_state_ = ds;
}

void check_master_state(void)
{
  ec_master_state_t ms;
  ecrt_master_state(master_, &ms);

  if (ms.slaves_responding != master_state_.slaves_responding)
    printf("%u slave(s).\n", ms.slaves_responding);
  if (ms.al_states != master_state_.al_states)
  {
    printf("AL states: 0x%02X.\n", ms.al_states);
    bool slaves_operational = (ms.al_states == 0x08);
    if (slaves_operational)
    {
      printf("All slaves now OPERATIONAL.\n");
    }
  }
  if (ms.link_up != master_state_.link_up)
    printf("Link is %s.\n", ms.link_up ? "UP" : "DOWN");

  master_state_ = ms;
}

/****************************************************************************/
// 多关节控制函数
/****************************************************************************/

// 1. 位置和时间控制模式
void set_position_and_time_mode(uint8_t slave_pos, uint16_t *positions, uint16_t *durations)
{
  // if (operational_state_ != OPERATIONAL_STATE)

  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;

  // 写入控制数据到EtherCAT域
  EC_WRITE_U16(domain_data_ + off_out, PositionAndTime);
  memcpy(domain_data_ + off_out + 2, positions, 12);  // positions
  memcpy(domain_data_ + off_out + 14, durations, 12); // durations

  printf("slave[%d] Set PositionAndTime mode:\n", slave_pos);
  printf("Positions: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", positions[i]);
  }
  printf("\nDurations: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", durations[i]);
  }
  printf("\n");
}

// 2. 位置和速度控制模式
void set_position_and_speed_mode(uint8_t slave_pos, uint16_t *positions, uint16_t *speeds)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;

  // 写入控制数据
  EC_WRITE_U16(domain_data_ + off_out, PositionAndSpeed);
  memcpy(domain_data_ + off_out + 2, positions, 12); // positions
  memcpy(domain_data_ + off_out + 14, speeds, 12);   // speeds

  printf("Set PositionAndSpeed mode:\n");
  printf("Positions: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", positions[i]);
  }
  printf("\nSpeeds: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", speeds[i]);
  }
  printf("\n");
}

// 3. 速度控制模式
void set_speed_mode(uint8_t slave_pos, int16_t *speeds)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;

  // 写入控制数据
  EC_WRITE_U16(domain_data_ + off_out, Speed);
  memcpy(domain_data_ + off_out + 2, speeds, 12); // speeds
  memset(domain_data_ + off_out + 14, 0, 12);     // param2 清零

  printf("Set Speed mode:\n");
  printf("Speeds: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", speeds[i]);
  }
  printf("\n");
}

// 4. 电流控制模式
void set_current_mode(uint8_t slave_pos, int16_t *currents)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;

  // 写入控制数据
  EC_WRITE_U16(domain_data_ + off_out, Current);
  memcpy(domain_data_ + off_out + 2, currents, 12); // currents
  memset(domain_data_ + off_out + 14, 0, 12);       // param2 清零

  printf("Set Current mode:\n");
  printf("Currents: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", currents[i]);
  }
  printf("\n");
}

// 5. PWM控制模式
void set_pwm_mode(uint8_t slave_pos, int16_t *pwm_values)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;

  // 写入控制数据
  EC_WRITE_U16(domain_data_ + off_out, Pwm);
  memcpy(domain_data_ + off_out + 2, pwm_values, 12); // pwm_values
  memset(domain_data_ + off_out + 14, 0, 12);         // param2 清零

  printf("Set PWM mode:\n");
  printf("PWM values: ");
  for (int i = 0; i < 6; i++)
  {
    printf("%d ", pwm_values[i]);
  }
  printf("\n");
}

/****************************************************************************/
// 单关节控制函数
/****************************************************************************/

// 单关节位置和时间控制
void set_single_joint_position_time(uint8_t slave_pos, StarkFingerId joint_id, uint16_t position, uint16_t duration)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data_ + off_out + single_offset, PositionAndTime);
  EC_WRITE_U8(domain_data_ + off_out + single_offset + 1, (uint8_t)joint_id-1);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 2, position);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 4, duration);

  printf("Single joint %s(%d): PositionAndTime, pos=%d, time=%d\n",
         get_joint_name(joint_id), joint_id, position, duration);
}

// 单关节位置和速度控制
void set_single_joint_position_speed(uint8_t slave_pos, StarkFingerId joint_id, uint16_t position, uint16_t speed)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data_ + off_out + single_offset, PositionAndSpeed);
  EC_WRITE_U8(domain_data_ + off_out + single_offset + 1, (uint8_t)joint_id-1);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 2, position);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 4, speed);

  printf("Single joint %s(%d): PositionAndSpeed, pos=%d, speed=%d\n",
         get_joint_name(joint_id), joint_id, position, speed);
}

// 单关节速度控制
void set_single_joint_speed(uint8_t slave_pos, StarkFingerId joint_id, uint16_t speed)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;

  size_t single_offset = 26;
  EC_WRITE_U8(domain_data_ + off_out + single_offset, Speed);
  EC_WRITE_U8(domain_data_ + off_out + single_offset + 1, (uint8_t)joint_id-1);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 2, speed);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 4, 0);

  printf("Single joint %s(%d): Speed, speed=%d\n",
         get_joint_name(joint_id), joint_id, speed);
}

// 单关节电流控制
void set_single_joint_current(uint8_t slave_pos, StarkFingerId joint_id, uint16_t current)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data_ + off_out + single_offset, Current);
  EC_WRITE_U8(domain_data_ + off_out + single_offset + 1, (uint8_t)joint_id-1);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 2, current);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 4, 0);

  printf("Single joint %s(%d): Current, current=%d\n",
         get_joint_name(joint_id), joint_id, current);
}

// 单关节PWM控制
void set_single_joint_pwm(uint8_t slave_pos, StarkFingerId joint_id, uint16_t pwm_value)
{
  unsigned int off_out = (slave_pos == 0) ? off_out_0 : off_out_1;
  size_t single_offset = 26;
  EC_WRITE_U8(domain_data_ + off_out + single_offset, Pwm);
  EC_WRITE_U8(domain_data_ + off_out + single_offset + 1, (uint8_t)joint_id-1);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 2, pwm_value);
  EC_WRITE_U16(domain_data_ + off_out + single_offset + 4, 0);

  printf("Single joint %s(%d): PWM, value=%d\n",
         get_joint_name(joint_id), joint_id, pwm_value);
}

/****************************************************************************/
// 反馈数据读取函数
/****************************************************************************/

void get_motor_status()
{
  memcpy(joint_pos_0, domain_data_ + off_in_0, 12);
  memcpy(joint_spd_0, domain_data_ + off_in_0 + 12, 12);
  memcpy(joint_cur_0, domain_data_ + off_in_0 + 24, 12);
  memcpy(joint_status_0, domain_data_ + off_in_0 + 36, 12);
  // printf("Motor Status 0 - Pos: ");
  // for (int i = 0; i < 6; i++) {
  //   uint16_t pos = *((uint16_t *)(joint_pos_0 + i * 2));
  //   printf("%d ", pos);
  // }
  // printf("\n");

  if (off_in_1 > 0) {
    memcpy(joint_pos_1, domain_data_ + off_in_1, 12);
    memcpy(joint_spd_1, domain_data_ + off_in_1 + 12, 12);
    memcpy(joint_cur_1, domain_data_ + off_in_1 + 24, 12);
    memcpy(joint_status_1, domain_data_ + off_in_1 + 36, 12);
  }
}

StarkNode::StarkNode() : Node("stark_ec_node") {
  // Declare and Get parameters
  slave_pos_left = this->declare_parameter<int>("slave_pos_left", 0);
  slave_pos_right = this->declare_parameter<int>("slave_pos_right", 0);
  log_level_ = static_cast<LogLevel>(this->declare_parameter<int>("log_level", 2));
  RCLCPP_INFO(this->get_logger(), "slave_pos_left: %d, slave_pos_right: %d, log_level: %d", slave_pos_left, slave_pos_right, log_level_);

  // Initialize Stark SDK
  if (!initialize_stark_handler()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Stark handler");
    throw std::runtime_error("Stark handler initialization failed");
  }

  // if (fw_type_left == StarkHardwareType::STARK_HARDWARE_TYPE_REVO2_TOUCH) {
  //   // 启用全部触觉传感器
  //   stark_enable_touch_sensor(slave_pos_left, 0x1F);
  //   // usleep(1000 * 1000); // wait for touch sensor to be ready
  // }

  // Initialize services
  std::string str_left = std::to_string(slave_pos_left);
  std::string str_right = std::to_string(slave_pos_left);
  // get_device_info_service_left = create_service<ros2_stark_interfaces::srv::GetDeviceInfo>(
  //     "get_device_info_" + str_left,
  //     std::bind(&StarkNode::handle_get_device_info, this, std::placeholders::_1, std::placeholders::_2));
  // get_device_info_service_right = create_service<ros2_stark_interfaces::srv::GetDeviceInfo>(
  //     "get_device_info_" + str_right,
  //     std::bind(&StarkNode::handle_get_device_info, this, std::placeholders::_1, std::placeholders::_2));     
  set_motor_multi_service_left = create_service<ros2_stark_interfaces::srv::SetMotorMulti>(
      "set_motor_multi_" + str_left,
      std::bind(&StarkNode::handle_set_motor_multi, this, std::placeholders::_1, std::placeholders::_2));
  set_motor_single_service_left = create_service<ros2_stark_interfaces::srv::SetMotorSingle>(
      "set_motor_single_" + str_left,
      std::bind(&StarkNode::handle_set_motor_single, this, std::placeholders::_1, std::placeholders::_2));
  
  // Initialize publishers
  motor_status_pub_left = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status_left", 10);
  // touch_status_pub_left = create_publisher<ros2_stark_interfaces::msg::TouchStatus>("touch_status_left", 10);
  
  // Initialize subscribers
  motor_multi_sub_left = create_subscription<ros2_stark_interfaces::msg::SetMotorMulti>(
      "set_motor_multi_" + str_left, 10, std::bind(&StarkNode::queue_motor_multi_command, this, std::placeholders::_1));
  motor_single_sub_left = create_subscription<ros2_stark_interfaces::msg::SetMotorSingle>(
      "set_motor_single_" + str_left, 10, std::bind(&StarkNode::queue_motor_single_command, this, std::placeholders::_1));

  if (slave_pos_right > 0) {
    set_motor_multi_service_right = create_service<ros2_stark_interfaces::srv::SetMotorMulti>(
      "set_motor_multi_" + str_right,
      std::bind(&StarkNode::handle_set_motor_multi, this, std::placeholders::_1, std::placeholders::_2));
    set_motor_single_service_right = create_service<ros2_stark_interfaces::srv::SetMotorSingle>(
      "set_motor_single_" + str_left,
      std::bind(&StarkNode::handle_set_motor_single, this, std::placeholders::_1, std::placeholders::_2));

    motor_status_pub_right = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status_right", 10);
    // touch_status_pub_right = create_publisher<ros2_stark_interfaces::msg::TouchStatus>("touch_status_right", 10);

    motor_multi_sub_right = create_subscription<ros2_stark_interfaces::msg::SetMotorMulti>(
      "set_motor_multi_" + str_right, 10, std::bind(&StarkNode::queue_motor_multi_command, this, std::placeholders::_1));
    motor_single_sub_right = create_subscription<ros2_stark_interfaces::msg::SetMotorSingle>(
      "set_motor_single_" + str_right, 10, std::bind(&StarkNode::queue_motor_single_command, this, std::placeholders::_1));
  }

  // Initialize timer
  timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&StarkNode::timer_callback, this));

  // 设备电量等信息，不需要频繁更新
  // info_timer_ = create_wall_timer(
  //     std::chrono::seconds(30),
  //     std::bind(&StarkNode::info_timer_callback, this));

  // 在单独线程中启动EtherCAT循环
  ethercat_thread_ = std::thread(start_cyclic_task);
  RCLCPP_INFO(this->get_logger(), "EtherCAT communication thread started");
}

StarkNode::~StarkNode() {
  cyclic_running_ = false;
  // cleanup();
}

// 读取STRING类型SDO
int read_sdo_string(uint8_t pos, uint16_t index, uint8_t subindex, char *value, size_t max_size)
{
  size_t result_size;
  uint32_t abort_code;

  int ret = ecrt_master_sdo_upload(master_, pos, index, subindex,
                                   (uint8_t *)value, max_size - 1,
                                   &result_size, &abort_code);

  if (ret)
  {
    printf("Failed to read SDO pos:%d 0x%04X:0x%02X - Error: %d, Abort code: 0x%08X\n",
           pos, index, subindex, ret, abort_code);
    return ret;
  }

  value[result_size] = '\0'; // 确保字符串结束
  printf("Read SDO pos:%d 0x%04X:0x%02X = \"%s\"\n", pos, index, subindex, value);
  return 0;
}

// 读取固件信息
int read_firmware_info(uint8_t pos, firmware_info_t *info)
{
  int ret = 0;

  printf("=== Reading Firmware Information ===\n");

  ret |= read_sdo_string(pos, CONFIG_OBJECT_INDEX, CTRL_WRIST_FW_VERSION_SUBINDEX,
                         info->ctrl_wrist_fw_version, sizeof(info->ctrl_wrist_fw_version));
  ret |= read_sdo_string(pos, CONFIG_OBJECT_INDEX, CTRL_SN_SUBINDEX,
                         info->ctrl_sn, sizeof(info->ctrl_sn));
  ret |= read_sdo_string(pos, CONFIG_OBJECT_INDEX, WRIST_FW_VERSION_SUBINDEX,
                         info->wrist_fw_version, sizeof(info->wrist_fw_version));
  ret |= read_sdo_string(pos, CONFIG_OBJECT_INDEX, WRIST_SN_SUBINDEX,
                         info->wrist_sn, sizeof(info->wrist_sn));

  printf("====================================\n");
  return ret;
}

bool StarkNode::initialize_stark_handler() {
  

  // 初始化 EtherCAT 主站
  printf("Requesting master_...\n");
  master_ = ecrt_request_master(0);
  if (!master_)
    return -1;

  printf("Creating domain...\n");
  domain_ = ecrt_master_create_domain(master_);
  if (!domain_)
    return -1;

  // 创建从站配置
  printf("Requesting slave[%d]...\n", slave_pos_left);
  ec_slave_config_t *sc_0;
  if (!(sc_0 = ecrt_master_slave_config(master_, slave_pos_left, slave_pos_left, STARK_VENDOR_ID, STARK_PRODUCT_CODE)))
  {
    fprintf(stderr, "Failed to get slave[%d] configuration.\n", slave_pos_left);
    return -1;
  }
  printf("Configuring PDOs...\n");
  if (ecrt_slave_config_pdos(sc_0, EC_END, slave_syncs))
  {
    fprintf(stderr, "Failed to configure PDOs.\n");
    return -1;
  }
  printf("Registering PDO entries...\n");
  if (ecrt_domain_reg_pdo_entry_list(domain_, domain_regs_0))
  {
    fprintf(stderr, "PDO entry registration failed!\n");
    return -1;
  }
  printf("PDO entry registration successful, off_in: %d, off_out: %d\n", off_in_0, off_out_0);
  printf("Slave[%d] configuration successful.\n", slave_pos_left);

  if (slave_pos_right > 0) {
    printf("Requesting slave[%d]...\n", slave_pos_right);
    ec_slave_config_t *sc_1;
    if (!(sc_1 = ecrt_master_slave_config(master_, slave_pos_right, slave_pos_right, STARK_VENDOR_ID, STARK_PRODUCT_CODE)))
    {
      fprintf(stderr, "Failed to get slave[%d] configuration.\n", slave_pos_right);
      return -1;
    }
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_1, EC_END, slave_syncs))
    {
      fprintf(stderr, "Failed to configure PDOs.\n");
      return -1;
    }
    printf("Registering PDO entries...\n");
    if (ecrt_domain_reg_pdo_entry_list(domain_, domain_regs_1))
    {
      fprintf(stderr, "PDO entry registration failed!\n");
      return -1;
    }
    printf("PDO entry registration successful, off_in: %d, off_out: %d\n", off_in_1, off_out_1);
    printf("Slave[%d] configuration successful.\n", slave_pos_right);
  }

  // 读取固件信息
  firmware_info_t fw_info_0;
  if (read_firmware_info(slave_pos_left, &fw_info_0) == 0)
  {
    printf("\n slave[%d] Firmware Information:\n", slave_pos_left);
    printf("  Control Board FW: %s\n", fw_info_0.ctrl_wrist_fw_version);
    printf("  Control Board SN: %s\n", fw_info_0.ctrl_sn);
    printf("  Wrist Board FW:   %s\n", fw_info_0.wrist_fw_version);
    printf("  Wrist Board SN:   %s\n", fw_info_0.wrist_sn);
  }
  if (slave_pos_right > 0) {
    firmware_info_t fw_info_1;
    if (read_firmware_info(slave_pos_right, &fw_info_1) == 0)
    {
      printf("\n slave[%d] Firmware Information:\n", slave_pos_right);
      printf("  Control Board FW: %s\n", fw_info_1.ctrl_wrist_fw_version);
      printf("  Control Board SN: %s\n", fw_info_1.ctrl_sn);
      printf("  Wrist Board FW:   %s\n", fw_info_1.wrist_fw_version);
      printf("  Wrist Board SN:   %s\n", fw_info_1.wrist_sn);
   }
  }
  return true;
}

void StarkNode::timer_callback() {
  publish_motor_status();
  // if (touch_device) {
  //   publish_touch_status();
  // }
}

void StarkNode::publish_motor_status() {
  auto msg = ros2_stark_interfaces::msg::MotorStatus();
  msg.slave_id = slave_pos_left;
  for (int i = 0; i < 6; i++) {
    uint16_t pos = *((uint16_t *)(joint_pos_0 + i * 2));
    uint16_t spd = *((uint16_t *)(joint_spd_0 + i * 2));
    uint16_t cur = *((uint16_t *)(joint_cur_0 + i * 2));
    uint16_t status = *((uint16_t *)(joint_status_0 + i * 2));
    msg.positions[i] = pos;
    msg.speeds[i] = spd;
    msg.currents[i] = cur;
    msg.states[i] = status;
  }
  motor_status_pub_left->publish(msg);

  if (slave_pos_right <= 0) return;
  auto msg_right = ros2_stark_interfaces::msg::MotorStatus();
  msg_right.slave_id = slave_pos_right;
  for (int i = 0; i < 6; i++) {
    uint16_t pos = *((uint16_t *)(joint_pos_1 + i * 2));
    uint16_t spd = *((uint16_t *)(joint_spd_1 + i * 2));
    uint16_t cur = *((uint16_t *)(joint_cur_1 + i * 2));
    uint16_t status = *((uint16_t *)(joint_status_1 + i * 2));
    msg_right.positions[i] = pos;
    msg_right.speeds[i] = spd;
    msg_right.currents[i] = cur;
    msg_right.states[i] = status;
  }
  motor_status_pub_right->publish(msg_right);
}

// void StarkNode::publish_touch_status() {
//   auto touch_status = stark_get_touch_status(slave_pos_left);
//   if (!touch_status) {
//     RCLCPP_WARN(this->get_logger(), "Failed to get slave[%d]  touch status", slave_pos_left);
//     return;
//   }
//   auto msg = ros2_stark_interfaces::msg::TouchStatus();
//   msg.slave_pos = slave_pos_left;
//   // 填充固定 5 个元素的数组, 代表 5个手指上对应的传感器信息
//   for (int i = 0; i < 5; i++) {
//     msg.data[i].normal_force1 = touch_status->items[i].normal_force1;
//     msg.data[i].tangential_force1 = touch_status->items[i].tangential_force1;
//     msg.data[i].tangential_direction1 = touch_status->items[i].tangential_direction1;
//     msg.data[i].self_proximity1 = touch_status->items[i].self_proximity1;
//     msg.data[i].status = touch_status->items[i].status;
//   }
//   touch_status_pub_->publish(msg);
//   free_touch_finger_data(touch_status);
// }

// void StarkNode::handle_get_device_info(
//     const std::shared_ptr<ros2_stark_interfaces::srv::GetDeviceInfo::Request> request,
//     std::shared_ptr<ros2_stark_interfaces::srv::GetDeviceInfo::Response> response) {
//   uint8_t slave_pos = request->slave_id;  
//   bool is_left = (slave_pos == slave_pos_left);  
//   response->sku_type = static_cast<uint8_t>(is_left ? sku_type_left : sku_type_right);
//   response->serial_number = is_left ? sn_left : sn_right;
//   response->firmware_version = is_left ? fw_version_left : fw_version_right;
//   if (request->get_turbo_mode) {
//     is_turbo_mode_enabled = stark_get_turbo_mode_enabled(slave_id);
//     response->turbo_mode = is_turbo_mode_enabled;
//   }
//   response->success = true;
// }

void StarkNode::handle_set_motor_multi(
    const std::shared_ptr<ros2_stark_interfaces::srv::SetMotorMulti::Request> request,
    std::shared_ptr<ros2_stark_interfaces::srv::SetMotorMulti::Response> response) {
  uint8_t slave_pos = request->slave_id;
  printf("slave[%d] Received multi motor command: mode=%d\n", slave_pos, request->mode);
  response->success = true;
  switch (request->mode) {
  case 1: {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(request->positions[i]);
    }
    uint16_t durations[6] {300, 300, 300, 300, 300, 300};
    set_position_and_time_mode(slave_pos, positions, durations);
  } break;
  case 2:
    int16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<int16_t>(request->speeds[i]);
    }
    set_speed_mode(slave_pos, speeds);
    break;
  case 3: 
    int16_t currents[6];
    for (int i = 0; i < 6; i++) {
      currents[i] = static_cast<int16_t>(request->currents[i]);
    }
    set_current_mode(slave_pos, currents);
    break;
  case 4: 
    int16_t pwms[6];
    for (int i = 0; i < 6; i++) {
      pwms[i] = static_cast<int16_t>(request->pwms[i]);
    }
    set_pwm_mode(slave_pos, pwms);
    break;
  case 5:
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(request->positions[i]);
    }
    uint16_t durations[6];
    for (int i = 0; i < 6; i++) {
      durations[i] = static_cast<uint16_t>(request->durations[i]);
    }
    set_position_and_time_mode(slave_pos, positions, durations);
  } break;
  case 6:
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(request->positions[i]);
    }
    uint16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<uint16_t>(request->speeds[i]);
    }
    set_position_and_speed_mode(slave_pos, positions, speeds);
  } break;
  default:
    response->success = false;
    printf("Invalid mode: %d\n", request->mode);
    break;
  }
}

void StarkNode::handle_set_motor_single(
    const std::shared_ptr<ros2_stark_interfaces::srv::SetMotorSingle::Request> request,
    std::shared_ptr<ros2_stark_interfaces::srv::SetMotorSingle::Response> response) {
  uint8_t slave_pos = request->slave_id;
  StarkFingerId finger_id = static_cast<StarkFingerId>(request->motor_id);
  response->success = true;
  switch (request->mode) {
  case 1:
    set_single_joint_position_time(slave_pos, finger_id, request->position, 300);
    break;
  case 2:
    set_single_joint_speed(slave_pos, finger_id, request->speed);
    break;
  case 3:
    /// 范围为-1000~1000或-最大电流~最大电流mA
    set_single_joint_current(slave_pos, finger_id, request->current);
    break;
  case 4:
    /// 范围为-1000~1000
    set_single_joint_pwm(slave_pos, finger_id, request->pwm);
    break;
  case 5:
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 期望时间范围为1~2000ms
    set_single_joint_position_time(slave_pos, finger_id, request->position, request->duration);
    break;
  case 6:
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 速度范围为1~1000或最小-最大速度(°/s）
    set_single_joint_position_speed(slave_pos, finger_id, request->position, request->speed);
    break;
  default:
    response->success = false;
    printf("Invalid mode: %d\n", request->mode);
    break;
  }
}

void StarkNode::queue_motor_multi_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorMulti> msg) {
  printf("slave[%d] Received multi motor command: mode=%d\n", msg->slave_id, msg->mode);
  std::lock_guard<std::mutex> lock(command_queue_mutex_);
    
  // 将消息添加到队列中
  motor_multi_command_queue_.push(*msg);
  
  // 可选：限制队列大小，防止内存溢出
  const size_t MAX_QUEUE_SIZE = 100;
  while (motor_multi_command_queue_.size() > MAX_QUEUE_SIZE) {
      motor_multi_command_queue_.pop();
      RCLCPP_WARN(this->get_logger(), "Motor command queue overflow, dropping oldest command");
  }
  
  RCLCPP_DEBUG(this->get_logger(), 
              "Queued motor multi command for slave %d, mode %d, queue size: %zu", 
              msg->slave_id, msg->mode, motor_multi_command_queue_.size());
}

void StarkNode::queue_motor_single_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorSingle> msg) {
  printf("slave[%d] Received single motor command: motor_id=%d, mode=%d\n", msg->slave_id, msg->motor_id, msg->mode);
  std::lock_guard<std::mutex> lock(command_queue_mutex_);
    
  // 将消息添加到队列中
  motor_single_command_queue_.push(*msg);
  
  // 可选：限制队列大小，防止内存溢出
  const size_t MAX_QUEUE_SIZE = 100;
  while (motor_single_command_queue_.size() > MAX_QUEUE_SIZE) {
      motor_single_command_queue_.pop();
      RCLCPP_WARN(this->get_logger(), "Motor command queue overflow, dropping oldest command");
  }
  
  RCLCPP_DEBUG(this->get_logger(), 
              "Queued motor single command for slave %d, motor %d, mode %d, queue size: %zu", 
              msg->slave_id, msg->motor_id, msg->mode, motor_single_command_queue_.size());
}

void run_motor_command() {
  // 只取一条
  std::lock_guard<std::mutex> lock(command_queue_mutex_);
  if (!motor_multi_command_queue_.empty()) {
      auto msg = motor_multi_command_queue_.front();
      motor_multi_command_queue_.pop();
      printf("Processing queued motor multi command for slave %d, mode %d, remaining queue size: %zu\n", msg.slave_id, msg.mode, motor_multi_command_queue_.size());
      run_motor_multi_command(std::make_shared<ros2_stark_interfaces::msg::SetMotorMulti>(msg));
      return;
  }
  if (!motor_single_command_queue_.empty()) {
      auto msg = motor_single_command_queue_.front();
      motor_single_command_queue_.pop();
      printf("Processing queued motor single command for slave %d, motor %d, mode %d, remaining queue size: %zu\n", msg.slave_id, msg.motor_id, msg.mode, motor_single_command_queue_.size());
      run_motor_single_command(std::make_shared<ros2_stark_interfaces::msg::SetMotorSingle>(msg));
      return;
  }
}

void run_motor_multi_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorMulti> msg) { 
  uint8_t slave_pos = msg->slave_id;
  // Handle motor multi command
  switch (msg->mode) {
  case 1: {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(msg->positions[i]);
    }
    uint16_t durations[6] {300, 300, 300, 300, 300, 300};
    set_position_and_time_mode(slave_pos, positions, durations);
  } break;
  case 2: {
    int16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<int16_t>(msg->speeds[i]);
    }
    set_speed_mode(slave_pos, speeds);
  } break;
  case 3:
    int16_t currents[6];
    for (int i = 0; i < 6; i++) {
      currents[i] = static_cast<int16_t>(msg->currents[i]);
    }
    set_current_mode(slave_pos, currents);
    break;
  case 4:
    int16_t pwms[6];
    for (int i = 0; i < 6; i++) {
      pwms[i] = static_cast<int16_t>(msg->pwms[i]);
    }
    set_pwm_mode(slave_pos, pwms);
    break;
  case 5:
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(msg->positions[i]);
    }
    uint16_t durations[6];
    for (int i = 0; i < 6; i++) {
      durations[i] = static_cast<uint16_t>(msg->durations[i]);
    }
    set_position_and_time_mode(slave_pos, positions, durations);
  } break;
  case 6:
  {
    uint16_t positions[6];
    for (int i = 0; i < 6; i++) {
      positions[i] = static_cast<uint16_t>(msg->positions[i]);
    }
    uint16_t speeds[6];
    for (int i = 0; i < 6; i++) {
      speeds[i] = static_cast<uint16_t>(msg->speeds[i]);
    }
    set_position_and_speed_mode(slave_pos, positions, speeds);
  } break;
  default:
    printf("Invalid mode: %d\n", msg->mode);
    break;
  }
}

void run_motor_single_command(const std::shared_ptr<ros2_stark_interfaces::msg::SetMotorSingle> msg) {
  uint8_t slave_pos = msg->slave_id;
  // Handle motor single command
  StarkFingerId finger_id = static_cast<StarkFingerId>(msg->motor_id);
  switch (msg->mode) {
  case 1:
    set_single_joint_position_time(slave_pos, finger_id, msg->position, 300);
    break;
  case 2:
    set_single_joint_speed(slave_pos, finger_id, msg->speed);
    break;
  case 3:
    /// 范围为-1000~1000或-最大电流~最大电流mA
    set_single_joint_current(slave_pos, finger_id, msg->current);
    break;
  case 4:
    /// 范围为-1000~1000
    set_single_joint_pwm(slave_pos, finger_id, msg->pwm);
    break;
  case 5:
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 期望时间范围为1~2000ms
    set_single_joint_position_time(slave_pos, finger_id, msg->position, msg->duration);
    break;
  case 6:
    /// 位置范围为0~1000或最小-最大位置（°）
    /// 速度范围为1~1000或最小-最大速度(°/s）
    set_single_joint_position_speed(slave_pos, finger_id, msg->position, msg->speed);
    break;
  default:
    printf("Invalid mode: %d\n", msg->mode);
    break;
  }
}

// 全局变量方式（如果只有一个实例）
static StarkNode* g_stark_node = nullptr;

void cleanup() {
  cyclic_running_ = false;
  
  // 等待循环任务结束
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  // EtherCAT 清理
  if (master_) {
      ecrt_master_deactivate(master_);
      ecrt_release_master(master_);
      master_ = nullptr;
  }
}

void signal_handler(int sig) {
    RCLCPP_INFO(rclcpp::get_logger("stark_ec_node"), 
               "Received signal %d, cleaning up...", sig);
    
    // if (g_stark_node) {
    //   g_stark_node->cleanup();
    // }
    cleanup();
    
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StarkNode>();
  g_stark_node = node.get();

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGQUIT, signal_handler);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void start_cyclic_task() {
  // 内存锁定
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
  {
    perror("mlockall failed");
  }

  // 设置实时调度策略
  struct sched_param param = {};
  param.sched_priority = sched_get_priority_max(SCHED_FIFO);
  printf("Using priority %i.\n", param.sched_priority);
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
  {
    perror("sched_setscheduler failed");
  }

  printf("Activating master_...\n");
  if (ecrt_master_activate(master_))
    return;

  if (!(domain_data_ = ecrt_domain_data(domain_)))
  {
    return;
  }
  // printf("Domain data pointer: %p, size: %ld\n", domain_data_, ecrt_domain_size(domain_));

  printf("Starting cyclic function.\n");
  struct timespec wakeupTime, time;
  clock_gettime(CLOCK_TO_USE, &wakeupTime);

  cyclic_running_ = true;
  while (cyclic_running_ && rclcpp::ok())
  {
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // Write application time to master
    ecrt_master_application_time(master_, TIMESPEC2NS(wakeupTime));

    // receive process data
    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);

    // check process data state (optional)
    // check_domain_state();

    if (counter)
    {
      counter--;
    }
    else
    {
      counter = FREQUENCY;
      check_master_state();
    }

    // 仅当从站处于 OPERATIONAL 状态时才执行读写操作
    if (master_state_.al_states == 0x08)
    {
      // static int read_counter = 0;
      // if (read_counter++ % (FREQUENCY * 1) == 0)
      // { // 每秒读取一次
        get_motor_status();
      // }

      run_motor_command();
    }

    if (sync_ref_counter)
    {
      sync_ref_counter--;
    }
    else
    {
      sync_ref_counter = 1;
      clock_gettime(CLOCK_TO_USE, &time);
      ecrt_master_sync_reference_clock_to(master_, TIMESPEC2NS(time));
    }
    ecrt_master_sync_slave_clocks(master_);

    // send process data
    ecrt_domain_queue(domain_);
    ecrt_master_send(master_);
  }
}