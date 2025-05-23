cmake_minimum_required(VERSION 3.10)
project(ros2_stark_interfaces)

# 设置 CMP0148 策略为 OLD，避免警告
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD) # rosidl_generate_interfaces
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/TouchStatusItem.msg"
  "msg/TouchStatus.msg"
  "msg/SetMotorSingle.msg"
  "msg/SetMotorMulti.msg"
  "srv/GetDeviceInfo.srv"
  "srv/SetMotorSingle.srv"
  "srv/SetMotorMulti.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



