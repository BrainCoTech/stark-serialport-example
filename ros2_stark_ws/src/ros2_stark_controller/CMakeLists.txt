cmake_minimum_required(VERSION 3.8)
project(ros2_stark_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_stark_msgs REQUIRED)

set(EXEC_NAME stark_node)
add_executable(${EXEC_NAME} src/${EXEC_NAME}.cpp)
target_include_directories(${EXEC_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 开发阶段，放在项目内的 lib 目录
target_link_libraries(${EXEC_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libbc_stark_sdk.so
)
# target_link_libraries(${EXEC_NAME}
#   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libusbcanfd.so # ZLG USB-CAN FD 库
# )

# 避免每次手动设置 LD_LIBRARY_PATH，可以在链接时嵌入 RPATH（运行时库搜索路径）
set_target_properties(${EXEC_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN"
)

ament_target_dependencies(${EXEC_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  ros2_stark_msgs
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

install(FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libbc_stark_sdk.so
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/stark_node_client.py
  DESTINATION lib/${PROJECT_NAME}
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
