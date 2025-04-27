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
find_package(ros2_stark_interfaces REQUIRED)

add_executable(stark_node src/stark_node.cpp)
target_include_directories(stark_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 生产阶段  
# target_link_libraries(stark_node bc_stark_sdk)
# find_library(STARK_SDK_LIBRARY NAMES bc_stark_sdk PATHS /usr/local/lib)
# target_link_libraries(stark_node ${STARK_SDK_LIBRARY})

# 开发阶段，放在项目内的 lib 目录
target_link_libraries(stark_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libbc_stark_sdk.so
)

# 避免每次手动设置 LD_LIBRARY_PATH，可以在链接时嵌入 RPATH（运行时库搜索路径）
set_target_properties(stark_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN"
)

ament_target_dependencies(stark_node
  rclcpp
  sensor_msgs
  std_msgs
  ros2_stark_interfaces
)

install(TARGETS stark_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libbc_stark_sdk.so
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Python 脚本
install(PROGRAMS
  scripts/stark_position_control.py
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
