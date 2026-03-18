cmake_minimum_required(VERSION 3.5)

project(hypa_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)

# 编译 rosout_bridge 节点
add_executable(rosout_bridge_node
  src/rosout_bridge_node.cpp
)

ament_target_dependencies(rosout_bridge_node
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
)

# 安装节点可执行文件
install(TARGETS rosout_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install project launch files
install(
  DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
