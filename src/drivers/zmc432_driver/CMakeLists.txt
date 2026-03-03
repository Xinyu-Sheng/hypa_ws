cmake_minimum_required(VERSION 3.8)
project(zmc432_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hypa_msgs REQUIRED)

# ZMotion SDK
set(ZMOTION_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
find_library(ZMOTION_LIB zmotion PATHS ${ZMOTION_SDK_DIR} REQUIRED)

# Executable
add_executable(motion_node
  src/motion_node.cpp
  src/motion_topic_node.cpp
  src/motion_controller.cpp
  src/zmotion_wrapper.cpp
  src/zmcaux.cpp
)

target_include_directories(motion_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/zmc432_driver
)

# 非 ROS 第三方库
target_link_libraries(motion_node
  ${ZMOTION_LIB}
)

ament_target_dependencies(motion_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  hypa_msgs
)

install(TARGETS motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
