cmake_minimum_required(VERSION 3.8)
project(hypa_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find Gazebo dependencies
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(
  include
)

# Create library targets (TODO: Uncomment when source files are created)
# add_library(${PROJECT_NAME}_core SHARED
# src/core/DataTypes.cpp
# )

# add_library(${PROJECT_NAME}_data SHARED
# src/data/GazeboDataProvider.cpp
# src/data/IDataProvider.cpp
# )

# Executable targets (暂时注释，稍后添加)
# add_executable(hypa_application_node
# src/nodes/hypa_application_node.cpp
# )

# Component targets (暂时注释，稍后添加)
# add_library(hypa_application_component SHARED
# src/components/HypaApplicationComponent.cpp
# )

# Link libraries (TODO: Uncomment when libraries are created)
# target_link_libraries(${PROJECT_NAME}_core
# ${ament_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}_data
# ${PROJECT_NAME}_core
# gz-transport13::gz-transport13
# gz-msgs10::gz-msgs10
# ${ament_LIBRARIES}
# )

# Register component
# rclcpp_components_register_nodes(hypa_application_component
# "hypa_application::HypaApplicationComponent"
# )

# Install targets (TODO: Uncomment when libraries are created)
# install(TARGETS
# ${PROJECT_NAME}_core
# ${PROJECT_NAME}_data
# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if they exist)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/
  )
endif()

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
