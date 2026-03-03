cmake_minimum_required(VERSION 3.8)
project(hwt9053_can_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)

# 数据解析库
add_library(hwt9053_parser
    src/hwt9053_parser.cpp
)

target_include_directories(hwt9053_parser
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hwt9053_parser
    sensor_msgs
)

# CAN 驱动节点（作为可组件化库）
add_library(hwt9053_can_driver_component SHARED
    src/hwt9053_can_driver_node.cpp
)

target_include_directories(hwt9053_can_driver_component
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hwt9053_can_driver_component
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    sensor_msgs
    ros2_socketcan
    can_msgs
)

target_link_libraries(hwt9053_can_driver_component
    hwt9053_parser
)

rclcpp_components_register_node(hwt9053_can_driver_component
    PLUGIN "hwt9053_can_driver::HWT9053CANDriverNode"
    EXECUTABLE hwt9053_can_driver_node
)

# 独立可执行文件
add_executable(hwt9053_can_driver_standalone
    src/main.cpp
)

target_include_directories(hwt9053_can_driver_standalone
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hwt9053_can_driver_standalone
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    ros2_socketcan
    can_msgs
)

target_link_libraries(hwt9053_can_driver_standalone
    hwt9053_parser
    hwt9053_can_driver_component
)

# 安装可执行文件
install(TARGETS
    hwt9053_can_driver_node
    hwt9053_can_driver_standalone
    DESTINATION lib/${PROJECT_NAME}
)

# 安装组件库
install(TARGETS hwt9053_can_driver_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装库
install(TARGETS hwt9053_parser
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include
)

# 安装 launch 文件
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
