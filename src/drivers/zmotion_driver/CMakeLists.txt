cmake_minimum_required(VERSION 3.8)
project(zmotion_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(
    ZMOTION_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/zmotion_sdk"
    CACHE PATH "Path to ZMotion SDK (contains libzmotion.so, zmotion.h, zmcaux.h)"
)

find_library(
    ZMOTION_LIB
    NAMES zmotion
    PATHS ${ZMOTION_SDK_DIR}
    NO_DEFAULT_PATH
)

set(ZMOTION_AUX_SRC ${ZMOTION_SDK_DIR}/zmcaux.cpp)

if(NOT ZMOTION_LIB)
    message(FATAL_ERROR "Unable to find libzmotion.so in ${ZMOTION_SDK_DIR}")
endif()

if(NOT EXISTS ${ZMOTION_AUX_SRC})
    message(FATAL_ERROR "Unable to find zmcaux.cpp in ${ZMOTION_SDK_DIR}")
endif()

add_library(${PROJECT_NAME}_core
    ${ZMOTION_AUX_SRC}
    src/zmotion_sdk_wrapper.cpp
    src/zmotion_driver_node.cpp
)

set_source_files_properties(${ZMOTION_AUX_SRC} PROPERTIES COMPILE_OPTIONS "-w")

target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wno-comment)

target_include_directories(${PROJECT_NAME}_core
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ZMOTION_SDK_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_core
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    std_msgs
)

target_link_libraries(${PROJECT_NAME}_core
    ${ZMOTION_LIB}
)

set_target_properties(${PROJECT_NAME}_core PROPERTIES
    BUILD_RPATH "${ZMOTION_SDK_DIR}"
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
set_target_properties(${PROJECT_NAME}_node PROPERTIES
    BUILD_RPATH "${ZMOTION_SDK_DIR}"
    INSTALL_RPATH "\$ORIGIN"
)

add_executable(${PROJECT_NAME}_mimic_swing_test_node
    tests/zmotion_driver_mimic_swing_test_node.cpp
)

target_compile_features(${PROJECT_NAME}_mimic_swing_test_node PUBLIC cxx_std_17)
target_compile_definitions(${PROJECT_NAME}_mimic_swing_test_node PRIVATE
    ZMOTION_TEST_MIMIC_GROUP=1
)

ament_target_dependencies(${PROJECT_NAME}_mimic_swing_test_node
    rclcpp
    sensor_msgs
    std_msgs
)

install(
    TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_mimic_swing_test_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES ${ZMOTION_LIB}
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
