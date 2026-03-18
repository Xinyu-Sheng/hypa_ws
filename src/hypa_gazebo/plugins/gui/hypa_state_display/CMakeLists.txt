if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Find dependencies
find_package(Qt5 COMPONENTS Core Gui Quick QuickControls2 REQUIRED)
find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Explicitly specify MOC headers
set(MOC_HEADERS
    include/hypa_state_display/HypaStateDisplayPlugin.hh
    include/hypa_state_display/HypaDataManager.hh
)

# Compile QML resources
qt5_add_resources(QML_RESOURCES hypa_state_display.qrc)

# Run MOC explicitly
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})

# Create plugin library
add_library(HypaStateDisplayPlugin SHARED
    src/HypaStateDisplayPlugin.cc
    src/HypaDataManager.cc
    ${MOC_SOURCES}
    ${QML_RESOURCES}
)

# Set position independent code for plugin
set_target_properties(HypaStateDisplayPlugin PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Link dependencies
ament_target_dependencies(HypaStateDisplayPlugin
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    rcl_interfaces
    std_msgs
    tf2
)

# Link order is critical: Qt first, then gz-gui
target_link_libraries(HypaStateDisplayPlugin
    Qt5::Core
    Qt5::Gui
    Qt5::Quick
    Qt5::QuickControls2
    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
)

# Install plugin library
install(TARGETS HypaStateDisplayPlugin
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Install config file
install(FILES HypaStateDisplayPlugin.config
    DESTINATION share/${PROJECT_NAME}/plugins/gui/hypa_state_display
)
