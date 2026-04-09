cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Force Qt5 only (ROS2 强制依赖，不可修改) + 新增Widgets（QCustomPlot必需）
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Qml Quick QuickControls2)
set(QT_VERSION_MAJOR 5)

# 全自动查找 QCustomPlot（最优解，适配你的Ubuntu24.04）
find_package(QCustomPlot REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(HYPA_DT_GUI_PLUGIN_TARGET HypaDtGuiPlugin)

add_library(${HYPA_DT_GUI_PLUGIN_TARGET} SHARED
    src/HypaDtGuiPlugin.cc
    src/HypaDataManager.cc
    src/HypaQCustomPlotItem.cc
    src/HypaSelectionModel.cc
    include/hypa-dt-gui-plugin/HypaDtGuiPlugin.hh
    include/hypa-dt-gui-plugin/HypaDataManager.hh
    include/hypa-dt-gui-plugin/HypaQCustomPlotItem.hh
    include/hypa-dt-gui-plugin/HypaSelectionModel.hh
    hypa-dt-gui-plugin.qrc
)

target_include_directories(${HYPA_DT_GUI_PLUGIN_TARGET}
    PUBLIC
    include
)

ament_target_dependencies(${HYPA_DT_GUI_PLUGIN_TARGET}
    rclcpp
    sensor_msgs
)

target_link_libraries(${HYPA_DT_GUI_PLUGIN_TARGET}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Qml
    Qt5::Quick
    Qt5::QuickControls2
    QCustomPlot::QCustomPlot # 修复：官方CMake链接方式
    gz-gui${GZ_GUI_VER}::gz-gui${GZ_GUI_VER}
)

set_target_properties(${HYPA_DT_GUI_PLUGIN_TARGET} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

install(TARGETS ${HYPA_DT_GUI_PLUGIN_TARGET}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES hypa-dt-gui-plugin.config
    DESTINATION share/${PROJECT_NAME}/plugins/gui/hypa-dt-gui-plugin
)

install(DIRECTORY qml/
    DESTINATION share/${PROJECT_NAME}/plugins/gui/hypa-dt-gui-plugin/qml
    FILES_MATCHING PATTERN "*.qml"
)

add_executable(hypa-dt-gui-plugin-test-node
    tests/hypa-dt-gui-plugin_test_node.cpp
)

target_include_directories(hypa-dt-gui-plugin-test-node
    PRIVATE
    include
)

ament_target_dependencies(hypa-dt-gui-plugin-test-node
    rclcpp
    sensor_msgs
)

target_link_libraries(hypa-dt-gui-plugin-test-node
    Qt5::Core
)

install(TARGETS hypa-dt-gui-plugin-test-node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)