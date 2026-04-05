cmake_minimum_required(VERSION 3.5)

# Following directive defines the project name.
project(hypa_gazebo)

# Following directives find required packages and load their configuration.
# The 'set' directive defines a variable (e.g. 'GZ_PLUGIN_VER').
# Such variables can be used lateron in the CMakeLists.txt file.
find_package(ament_cmake REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# 添加GUI插件的依赖项
find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER ${gz-gui8_VERSION_MAJOR})

# 注释掉不存在的库目标
# add_library(BasicSystem
# SHARED
# src/BasicSystem.cc
# )

# target_include_directories(
# BasicSystem PRIVATE include
# )

# target_link_libraries(BasicSystem PRIVATE
# gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# 注释掉不存在的库目标
# add_library(FullSystem
# SHARED
# src/FullSystem.cc
# )

# target_include_directories(
# FullSystem PRIVATE include
# )

# target_link_libraries(FullSystem PRIVATE
# gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# Add plugin subdirectories
add_subdirectory(plugins/rope_visualizer)
add_subdirectory(plugins/gui/hypa-dt-gui-plugin)

# 注释掉不存在的库目标的安装
# install(
# TARGETS BasicSystem FullSystem
# DESTINATION lib/${PROJECT_NAME}
# )

# 安装世界、启动和配置资源到对应的 share 子目录。
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# TODO: Install plugin configuration files when plugins are properly structured
# install(
# DIRECTORY plugins/gui/hypa-gui-plugin/
# DESTINATION share/${PROJECT_NAME}/plugins/gui/hypa-gui-plugin
# FILES_MATCHING PATTERN "*.config" PATTERN "*.qml"
# )

# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'sourece install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

# ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in") # 这一项是向下兼容的，保留了sh脚本的安装，虽然现在主要使用dsv脚本

# Following directive configures ament based on the previous directives, and should
# typically be the last directive in the CMakeLists.txt file.
ament_package()
