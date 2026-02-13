cmake_minimum_required(VERSION 3.5)

# Rope Visualizer Plugin
add_library(RopeVisualizer SHARED
  RopeVisualizer.cc
)

target_include_directories(RopeVisualizer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(RopeVisualizer PUBLIC
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

# Install the plugin library
install(
  TARGETS RopeVisualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin configuration files (if any)
install(
  FILES
  DESTINATION share/${PROJECT_NAME}/rope_visualizer
)
