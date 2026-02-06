cmake_minimum_required(VERSION 3.8)
project(hypa_launch)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
  )
endif()

# Install documentation
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/docs)
  install(DIRECTORY docs/
    DESTINATION share/${PROJECT_NAME}/docs/
  )
endif()

ament_package()
