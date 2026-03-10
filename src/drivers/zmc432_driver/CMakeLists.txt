cmake_minimum_required(VERSION 3.8)
project(zmc432_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # suppress warnings coming from vendor headers (e.g. zmcaux.h) such as
  # "/*" within comment (-Wcomment).  We cannot modify those headers, so
  # add a blanket disable for this warning locally.
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-comment)
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

# 创建静态库
add_library(${PROJECT_NAME} STATIC
  src/motion_controller.cpp
  src/zmotion_wrapper.cpp
  src/zmcaux.cpp
  src/ecat_init.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/zmc432_driver
)

# 非 ROS 第三方库
target_link_libraries(${PROJECT_NAME}
  ${ZMOTION_LIB}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  hypa_msgs
)

# Executable
add_executable(motion_node
  src/motion_node.cpp
  src/motion_topic_node.cpp
)

target_link_libraries(motion_node ${PROJECT_NAME})

install(TARGETS motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ensure new headers installed as part of include directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 ZMotion SDK 库文件
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmotion.so
  DESTINATION lib/${PROJECT_NAME}
)

# 可选：添加测试程序
if(BUILD_TESTING)
  add_executable(test_enable test/test_enable.cpp)
  target_include_directories(test_enable PUBLIC include)
  ament_target_dependencies(test_enable rclcpp hypa_msgs)
  target_link_libraries(test_enable ${PROJECT_NAME} ${ZMOTION_LIB})

  # EtherCAT 初始化测试
  add_executable(test_ecat_init test/test_ecat_init.cpp)
  target_include_directories(test_ecat_init PUBLIC include)
  ament_target_dependencies(test_ecat_init rclcpp)
  target_link_libraries(test_ecat_init ${PROJECT_NAME} ${ZMOTION_LIB})

  # 安装测试程序
  install(TARGETS test_enable test_ecat_init
    DESTINATION lib/${PROJECT_NAME}
  )

  # ensure tests can find zmotion library at runtime
  set_target_properties(test_enable PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE)
  set_target_properties(test_ecat_init PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE)
endif()

# 安装环境hook脚本
install(DIRECTORY hook/
  DESTINATION share/${PROJECT_NAME}/hook
)

# 安装 ZMotion SDK 库文件
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libzmotion.so
  DESTINATION lib/${PROJECT_NAME}
)

# 设置 RPATH 以便运行时自动找到库文件
set_target_properties(motion_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
