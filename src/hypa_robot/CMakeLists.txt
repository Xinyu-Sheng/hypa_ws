cmake_minimum_required(VERSION 3.8)
project(hypa_robot)

# 元包只需要ament_cmake
find_package(ament_cmake REQUIRED)

# 元包不需要编译任何代码，只需要安装包信息
ament_package()
