# HWT9053 CAN IMU 驱动实施指导

> **项目版本**: HWT9053 CAN Driver v0.0.1  
> **作者**: Xinyu Sheng  
> **日期**: 2026-03-03  
> **ROS 2 版本**: Humble (支持 Jazzy)

---

## 📋 目录

- [环境检查](#环境检查)
- [依赖安装](#依赖安装)
- [包结构说明](#包结构说明)
- [构建与编译](#构建与编译)
- [使用示例](#使用示例)
- [调试与测试](#调试与测试)
- [常见问题](#常见问题)

---

## 环境检查

### 1. 确认 ROS 2 版本

```bash
ros2 --version
```

**预期输出**（Humble）:
```
ROS 2 Humble Hawksbill (development os: Ubuntu 22.04.6 LTS)
```

如果您使用的是不同版本，代码应该仍然兼容 ROS 2 Humble 及更新版本（已在 Humble 上开发和测试）。

### 2. 确认工作区位置

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
ls src/drivers/hwt9053_can_driver
```

应该看到以下目录结构：
```
hwt9053_can_driver/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── hwt9053_can_driver/
│       ├── hwt9053_parser.hpp
│       └── hwt9053_can_driver_node.hpp
├── src/
│   ├── hwt9053_parser.cpp
│   ├── hwt9053_can_driver_node.cpp
│   └── main.cpp
├── launch/
│   ├── hwt9053_driver.launch.py
│   └── imu_system.launch.py
├── config/
│   └── hwt9053_params.yaml
└── test/
```

---

## 依赖安装

### 系统依赖

#### 1. CAN 工具包（必需）

```bash
# 安装 SocketCAN 工具
sudo apt update
sudo apt install -y can-utils
```

验证安装：
```bash
which cansend canrecv candump
```

#### 2. ROS 2 SocketCAN 包（必需）

```bash
# 安装 ROS 2 SocketCAN 接口和消息定义
sudo apt install -y \
  ros-humble-ros2-socketcan \
  ros-humble-ros2-socketcan-msgs
```

验证安装：
```bash
# 应该显示包的文件列表
dpkg -L ros-humble-ros2-socketcan | head -20
```

#### 3. ROS 2 CAN 消息包（必需）

```bash
# 安装 CAN 消息类型定义
sudo apt install -y ros-humble-can-msgs
```

### Python 依赖（可选，用于 launch 文件）

launch 文件使用 Python 3（ROS 2 标准），应该已随 ROS 2 安装。验证：

```bash
python3 --version
ros2 launch --help
```

---

## 包结构说明

### 关键文件描述

#### 1. **include/hwt9053_can_driver/hwt9053_parser.hpp**

HWT9053 CAN 数据解析库的头文件。

**主要类**:
- `HWT9053Parser`: CAN 帧解析和 IMU 消息转换

**关键方法**:
```cpp
// 解析单个 CAN 帧
void ParseCANFrame(uint32_t _can_id, const std::array<uint8_t, 8> &_data, uint8_t _dlc);

// 转换为 ROS IMU 消息
sensor_msgs::msg::Imu ToIMUMessage() const;

// 获取传感器原始数据
const HWT9053Data &GetData() const;
```

**CAN ID 映射**:
| CAN ID | 数据类型  | 内容                  |
| ------ | --------- | --------------------- |
| 0x50   | 加速度    | ax, ay, az (±16g)     |
| 0x51   | 角速度    | gx, gy, gz (±2000°/s) |
| 0x52   | 欧拉角    | roll, pitch, yaw      |
| 0x53   | 磁场      | mx, my, mz            |
| 0x54   | 温度/状态 | temperature           |

#### 2. **include/hwt9053_can_driver/hwt9053_can_driver_node.hpp**

ROS 2 驱动节点的头文件。

**主要类**:
- `HWT9053CANDriverNode`: 生命周期节点，管理 CAN 数据订阅和 IMU 发布

**生命周期**:
- `on_configure()`: 配置参数、创建发布器和订阅器
- `on_activate()`: 启动数据发送
- `on_deactivate()`: 停止数据发送
- `on_cleanup()`: 清理资源

#### 3. **src/hwt9053_parser.cpp**

解析器的实现，包含：
- CAN 帧字节到整数的转换
- 原始数据到物理量的缩放
- 欧拉角到四元数的转换

#### 4. **src/hwt9053_can_driver_node.cpp**

驱动节点的实现，包含：
- 参数初始化和声明
- CAN 帧回调处理
- IMU 消息发布

#### 5. **launch/hwt9053_driver.launch.py**

单个 HWT9053 驱动的启动文件。

**启动参数**:
```bash
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  can_interface:=can0 \
  imu_frame_id:=imu_link \
  use_sim_time:=false \
  log_debug:=false
```

#### 6. **launch/imu_system.launch.py**

综合 IMU 系统启动文件，支持仿真和真机切换。

**启动参数**:
```bash
# 真机模式
ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=false

# 仿真模式
ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=true
```

---

## 构建与编译

### 1. 构建单个包

```bash
cd /home/xinyu/Projects/HKU/hypa_ws

# 仅构建 hwt9053_can_driver（推荐用于开发）
colcon build --packages-select hwt9053_can_driver \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

**预期输出**:
```
Starting >>> hwt9053_can_driver
Finished <<< hwt9053_can_driver [X.XXs]

Summary: 1 package finished [X.XXs]
```

### 2. 构建整个 hypa 工作区（包含驱动）

```bash
cd /home/xinyu/Projects/HKU/hypa_ws

# 构建所有包，包括新的 hwt9053_can_driver
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### 3. 构建后环境设置

```bash
# 导入工作区环境
source /home/xinyu/Projects/HKU/hypa_ws/install/setup.bash

# 验证环境变量
echo $AMENT_PREFIX_PATH | grep hwt9053_can_driver
```

### 4. 完全清理和重建（如果出现编译错误）

```bash
cd /home/xinyu/Projects/HKU/hypa_ws

# 删除构建和安装文件
rm -rf build install log

# 重新构建
colcon build --packages-select hwt9053_can_driver \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## 使用示例

### 前置条件

#### 1. 配置物理 CAN 接口（真机模式）

在开始之前，需要配置系统 CAN 接口（如果使用真实 CAN 硬件）：

```bash
# 查看可用的 CAN 接口
ip link show | grep can

# 配置 CAN0 为 500kbps（HWT9053 标准波特率）
sudo ip link set can0 up type can bitrate 500000

# 验证配置
ip -d link show can0
```

**预期配置**:
```
3: can0: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN mode DEFAULT group default qlen 10
    link/can promiscuity 0 
    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0
    ...
    bitrate 500000 sample-point 0.875
```

#### 2. 启动 ros2_socketcan 节点（如果使用 CAN 硬件）

ros2_socketcan 提供了标准的 CAN 接口，将物理 CAN 消息转换为 ROS 话题。

```bash
# 启动 CAN 总线节点
ros2 run ros2_socketcan socket_can_receiver --ros-args \
  -p interface:=can0 \
  -p can_id_include:=50,51,52,53,54
```

或使用 launch 文件启动整个系统。

### 使用场景 1: 单独运行 HWT9053 驱动

```bash
# 终端 1: 启动 ROS 2 守护程序
source /home/xinyu/Projects/HKU/hypa_ws/install/setup.bash
ros2 daemon start

# 终端 2: 启动驱动节点
source /home/xinyu/Projects/HKU/hypa_ws/install/setup.bash
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  can_interface:=can0 \
  log_debug:=true
```

### 使用场景 2: 多机器人部署

```bash
# 启动机器人 1
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot1

# 启动机器人 2（在另一个终端）
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot2
```

话题自动隔离：
```bash
# 机器人 1 的 IMU：/robot1/imu/data
# 机器人 2 的 IMU：/robot2/imu/data
```

### 使用场景 3: 仿真与真机切换

```bash
# 仿真模式（需要 Gazebo）
ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=true

# 真机模式
ros2 launch hwt9053_can_driver imu_system.launch.py use_sim:=false
```

---

## 调试与测试

### 1. 验证节点运行

```bash
# 查看所有节点
ros2 node list

# 应该显示：
# /hwt9053_can_driver
```

### 2. 检查发布的话题

```bash
# 列出所有话题
ros2 topic list --include-hidden-topics

# 查看 IMU 话题的详细信息
ros2 topic info /robot/imu/data
```

### 3. 实时监控 IMU 数据

```bash
# 订阅并显示 IMU 消息
ros2 topic echo /robot/imu/data

# 应该看到类似输出：
# header:
#   stamp:
#     sec: 1234567890
#     nanosec: 123456789
#   frame_id: imu_link
# orientation:
#   x: 0.0
#   y: 0.0
#   z: 0.0
#   w: 1.0
# angular_velocity:
#   x: 0.001
#   y: -0.002
#   z: 0.003
# linear_acceleration:
#   x: 0.001
#   y: 0.002
#   z: 9.81
# ...
```

### 4. 监控 CAN 总线流量

```bash
# 查看原始 CAN 帧
candump can0

# 过滤特定 ID（HWT9053 专用）
candump can0,050~54F

# 应该看到类似输出：
# can0  050   [8]  12 34 56 78 AB CD EF 00
# can0  051   [8]  11 22 33 44 55 66 77 88
# ...
```

### 5. 启用调试日志

```bash
# 运行时启用调试日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  log_debug:=true

# 查看完整日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  log_debug:=true 2>&1 | grep -E "CAN|IMU|接收"
```

### 6. 性能分析

```bash
# 测量话题发布频率
ros2 topic hz /robot/imu/data

# 应该显示：
# average rate: 100.0
# min: 10.0ms max: 11.0ms std dev: 0.50ms window: 10
```

---

## 常见问题

### Q1: 编译时找不到 ros2_socketcan

**错误信息**:
```
CMake Error: Could not find a package configuration file provided by "ros2_socketcan"
```

**解决方案**:

```bash
# 检查包是否安装
dpkg -l | grep ros-humble-ros2-socketcan

# 如果未安装，执行：
sudo apt update
sudo apt install -y ros-humble-ros2-socketcan ros-humble-ros2-socketcan-msgs

# 清理并重新构建
cd /home/xinyu/Projects/HKU/hypa_ws
rm -rf build install
colcon build --packages-select hwt9053_can_driver
```

### Q2: CAN 接口显示"Device or resource busy"

**原因**: CAN 接口已被其他程序占用

**解决方案**:

```bash
# 查找占用 CAN 接口的进程
lsof /dev/can0

# 终止该进程或另外选择一个接口
sudo ip link set can0 down

# 重新配置
sudo ip link set can0 up type can bitrate 500000
```

### Q3: IMU 消息为零或不变

**原因**: CAN 总线上没有数据或未启动 ros2_socketcan

**排查步骤**:

```bash
# 1. 检查 CAN 总线是否有数据
candump can0

# 2. 验证 ros2_socketcan 是否正在运行
ros2 node list | grep socketcan

# 3. 检查 from_can_bus 话题是否有数据
ros2 topic hz /robot/from_can_bus

# 4. 启用驱动调试日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py log_debug:=true
```

### Q4: 多机器人之间的话题冲突

**症状**: 多个机器人发布到同一个话题

**解决方案**: 确保每个机器人有唯一的 `robot_name` 参数

```bash
# 正确做法：
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot1
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot2

# 话题自动隔离到：
# /robot1/imu/data
# /robot2/imu/data
```

### Q5: 编译时出现 "fatal error: can_msgs/msg/frame.hpp"

**原因**: can_msgs 包未安装

**解决方案**:

```bash
sudo apt install -y ros-humble-can-msgs

# 清理并重新构建
cd /home/xinyu/Projects/HKU/hypa_ws
colcon clean packages --this
colcon build --packages-select hwt9053_can_driver
```

### Q6: 如何在 Jazzy 上使用该驱动？

代码已设计为与 ROS 2 Humble 和 Jazzy 兼容。只需在 Jazzy 环境中安装相应的包：

```bash
# 对于 Jazzy
sudo apt install -y \
  ros-jazzy-ros2-socketcan \
  ros-jazzy-ros2-socketcan-msgs \
  ros-jazzy-can-msgs
```

然后按照相同的过程构建和运行。

---

## 集成到 hypa_drivers 元包

### 更新元包依赖

编辑 `src/hypa_drivers/package.xml`：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>hypa_drivers</name>
  <version>0.0.1</version>
  <description>HYPA 驱动程序元包</description>
  <maintainer email="sheng.xin.yu@faxmail.com">Xinyu Sheng</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 驱动依赖 -->
  <exec_depend>zmc432_driver</exec_depend>
  <exec_depend>hwt9053_can_driver</exec_depend>  <!-- 新增 -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

编辑 `src/hypa_drivers/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(hypa_drivers)

find_package(ament_cmake REQUIRED)

# 元包不需要编译代码，只需声明依赖
ament_package(
  DEPENDS
    zmc432_driver
    hwt9053_can_driver  # 新增
)
```

### 构建整个驱动库

```bash
cd /home/xinyu/Projects/HKU/hypa_ws

colcon build --packages-select hwt9053_can_driver hypa_drivers \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## 遇到问题时的调试技巧

### 1. 完整的编译日志

```bash
colcon build --packages-select hwt9053_can_driver \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
```

### 2. ROS 2 CLI 调试

```bash
# 设置日志级别为 DEBUG
export RCL_LOG_LEVEL=DEBUG

ros2 launch hwt9053_can_driver hwt9053_driver.launch.py
```

### 3. 使用 rqt 图形化工具观察节点

```bash
# 安装 rqt（如未安装）
sudo apt install -y ros-humble-rqt ros-humble-rqt-graph

# 启动图形化界面
rqt
# 在 Plugins -> Introspection -> Node Graph 中观看节点关系
```

### 4. 查看包信息

```bash
# 验证包被正确安装
ros2 pkg list | grep hwt9053

# 查看包的详细信息
ros2 pkg xml hwt9053_can_driver

# 查看可执行文件
ls -la install/hwt9053_can_driver/lib/hwt9053_can_driver/
```

---

## 下一步

1. **硬件连接**: 将 HWT9053 传感器的 CAN 线连接到计算机的 CAN 接口
2. **参数调整**: 根据实际硬件配置修改 `config/hwt9053_params.yaml`
3. **集成应用**: 在 `hypa_application` 中订阅 `/robot/imu/data` 话题
4. **容器部署**: 使用 Docker Compose 进行多版本管理和部署

---

## 参考资源

- [ROS 2 官方文档](https://docs.ros.org/en/humble/)
- [ros2_socketcan 文档](https://github.com/autowarefoundation/ros2_socketcan)
- [HWT9053 CAN 协议文档](https://wit-motion.yuque.com/docs/share/e8142d94-4e2d-413b-8096-eeb79144f710)
- [HYPA 项目 CONTRIBUTING.md](/home/xinyu/Projects/HKU/hypa_ws/CONTRIBUTING.md)
- [HYPA 项目 AGENTS.md](/home/xinyu/Projects/HKU/hypa_ws/AGENTS.md)

---

## 许可证

Apache License 2.0

## 维护者

Xinyu Sheng <sheng.xin.yu@faxmail.com>
