# ROS2 Action 服务器实现计划

**项目**: hypa_hardware - ZMC432 运动控制硬件抽象层
**目标**: 提供 ROS2 Action 接口，通过 ZMC432 控制器的 EtherCAT 接口驱动 AKD 伺服驱动器
**日期**: 2026-02-24
**版本**: 1.0

---

## 目录

1. [项目概述](#项目概述)
2. [系统架构](#系统架构)
3. [详细实施步骤](#详细实施步骤)
4. [消息接口定义](#消息接口定义)
5. [代码实现](#代码实现)
6. [构建配置](#构建配置)
7. [验证与测试](#验证与测试)
8. [决策与约定](#决策与约定)

---

## 项目概述

### 背景

- **硬件**: ZMC432-V2 运动控制器（正运动技术）通过 EtherCAT 总线连接 AKD 伺服驱动器（Kollmorgen）
- **现有资源**:
  - ZMotion SDK: `zmotion.h` + `libzmotion.so`（提供底层 API）
  - 参考实现: `zmcaux.cpp`（辅助函数）
  - AKD EtherCAT 通信手册（DS402 协议对象字典）
- **目标**: 在 ROS2 生态中提供统一的运动控制接口，支持单轴独立运动和多轴插补运动

### 功能需求

1. **单轴运动控制**
   - 绝对位置运动
   - 相对位置运动
   - 连续速度运动（JOG）
   - 回零操作

2. **多轴插补运动**
   - 直线插补（多轴同步运动）
   - 圆弧插补（3点定圆）
   - 螺旋插补（3轴）
   - 椭圆插补
   - 空间圆弧+螺旋插补
   - 支持缓冲插补（连续轨迹流式执行）

3. **状态监控**
   - 实时反馈位置（规划位置 DPOS、反馈位置 MPOS）
   - 实时速度
   - 轴状态字（告警、使能、到位等）
   - 运动进度

4. **错误处理**
   - 通信错误检测与恢复
   - 轴故障诊断（基于 AKD 紧急消息代码）
   - 运动取消支持

### 设计原则

- **分层架构**: ROS2 Action → MotionController → ZMotionWrapper → ZMC432
- **线程安全**: 命令缓冲队列独立线程执行，避免阻塞 ROS2 callback
- **可扩展性**: 支持未来添加新的插补模式和运动类型
- **错误隔离**: 单轴故障不影响其他轴，提供清晰错误信息

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 生态系统                          │
├─────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────┐         ┌──────────────────────────┐   │
│  │  Action      │         │  其他节点（hypa_application）│   │
│  │  Client      ├────────▶│  hypa_bringup, etc.      │   │
│  │              │         │                          │   │
│  └──────────────┘         └──────────────────────────┘   │
│           │                                                  │
│           │ MultiAxisMotion.action                          │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────┐         │
│  │        MotionActionServer (hypa_hardware)    │         │
│  │  - Goal 处理与验证                          │         │
│  │  - Feedback 发布（50ms 间隔）               │         │
│  │  - Cancel 处理                             │         │
│  └──────────────────────────────────────────────┘         │
│           │                                                  │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────┐         │
│  │          MotionController                    │         │
│  │  - 命令缓冲队列管理                          │         │
│  │  - 多轴运动协调                             │         │
│  │  - 执行线程（独立）                         │         │
│  │  - 状态聚合                                 │         │
│  └──────────────────────────────────────────────┘         │
│           │                                                  │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────┐         │
│  │          ZMotionWrapper                      │         │
│  │  - ZMC432 连接管理                          │         │
│  │  - ZMotion API 封装                         │         │
│  │  - 错误码转换                               │         │
│  │  - 轴参数配置（units, speed, accel）       │         │
│  └──────────────────────────────────────────────┘         │
│           │                                                  │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────┐         │
│  │          libzmotion.so (ZMotion SDK)         │         │
│  │  - ZMC_OpenEth, ZMC_Close                   │         │
│  │  - ZAux_Direct_Move, ZAux_Direct_MoveAbs   │         │
│  │  - ZAux_Direct_MoveCirc, ZAux_Direct_MoveSpiral│     │
│  │  - ZAux_Direct_GetDpos, ZAux_Direct_GetMpos│         │
│  └──────────────────────────────────────────────┘         │
│           │                                                  │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────┐         │
│  │          ZMC432 控制器（IP: 192.168.0.11）  │         │
│  │  - EtherCAT Master                         │         │
│  │  - 轴映射（AXIS_ADDRESS）                  │         │
│  │  - PDO 映射（0x1701, 0x1702 等）           │         │
│  └──────────────────────────────────────────────┘         │
│           │                                                  │
│           ▼                                                  │
│  ┌──────────────────────────────────────────────┐         │
│  │          AKD 伺服驱动器（EtherCAT 从站）    │         │
│  │  - DS402 状态机（6040h, 6041h）            │         │
│  │  - 目标位置（607Ah）                       │         │
│  │  - 操作模式（6060h, 6061h）                │         │
│  └──────────────────────────────────────────────┘         │
│                                                            │
└─────────────────────────────────────────────────────────────┘
```

---

## 详细实施步骤

### 步骤 1: 定义消息接口

**文件**: `hypa_msgs/action/MultiAxisMotion.action`

定义 Action 的 Goal、Result、Feedback 结构，支持单轴和多轴运动。

### 步骤 2: 集成 ZMotion SDK

1. 复制 `ref/source/zmotion.h` 到 `hypa_hardware/include/hypa_hardware/`
2. 复制 `ref/source/libzmotion.so` 到 `hypa_hardware/lib/`
3. （可选）复制 `ref/source/zmcaux.cpp` 作为辅助函数参考

### 步骤 3: 实现 ZMotion 封装层

**文件**:
- `hypa_hardware/include/hypa_hardware/zmotion_wrapper.hpp`
- `hypa_hardware/src/zmotion_wrapper.cpp`

**类**: `ZMotionWrapper`

**关键方法**:
- `bool connect(const std::string& ip)`
- `void disconnect()`
- `std::optional<std::string> set_units(int axis, double units)`
- `std::optional<std::string> move_absolute(int axis, double position)`
- `std::optional<std::string> move_relative(int axis, double distance)`
- `std::optional<std::string> move_line_absolute(const std::vector<int>& axes, const std::vector<double>& positions)`
- `std::optional<std::string> buffer_move(const std::vector<int>& axes, const std::vector<double>& positions)`
- `std::optional<std::string> start_continuous()`
- `std::optional<std::string> stop_continuous()`
- `double get_position(int axis)`
- `double get_feedback(int axis)`
- `double get_speed(int axis)`
- `uint32_t get_axis_status(int axis)`

### 步骤 4: 实现运动控制器

**文件**:
- `hypa_hardware/include/hypa_hardware/motion_controller.hpp`
- `hypa_hardware/src/motion_controller.cpp`

**类**: `MotionController`

**数据结构**:
```cpp
struct AxisConfig {
  int axis_number;
  double units;          // 脉冲当量（物理单位/脉冲）
  double speed;          // 最大速度
  double acceleration;   // 加速度
  double deceleration;   // 减速度
};

enum MotionType {
  SINGLE_AXIS = 1,
  INTERPOLATED = 2,
  CONTINUOUS_TRAJECTORY = 3
};

enum InterpolationMode {
  LINEAR = 0,
  CIRCULAR = 1,
  SPIRAL = 2,
  ECLIPSE = 3,
  SPHERICAL = 4
};

struct MotionCommand {
  MotionType motion_type;
  InterpolationMode interpolation_mode;
  std::vector<int> axes;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
  std::vector<double> decelerations;
  std::vector<double> circular_params;  // 圆弧圆心或中间点
  double wait_time;                     // 运动后等待（ms）
};
```

**关键方法**:
- `bool initialize(const std::string& controller_ip)`
- `void start()`
- `void stop()`
- `void queue_motion(const MotionCommand& cmd)`
- `MotionStatus CurrentStatus()`
- `void cancel_current_motion()`

**执行线程逻辑**:
```cpp
void execution_loop() {
  while (running_) {
    MotionCommand cmd;
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      buffer_cv_.wait(lock, [this] { return !command_buffer_.empty() || !running_; });
      if (!running_) break;
      cmd = command_buffer_.front();
      command_buffer_.pop_front();
      executing_ = true;
    }

    switch (cmd.motion_type) {
      case SINGLE_AXIS:
        execute_single_axis(cmd);
        break;
      case INTERPOLATED:
        execute_interpolated(cmd);
        break;
      case CONTINUOUS_TRAJECTORY:
        execute_continuous_trajectory(cmd);
        break;
    }

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      executing_ = false;
    }
  }
}
```

### 步骤 5: 实现 ROS2 Action 服务器

**文件**:
- `hypa_hardware/include/hypa_hardware/motion_action_server.hpp`
- `hypa_hardware/src/motion_action_server.cpp`

**类**: `MotionActionServer`（继承 `rclcpp_action::ServerBase<MultiAxisMotion>`）

**回调实现**:

```cpp
rcl_action_goal_response_t handle_goal(
  const rclcpp_action::GoalUUID & goal_uuid,
  std::shared_ptr<const MultiAxisMotion::Goal> goal) {
  // 验证目标
  if (!validate_goal(goal)) {
    return rcl_action_goal_response_t::REJECT;
  }
  return rcl_action_goal_response_t::ACCEPT_AND_EXECUTE;
}

void handle_accepted(const GoalUUID & goal_uuid) {
  // 在独立线程执行
  std::thread([this, goal_uuid]() {
    execute_goal(goal_uuid);
  }).detach();
}

void execute_goal(const GoalUUID & goal_uuid) {
  // 1. 转换 goal 为 MotionCommand
  // 2. 提交给 controller_->queue_motion()
  // 3. 启动反馈定时器（50ms）
  // 4. 循环检查运动状态，发布 feedback
  // 5. 运动完成或取消时设置 result
}

rcl_action_cancel_response_t handle_cancel(
  const GoalUUID & goal_uuid,
  std::shared_ptr<MultiAxisMotion::Goal> goal) {
  controller_->cancel_current_motion();
  return rcl_action_cancel_response_t::ACCEPT;
}
```

### 步骤 6: 创建主节点

**文件**: `hypa_hardware/src/motion_node.cpp`

```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_hardware_node");

  // 声明参数
  node->declare_parameter<std::string>("controller_ip", "192.168.0.11");
  node->declare_parameter<double>("default_units", 1.0);
  node->declare_parameter<double>("default_speed", 10.0);
  node->declare_parameter<double>("default_accel", 100.0);
  node->declare_parameter<double>("default_decel", 100.0);

  // 创建 MotionController
  auto controller = std::make_shared<MotionController>();
  std::string ip = node->get_parameter("controller_ip").as_string();
  if (!controller->initialize(ip)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to connect to ZMC432 at %s", ip.c_str());
    return 1;
  }

  // 配置默认轴参数
  controller->configure_axis(0, 1.0, 10.0, 100.0, 100.0);
  // ... 其他轴

  // 启动控制器
  controller->start();

  // 创建 Action Server
  MotionActionServer action_server(node, "motion/multi_axis_move", controller);

  RCLCPP_INFO(node->get_logger(), "Motion action server ready on /motion/multi_axis_move");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 步骤 7: 更新 CMakeLists.txt

**文件**: `hypa_hardware/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(hypa_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hypa_msgs REQUIRED)

# ZMotion SDK
set(ZMOTION_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
find_library(ZMOTION_LIB zmotion PATHS ${ZMOTION_SDK_DIR} REQUIRED)

# 可执行目标
add_executable(motion_node
  src/motion_node.cpp
  src/motion_action_server.cpp
  src/motion_controller.cpp
  src/zmotion_wrapper.cpp
)

target_include_directories(motion_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hypa_hardware
)

ament_target_dependencies(motion_node
  ${ZMOTION_LIB}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  hypa_msgs
)

install(TARGETS motion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 步骤 8: 创建 Launch 文件

**文件**: `hypa_hardware/launch/motion_server.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_ip',
            default_value='192.168.0.11',
            description='ZMC432 controller IP address'
        ),
        Node(
            package='hypa_hardware',
            executable='motion_node',
            name='motion_hardware_node',
            output='screen',
            parameters=[
                {'controller_ip': LaunchConfiguration('controller_ip')},
            ],
        ),
    ])
```

### 步骤 9: 更新 package.xml

**文件**: `hypa_hardware/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hypa_hardware</name>
  <version>0.0.1</version>
  <description>Hardware abstraction layer for ZMC432 motion controller via ROS2 actions</description>
  <maintainer email="xinyu@hku.hk">xinyu</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>hypa_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 步骤 10: 定义 hypa_msgs Action

**文件**: `hypa_msgs/action/MultiAxisMotion.action`

```yaml
# Goal
int32[] axes                       # 轴号列表（0-31）
float64[] positions                # 目标位置（物理单位）
float64[] velocities               # 运动速度（物理单位/秒）
float64[] accelerations            # 加速度（可选）
float64[] decelerations            # 减速度（可选）
uint8 motion_type                  # 1=单轴独立, 2=插补同步, 3=连续轨迹
uint8 interpolation_mode           # 0=直线, 1=圆弧, 2=螺旋, 3=椭圆, 4=空间圆弧
float64[] circular_params          # 圆弧参数（圆心坐标或中间点）
float64 wait_time                  # 运动后等待时间（ms，可选）

---
# Result
bool success                       # 运动是否成功完成
float64[] final_positions          # 最终位置
int32 error_code                   # 错误码（0=无错误）
string error_message               # 错误描述

---
# Feedback
float64[] current_positions        # 当前规划位置（DPOS）
float64[] feedback_positions       # 反馈位置（MPOS）
float64[] current_velocities       # 当前速度
uint32[] axis_statuses             # 各轴状态字
float64 progress                   # 运动进度（0-100%）
int32[] executing_axis             # 正在执行的轴列表
```

---

## 消息接口定义

### MultiAxisMotion.action 详细说明

#### Goal 字段

| 字段名               | 类型        | 必需 | 说明                                                                                                                                                                                                                          |
| -------------------- | ----------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `axes`               | `int32[]`   | 是   | 参与运动的轴号列表，例如 `[0]` 或 `[0, 1, 2]`                                                                                                                                                                                 |
| `positions`          | `float64[]` | 是   | 各轴的目标位置（物理单位），长度必须与 `axes` 相同                                                                                                                                                                            |
| `velocities`         | `float64[]` | 是   | 各轴的运动速度（物理单位/秒），长度必须与 `axes` 相同                                                                                                                                                                         |
| `accelerations`      | `float64[]` | 否   | 各轴加速度（物理单位/秒²），缺省使用轴配置参数                                                                                                                                                                                |
| `decelerations`      | `float64[]` | 否   | 各轴减速度（物理单位/秒²），缺省使用轴配置参数                                                                                                                                                                                |
| `motion_type`        | `uint8`     | 是   | 运动类型：<br>**1** = SINGLE_AXIS（各轴独立运动，不同步）<br>**2** = INTERPOLATED（多轴插补，同步起停）<br>**3** = CONTINUOUS_TRAJECTORY（连续轨迹，流式缓冲）                                                                |
| `interpolation_mode` | `uint8`     | 条件 | 插补模式（仅 `motion_type=2` 或 `3` 时有效）：<br>**0** = LINEAR（直线插补）<br>**1** = CIRCULAR（圆弧插补，3点定圆）<br>**2** = SPIRAL（螺旋插补，3轴）<br>**3** = ECLIPSE（椭圆插补）<br>**4** = SPHERICAL（空间圆弧+螺旋） |
| `circular_params`    | `float64[]` | 条件 | 圆弧插补的额外参数（圆心坐标或中间点），长度取决于插补模式                                                                                                                                                                    |
| `wait_time`          | `float64`   | 否   | 运动完成后等待时间（毫秒），缺省 0                                                                                                                                                                                            |

#### Result 字段

| 字段名            | 类型        | 说明                                                                                                                                                                   |
| ----------------- | ----------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `success`         | `bool`      | 运动是否成功完成（true）或被取消/失败（false）                                                                                                                         |
| `final_positions` | `float64[]` | 各轴最终位置（物理单位）                                                                                                                                               |
| `error_code`      | `int32`     | 错误码：<br>**0** = 无错误<br>**1** = 通信错误<br>**2** = 轴配置错误<br>**3** = 参数错误<br>**4** = 运动执行错误<br>**5** = 轴故障（AKD 紧急消息）<br>**6** = 用户取消 |
| `error_message`   | `string`    | 人类可读的错误描述                                                                                                                                                     |

#### Feedback 字段

| 字段名               | 类型        | 说明                                                 |
| -------------------- | ----------- | ---------------------------------------------------- |
| `current_positions`  | `float64[]` | 各轴的规划位置（DPOS，单位：物理单位）               |
| `feedback_positions` | `float64[]` | 各轴的反馈位置（MPOS，单位：物理单位）               |
| `current_velocities` | `float64[]` | 各轴的当前速度（物理单位/秒）                        |
| `axis_statuses`      | `uint32[]`  | 各轴的状态字（ZMotion 状态字，位域定义见 zmotion.h） |
| `progress`           | `float64`   | 整体运动进度（0-100%），基于各轴剩余距离计算         |
| `executing_axis`     | `int32[]`   | 当前正在执行的轴列表                                 |

---

## 代码实现

### 文件结构

```
hypa_hardware/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE
├── docs/
│   └── ROS2_Action_Implementation_Plan.md  (本文件)
├── include/
│   └── hypa_hardware/
│       ├── zmotion.h                        (ZMotion SDK 头文件)
│       ├── zmotion_wrapper.hpp
│       ├── motion_controller.hpp
│       └── motion_action_server.hpp
├── lib/
│   └── libzmotion.so                        (ZMotion SDK 动态库)
├── launch/
│   └── motion_server.launch.py
└── src/
    ├── motion_node.cpp
    ├── motion_action_server.cpp
    ├── motion_controller.cpp
    └── zmotion_wrapper.cpp
```

### 关键类说明

#### ZMotionWrapper

**职责**: 封装 ZMotion SDK 的 C API，提供 C++ 面向对象接口，处理连接生命周期和错误转换。

**设计要点**:
- 使用 RAII 管理 `ZMC_HANDLE`
- 所有方法返回 `std::optional<std::string>`，空值表示成功，有值表示错误信息
- 线程安全：假设单线程调用（由 MotionController 保证）

#### MotionController

**职责**: 管理多轴运动协调、命令缓冲队列、执行线程。

**设计要点**:
- 使用 `std::deque<MotionCommand>` 作为 FIFO 缓冲队列
- 独立执行线程 `execution_thread_` 从队列取命令并执行
- `executing_` 标志表示当前是否有运动在执行
- `cancel_requested_` 用于请求取消当前运动
- 支持 `motion_type=3` 的连续轨迹：使用 `buffer_move` + `start_continuous` 流式发送点

**执行逻辑**:
- **SINGLE_AXIS**: 循环调用 `zmotion_->move_absolute/relative` 各轴独立运动（不等待同步）
- **INTERPOLATED**: 调用 `zmotion_->move_line_absolute` 等一次完成多轴插补
- **CONTINUOUS_TRAJECTORY**:
  1. 调用 `zmotion_->start_continuous()` 启用连续模式
  2. 循环调用 `zmotion_->buffer_move()` 缓冲每个插补点
  3. 当缓冲队列空或收到停止请求时调用 `zmotion_->stop_continuous()`

#### MotionActionServer

**职责**: ROS2 Action 服务器，处理 goal、feedback、cancel。

**设计要点**:
- 使用 `rclcpp_action::Server` 模板
- `handle_goal`: 快速验证，拒绝无效参数（轴号越界、数组长度不匹配等）
- `handle_accepted`: 启动独立线程执行 `execute_goal`，避免阻塞
- `execute_goal`: 提交命令到 controller，循环读取状态发布 feedback，等待完成
- `handle_cancel`: 设置取消标志，controller 检测到后停止运动

---

## 构建配置

### CMakeLists.txt 关键点

1. **链接 ZMotion SDK**:
   ```cmake
   set(ZMOTION_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
   find_library(ZMOTION_LIB zmotion PATHS ${ZMOTION_SDK_DIR} REQUIRED)
   target_link_libraries(motion_node ${ZMOTION_LIB} ...)
   ```

2. **包含头文件路径**:
   ```cmake
   target_include_directories(motion_node PRIVATE
     ${CMAKE_CURRENT_SOURCE_DIR}/include
     ${CMAKE_CURRENT_SOURCE_DIR}/include/hypa_hardware
   )
   ```

3. **安装**:
   - 可执行目标安装到 `lib/${PROJECT_NAME}/`
   - launch 文件安装到 `share/${PROJECT_NAME}/launch/`
   - 头文件安装到 `include/`

### package.xml 关键点

- 包名改为 `hypa_hardware`（与目录名一致）
- 添加 `<depend>rclcpp_action</depend>`
- `hypa_msgs` 使用 `<exec_depend>`（运行时依赖）

---

## 验证与测试

### 编译验证

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hypa_msgs hypa_hardware
source install/setup.bash
```

**预期输出**:
- 无编译错误
- 生成 `install/hypa_hardware/lib/hypa_hardware/motion_node`
- `libzmotion.so` 正确链接

### 运行验证

```bash
# 启动 action 服务器
ros2 launch hypa_hardware motion_server.launch.py
```

**预期输出**:
```
[INFO] [motion_hardware_node]: Connected to ZMC432 at 192.168.0.11
[INFO] [motion_hardware_node]: Axis 0 configured: units=1.0, speed=10.0, accel=100.0
[INFO] [motion_hardware_node]: Action server ready on /motion/multi_axis_move
```

### 功能测试

#### 测试 1: 单轴绝对运动

```bash
ros2 action send_goal /motion/multi_axis_move hypa_msgs/action/MultiAxisMotion \
  '{axes: [0], positions: [100.0], velocities: [10.0], motion_type: 1}'
```

**预期**:
- 轴 0 移动到位置 100.0（物理单位）
- 速度 10.0 单位/秒
- Feedback 显示位置逐渐增加
- Result: `success: true`

#### 测试 2: 双轴直线插补

```bash
ros2 action send_goal /motion/multi_axis_move hypa_msgs/action/MultiAxisMotion \
  '{axes: [0, 1], positions: [100.0, 50.0], velocities: [10.0, 5.0], motion_type: 2, interpolation_mode: 0}'
```

**预期**:
- 轴 0 和轴 1 同步运动，形成直线轨迹
- 两轴同时到达目标位置
- Feedback 中 `progress` 同步增长

#### 测试 3: 三轴螺旋插补

```bash
ros2 action send_goal /motion/multi_axis_move hypa_msgs/action/MultiAxisMotion \
  '{axes: [0, 1, 2], positions: [100.0, 0.0, 50.0], velocities: [10.0, 5.0, 2.0], motion_type: 2, interpolation_mode: 2}'
```

**预期**:
- 三轴按螺旋轨迹运动
- Z轴（轴2）匀速上升，X-Y平面做圆周运动

#### 测试 4: 连续轨迹流式缓冲

```bash
# 终端 1: 启动服务器
ros2 launch hypa_hardware motion_server.launch.py

# 终端 2: 发送连续轨迹（多个点）
ros2 action send_goal /motion/multi_axis_move hypa_msgs/action/MultiAxisMotion \
  '{axes: [0, 1], positions: [0.0, 0.0], velocities: [20.0, 10.0], motion_type: 3, interpolation_mode: 0}'
# 在另一个终端快速发送多个 goal（或编写 client 连续发送）
```

**预期**:
- 第一个点执行后，自动缓冲下一个点
- 轨迹连续无停顿
- 适合点胶、焊接等连续轨迹应用

#### 测试 5: 运动取消

```bash
# 发送一个长距离运动
ros2 action send_goal /motion/multi_axis_move hypa_msgs/action/MultiAxisMotion \
  '{axes: [0], positions: [1000.0], velocities: [50.0], motion_type: 1}' &
# 等待运动开始后，按 Ctrl+C 或发送 cancel
ros2 action cancel_goal /motion/multi_axis_move
```

**预期**:
- 运动立即停止
- Result: `success: false, error_code: 6, error_message: "Cancelled by user"`

### 错误场景测试

1. **无效轴号**: `axes: [99]` → 应拒绝 goal
2. **数组长度不匹配**: `axes: [0, 1], positions: [100.0]` → 应拒绝
3. **通信断开**: 断开网线 → 应返回 `error_code: 1`
4. **AKD 故障**: 模拟驱动器故障（如过流）→ 应返回 `error_code: 5` 并包含 AKD 错误码

---

## 决策与约定

### 已确定的决策

| 决策项           | 选择                                         | 理由                               |
| ---------------- | -------------------------------------------- | ---------------------------------- |
| **包名**         | `hypa_hardware`                              | 与目录名一致，避免混淆             |
| **Action 命名**  | `/motion/multi_axis_move`                    | 统一接口，涵盖单轴和多轴           |
| **单位系统**     | 物理单位（mm/degree）                        | 用户友好，需配置 units 参数转换    |
| **IP 配置**      | Launch 参数（默认 192.168.0.11）             | 灵活，支持不同网络环境             |
| **缓冲策略**     | 边执行边缓冲（streaming）                    | 适合长轨迹和连续轨迹应用           |
| **插补模式**     | 全部支持（直线、圆弧、螺旋、椭圆、空间圆弧） | 满足复杂运动需求                   |
| **线程模型**     | MotionController 独立执行线程                | 避免阻塞 ROS2 callback，提高响应性 |
| **反馈频率**     | 50ms（可配置）                               | 平衡实时性和网络负载               |
| **错误码范围**   | 0-6（见 Result 定义）                        | 简洁清晰，可扩展                   |
| **ZMotion 封装** | 直接链接 libzmotion.so，不修改 SDK           | 保持 SDK 完整性，便于升级          |

### 约定

1. **轴号约定**: 使用 ZMC432 的物理轴号（0-31），通过 EtherCAT 映射到 AKD 驱动器
2. **单位转换**: `物理位置 = 脉冲位置 × units`，`units` 在轴配置时设置
3. **状态字**: 直接使用 ZMotion 的 `AXISSTATUS` 位域，定义见 `zmotion.h`
4. **AKD 错误映射**: 将 AKD 紧急消息代码（如 0x7180）映射为字符串描述，包含在 `error_message` 中
5. **连续轨迹缓冲**: 使用 `buffer_move` + `start_continuous`，执行线程持续发送直到队列空或停止
6. **取消响应**: `cancel` 请求设置标志，执行线程检测到后立即停止并清理缓冲

### 待澄清事项

1. **最大轴数**: 是否限制单次运动的最大轴数？（建议限制为 8 轴，避免 PDO 映射超限）
2. **圆弧插补参数**: 3点定圆模式中，`circular_params` 的具体格式？（建议：`[center_x, center_y, end_angle]` 2D，`[center_x, center_y, center_z, end_angle, end_pitch]` 3D）
3. **螺旋插补参数**: 螺旋的圈数、螺距如何指定？（建议：`[radius, pitch, turns, end_angle]`）
4. **单位默认值**: 如果用户未配置 units，默认 1.0（脉冲单位）是否合适？
5. **回零操作**: 是否通过单独的 Action 还是作为 motion_type 的一种？（建议：单独的 `Home.action`）

---

## 附录

### A. ZMotion API 参考

关键函数（来自 `zmotion.h` 和 `zmcaux.cpp`）:

| 函数                        | 说明          | 用途                       |
| --------------------------- | ------------- | -------------------------- |
| `ZMC_OpenEth`               | 以太网连接    | 建立与 ZMC432 的 TCP 连接  |
| `ZMC_Close`                 | 关闭连接      | 释放资源                   |
| `ZAux_Direct_SetAtype`      | 设置轴类型    | 配置轴为脉冲轴/EtherCAT 轴 |
| `ZAux_Direct_SetUnits`      | 设置脉冲当量  | 单位转换                   |
| `ZAux_Direct_SetSpeed`      | 设置速度      | 规划速度                   |
| `ZAux_Direct_SetAccel`      | 设置加速度    | 规划加速度                 |
| `ZAux_Direct_MoveAbs`       | 绝对运动      | 单轴绝对位置               |
| `ZAux_Direct_Move`          | 相对运动      | 单轴相对位置               |
| `ZAux_Direct_MoveCircAbs`   | 圆弧绝对运动  | 2轴/3轴圆弧插补            |
| `ZAux_Direct_MoveSpiral`    | 螺旋运动      | 3轴螺旋插补                |
| `ZAux_Direct_MoveEclipse`   | 椭圆运动      | 2轴椭圆插补                |
| `ZAux_Direct_MoveSpherical` | 空间圆弧+螺旋 | 3轴空间圆弧                |
| `ZAux_Direct_GetDpos`       | 读取规划位置  | 反馈 DPOS                  |
| `ZAux_Direct_GetMpos`       | 读取反馈位置  | 反馈 MPOS                  |
| `ZAux_Direct_GetMspeed`     | 读取反馈速度  | 反馈速度                   |
| `ZAux_Direct_GetAxisStatus` | 读取轴状态    | 告警、使能、到位等         |

### B. AKD EtherCAT 关键对象

| 索引  | 名称                       | 说明         | 访问  |
| ----- | -------------------------- | ------------ | ----- |
| 6040h | Control Word               | DS402 控制字 | RxPDO |
| 6041h | Status Word                | DS402 状态字 | TxPDO |
| 6060h | Modes of Operation         | 操作模式设置 | RxPDO |
| 6061h | Modes of Operation Display | 操作模式显示 | TxPDO |
| 607Ah | Target Position            | 目标位置     | RxPDO |
| 6063h | Position Actual Value      | 实际位置     | TxPDO |
| 606Ch | Velocity Actual Value      | 实际速度     | TxPDO |

**操作模式**:
- **1**: Profile Position Mode（轮廓位置模式）
- **3**: Profile Velocity Mode（轮廓速度模式）
- **7**: Interpolated Position Mode（插补位置模式，CSP）
- **8**: Cyclic Synchronous Position（循环同步位置，CSP）
- **6**: Homing Mode（回零模式）

### C. 参考文档

- `AKD_EtherCAT_Communications_Manual_EN_REV_U.txt` - AKD EtherCAT 通信手册
- `ZMC432_V2_User_Manual_v1.6.0.txt` - ZMC432 用户手册
- `ZMotion_PC_Library_Programming_Guide_v2.1.2.txt` - ZMotion PC 函数库编程手册

---

## 版本历史

| 版本 | 日期       | 修改说明               | 作者  |
| ---- | ---------- | ---------------------- | ----- |
| 1.0  | 2026-02-24 | 初始版本，完整实现计划 | xinyu |

---

### 关于 `ament_target_dependencies` 的重要说明

#### 问题背景

在 ROS 2 中，使用 `rosidl_generate_interfaces()` 生成的包（如 `hypa_msgs`）**不会**创建传统的 CMake 目标（如 `hypa_msgs::hypa_msgs`）。因此，不能使用 `target_link_libraries(motion_node hypa_msgs)` 来链接这类包。

#### `ament_target_dependencies` vs `target_link_libraries`

**`ament_target_dependencies()` 是 ROS 2 推荐的依赖管理方式，它自动处理：**
- ✅ 包含目录（include directories）
- ✅ 链接库（libraries）
- ✅ 编译定义（compile definitions）
- ✅ 对于 rosidl 包，正确设置 action/message/service 的生成头文件路径

**`target_link_libraries()` 仍然需要用于：**
- ✅ 项目自己的头文件路径（通过 `target_include_directories`）
- ✅ 非 ROS 的第三方库（如 ZMotion SDK 的 `libzmotion.so`）

#### 正确的 CMakeLists.txt 组织方式

```cmake
# 1. 项目自己的头文件路径（必须保留）
target_include_directories(motion_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/hypa_hardware
)

# 2. 非 ROS 第三方库（如 ZMotion SDK）
target_link_libraries(motion_node
  ${ZMOTION_LIB}
)

# 3. ROS 依赖（使用 ament_target_dependencies）
ament_target_dependencies(motion_node
  rclcpp
  rclcpp_action
  hypa_msgs
  std_msgs
  geometry_msgs
)
```

#### 常见错误

❌ **错误 1**: 尝试链接不存在的目标
```cmake
target_link_libraries(motion_node
  hypa_msgs::hypa_msgs  # 这个目标不存在！
)
```

❌ **错误 2**: 将 ROS 依赖也放入 `target_link_libraries`
```cmake
target_link_libraries(motion_node
  ${ZMOTION_LIB}
  rclcpp          # 应该用 ament_target_dependencies
  rclcpp_action   # 应该用 ament_target_dependencies
  hypa_msgs        # 应该用 ament_target_dependencies
)
```

✅ **正确**: 分离 ROS 和非 ROS 依赖

#### 参考文档

- [ROS 2 Tutorial: Writing an Action Server-Client (C++)](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [ROS 2 Tutorial: Creating an Action](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
- [ament_cmake 文档](https://ament.github.io/ament_cmake/)

---

## 决策与约定
````
<userPrompt>
Provide the fully rewritten file, incorporating the suggested code change. You must produce the complete file.
</userPrompt>
