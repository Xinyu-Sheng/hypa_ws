# zmc432_driver

**定位**: HYPA 硬件驱动包 - 提供 ZMC432 运动控制器的 ROS2 接口。

通过 ROS2 Topic 接口控制 ZMC432 控制器，使用 EtherCAT 总线驱动 AKD 伺服驱动器。
支持单轴独立运动和多轴插补运动（直线、圆弧、螺旋、椭圆、空间圆弧）。

## 功能特性

- **单轴运动控制**
  - 绝对位置运动
  - 相对位置运动
  - 连续速度运动（JOG）

- **多轴插补运动**
  - 直线插补（多轴同步）
  - 圆弧插补（3点定圆，2D/3D）
  - 螺旋插补（3轴）
  - 椭圆插补（2轴）
  - 空间圆弧+螺旋插补（3轴）
  - 连续轨迹流式缓冲（支持点胶、焊接等连续轨迹应用）

- **状态监控**
  - 实时反馈位置（规划位置 DPOS、反馈位置 MPOS）
  - 实时速度
  - 轴状态字（告警、使能、到位等）
  - 运动进度反馈

- **错误处理**
  - 通信错误检测
  - 轴故障诊断（基于 AKD 紧急消息）
  - 运动取消支持

## 系统架构

```
ROS2 Topic Publisher → MotionTopicNode → MotionController → ZMotionWrapper → ZMC432 → EtherCAT → AKD Drives
ROS2 Topic Subscriber ← MotionTopicNode ← MotionController ← ZMotionWrapper ← ZMC432 ← EtherCAT ← AKD Drives
```

## 依赖

- ROS2 (Humble/Ironwall/Jazzy)
- ZMotion SDK (libzmotion.so + zmotion.h)
- hypa_msgs (包含 MotionCommand.msg 和 MotionStatus.msg)

## 编译

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hypa_msgs zmc432_driver
source install/setup.bash
```

## 运行

启动 Topic 节点：

```bash
ros2 launch zmc432_driver motion_server.launch.py
```

可选参数：
- `controller_ip`: ZMC432 IP 地址（默认 192.168.0.11）
- `default_units`: 默认脉冲当量（默认 1.0）
- `default_speed`: 默认速度（默认 10.0）
- `default_accel`: 默认加速度（默认 100.0）
- `default_decel`: 默认减速度（默认 100.0）
- `motion_command_topic`: 运动命令主题名（默认 `motion_command`）
- `motion_status_topic`: 运动状态主题名（默认 `motion_status`）

## 使用示例

### 单轴绝对运动

```bash
ros2 topic pub /hypa/motion_command hypa_msgs/msg/MotionCommand \
  '{axes: [0], positions: [100.0], velocities: [10.0], motion_type: 1, accelerations: [100.0], decelerations: [100.0]}'
```

### 双轴直线插补

```bash
ros2 topic pub /hypa/motion_command hypa_msgs/msg/MotionCommand \
  '{axes: [0, 1], positions: [100.0, 50.0], velocities: [10.0, 5.0], motion_type: 2, interpolation_mode: 0, accelerations: [50.0, 50.0], decelerations: [50.0, 50.0]}'
```

### 三轴螺旋插补

```bash
ros2 topic pub /hypa/motion_command hypa_msgs/msg/MotionCommand \
  '{axes: [0, 1, 2], positions: [100.0, 0.0, 50.0], velocities: [10.0, 5.0, 2.0], motion_type: 2, interpolation_mode: 2, circular_params: [10.0, 5.0, 2.0, 360.0], accelerations: [100.0, 50.0, 20.0], decelerations: [100.0, 50.0, 20.0]}'
```

### 监控运动状态

```bash
ros2 topic echo /hypa/motion_status
```

## Topic 接口定义

详见 `hypa_msgs/msg/MotionCommand.msg` 和 `hypa_msgs/msg/MotionStatus.msg`。

### Goal 字段

| 字段名               | 类型        | 必需 | 说明                                                 |
| -------------------- | ----------- | ---- | ---------------------------------------------------- |
| `axes`               | `int32[]`   | 是   | 轴号列表（0-31）                                     |
| `positions`          | `float64[]` | 是   | 目标位置（物理单位）                                 |
| `velocities`         | `float64[]` | 是   | 运动速度（物理单位/秒）                              |
| `accelerations`      | `float64[]` | 否   | 加速度（可选）                                       |
| `decelerations`      | `float64[]` | 否   | 减速度（可选）                                       |
| `motion_type`        | `uint8`     | 是   | 运动类型：1=单轴, 2=插补, 3=连续轨迹                 |
| `interpolation_mode` | `uint8`     | 条件 | 插补模式：0=直线, 1=圆弧, 2=螺旋, 3=椭圆, 4=空间圆弧 |
| `circular_params`    | `float64[]` | 条件 | 圆弧/螺旋/椭圆参数                                   |
| `wait_time`          | `float64`   | 否   | 运动后等待（ms）                                     |

### Result 字段

| 字段名            | 类型        | 说明               |
| ----------------- | ----------- | ------------------ |
| `success`         | `bool`      | 运动是否成功       |
| `final_positions` | `float64[]` | 最终位置           |
| `error_code`      | `int32`     | 错误码（0=无错误） |
| `error_message`   | `string`    | 错误描述           |

### Feedback 字段

| 字段名               | 类型        | 说明               |
| -------------------- | ----------- | ------------------ |
| `current_positions`  | `float64[]` | 规划位置（DPOS）   |
| `feedback_positions` | `float64[]` | 反馈位置（MPOS）   |
| `current_velocities` | `float64[]` | 当前速度           |
| `axis_statuses`      | `uint32[]`  | 轴状态字           |
| `progress`           | `float64`   | 运动进度（0-100%） |
| `executing_axis`     | `int32[]`   | 正在执行的轴       |

## 配置说明

### 轴参数配置

运动节点启动时会自动配置轴 0-3 使用默认参数。要配置更多轴或修改参数，可以：

1. 修改 `motion_node.cpp` 中的配置循环
2. 或通过扩展代码提供动态配置服务（TODO）

### 单位系统

使用物理单位（mm 或 degree），通过 `units` 参数转换为脉冲单位：
```
脉冲位置 = 物理位置 / units
```

例如，如果 `units=0.001`，则 1 个脉冲对应 0.001mm。

### EtherCAT 配置

ZMC432 需要正确配置 EtherCAT 主站，将 AKD 驱动器映射到轴地址。参考：
- `ZMC432_V2_User_Manual_v1.6.0.txt`
- `AKD_EtherCAT_Communications_Manual_EN_REV_U.txt`

### EtherCAT 总线初始化接口

本包现在包含一个可调用的初始化函数，用于在 Linux 上通过 `libzmotion.so` 完成总线配置、扫描、PDO/轴映射、参数下发和启动。

* **C++ API**
  ```cpp
  #include "zmc432_driver/ecat_init.hpp"
  #include "zmc432_driver/motion_controller.hpp"

  zmc432_driver::EcatInitInfo info; // 使用默认参数
  auto err = controller->initialize_bus(info, /*slot*/0, /*timeout_ms*/5000);
  if (err) { /* 处理错误 */ }
  ```

  结构 `EcatInitInfo` 对应底层 C 结构 `EcatInitInfoSet`，默认构造函数会填充典型值。高级用户可以设置字段后调用 `info.toC()`。

* **ROS 2 参数**

  运动节点 `motion_node` 提供可选参数：

  ```yaml
  perform_ecat_init: true
  ecat_slot_id: 0
  ecat_timeout_ms: 5000
  ecat.drive_axis_start: 0
  ecat.drive_axis_num: -1
  # 其他参数目前只支持通过编程修改
  ```

  启动时添加 `perform_ecat_init:=true` 即可自动执行初始化。

* **测试程序**
  包含两个可选测试：
  - `test_enable`：轴使能功能。
  - `test_ecat_init`：连接后执行总线初始化，命令行可传 IP 地址。

  在构建时启用测试 (`colcon build --packages-select zmc432_driver --cmake-args -DBUILD_TESTING=ON`)，可通过 `ros2 run zmc432_driver test_ecat_init 192.168.0.11` 运行。

* **注意**
  - 确保 `libzmotion.so` 是 Linux 版本并在安装目录可被加载。
  - 需要 root 或具有 EtherCAT 访问权限的用户运行。

```

## 故障排除

### 编译错误

- **找不到 libzmotion.so**: 确保 `libzmotion.so` 已复制到 `zmc432_driver/lib/` 目录
- **找不到 zmotion.h**: 确保 `zmotion.h` 已复制到 `zmc432_driver/include/zmc432_driver/`
- **hypa_msgs 未找到**: 先编译 hypa_msgs: `colcon build --packages-select hypa_msgs`

### 运行时错误

- **连接失败**: 检查 ZMC432 IP 地址是否正确，网络是否连通
- **运动失败**: 检查轴是否已正确配置（units、速度等），EtherCAT 连接是否正常
- **轴故障**: 查看 AKD 驱动器状态，参考 AKD 手册中的错误代码

## 开发计划

- [ ] 添加轴配置服务（动态配置 units、速度、加速度等）
- [ ] 添加回零（homing）action
- [ ] 添加位置/速度/力矩模式切换
- [ ] 支持更多插补模式参数验证
- [ ] 添加轨迹点预验证功能
- [ ] 支持多控制器冗余

## 参考文档

- `docs/ROS2_Action_Implementation_Plan.md` - 详细实现计划
- `ref/docs/ZMC432_V2_User_Manual_v1.6.0.txt` - ZMC432 用户手册
- `ref/docs/AKD_EtherCAT_Communications_Manual_EN_REV_U.txt` - AKD EtherCAT 通信手册
- `ref/docs/ZMotion_PC_Library_Programming_Guide_v2.1.2.txt` - ZMotion PC 函数库编程手册

## 许可证

Apache-2.0 (请根据实际情况修改)

## 维护者

xsheng420@connect.hkust-gz.edu.cn
