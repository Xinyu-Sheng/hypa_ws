# HWT9053 CAN 驱动 - 快速参考指南

![状态: 完成](https://img.shields.io/badge/Status-Complete-brightgreen)
![ROS版本: Humble](https://img.shields.io/badge/ROS-Humble-blue)

## 🎯 5分钟快速开始

### 前置条件检查

```bash
# 确认 ROS 2 版本
ros2 --version
# 应该显示: ROS 2 Humble Hawksbill ...

# 确认工作区
cd /home/xinyu/Projects/HKU/hypa_ws
ls src/drivers/hwt9053_can_driver
```

### 环境准备（运行一次）

```bash
# 1️⃣ 安装系统依赖
sudo apt update && sudo apt install -y can-utils

# 2️⃣ 安装 ROS 2 包
sudo apt install -y \
  ros-humble-ros2-socketcan \
  ros-humble-ros2-socketcan-msgs \
  ros-humble-can-msgs

# 3️⃣ 配置 CAN 硬件（如果使用真实 CAN 接口）
sudo ip link set can0 up type can bitrate 500000
```

### 构建与运行

```bash
# 4️⃣ 构建驱动
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hwt9053_can_driver \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 5️⃣ 导入环境
source install/setup.bash

# 6️⃣ 启动驱动
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py

# 7️⃣ 验证数据（新终端）
ros2 topic echo /robot/imu/data
```

---

## 📋 常用命令速查

### 构建命令

```bash
# 构建单个包（推荐）
colcon build --packages-select hwt9053_can_driver

# 构建并导出编译数据库（用于 IDE）
colcon build --packages-select hwt9053_can_driver \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 构建所有包
colcon build

# 完整重建（清除缓存）
rm -rf build install log && colcon build --packages-select hwt9053_can_driver
```

### 运行命令

```bash
# 基础运行
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py

# 多机器人
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot1
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot2

# 启用调试日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py log_debug:=true

# 自定义参数
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=my_robot \
  can_interface:=can1 \
  imu_frame_id:=imu_sensor
```

### 调试命令

```bash
# 查看节点
ros2 node list

# 查看话题
ros2 topic list

# 监控 IMU 数据
ros2 topic echo /robot/imu/data

# 发布频率
ros2 topic hz /robot/imu/data

# 查看原始 CAN 帧
candump can0

# 查看 CAN 接口状态
ip -d link show can0

# 节点信息
ros2 node info /hwt9053_can_driver
```

---

## 🔧 硬件配置

### CAN 接口速率

```bash
# 标准 500 kbps（推荐）
sudo ip link set can0 up type can bitrate 500000

# 其他常见速率
sudo ip link set can0 up type can bitrate 250000  # 低速
sudo ip link set can0 up type can bitrate 1000000 # 高速
```

### 关闭 CAN 接口

```bash
sudo ip link set can0 down
```

---

## 📊 节点信息

| 项目     | 值                                |
| -------- | --------------------------------- |
| 节点名   | `hwt9053_can_driver`              |
| 命名空间 | `/{robot_name}/`                  |
| 节点类型 | `rclcpp_lifecycle::LifecycleNode` |
| 发布话题 | `imu/data` (`sensor_msgs/Imu`)    |
| 订阅话题 | `from_can_bus` (`can_msgs/Frame`) |
| 默认频率 | ~100 Hz（取决于 CAN 总线）        |

---

## 🔗 话题接口

### 发布：IMU 数据

```
话题: /{robot_name}/imu/data
类型: sensor_msgs/Imu
频率: ~100 Hz
字段:
  - header.frame_id: IMU 传感器 TF 坐标系
  - linear_acceleration: 线性加速度 (m/s²)
  - angular_velocity: 角速度 (rad/s)
  - orientation: 方向四元数 (x, y, z, w)
  - [x_covariance]: 协方差矩阵
```

### 订阅：CAN 帧

```
话题: /{robot_name}/from_can_bus
类型: can_msgs/Frame
字段:
  - id: CAN 标识符 (0x50-0x54 for HWT9053)
  - dlc: 数据长度 (8 bytes for HWT9053)
  - data: 8 字节原始数据
```

---

## 🎨 参数列表

| 参数             | 默认值         | 说明        | 建议值                   |
| ---------------- | -------------- | ----------- | ------------------------ |
| `robot_name`     | `robot`        | 机器人标识  | `robot1`, `robot2` ...   |
| `can_interface`  | `can0`         | CAN 接口    | `can0`, `can1` ...       |
| `imu_frame_id`   | `imu_link`     | TF frame ID | `imu_link`, `imu_sensor` |
| `use_sim_time`   | `false`        | 仿真时钟    | `true` (Gazebo)          |
| `log_debug`      | `false`        | 调试日志    | `true` (开发)            |
| `imu_topic_name` | `imu/data`     | IMU 话题    | 相对路径                 |
| `can_bus_topic`  | `from_can_bus` | CAN 话题    | 相对路径                 |

---

## 📁 文件结构

```
hwt9053_can_driver/
├── 📄 README.md                        # 本文件
├── 📄 IMPLEMENTATION_GUIDE.md          # 详细实施指南
├── 📄 QUICK_REFERENCE.md              # 快速参考（本文件）
├── CMakeLists.txt
├── package.xml
├── include/hwt9053_can_driver/
│   ├── hwt9053_parser.hpp             # 解析库头文件
│   └── hwt9053_can_driver_node.hpp    # 节点头文件
├── src/
│   ├── hwt9053_parser.cpp             # 解析库实现
│   ├── hwt9053_can_driver_node.cpp    # 节点实现
│   └── main.cpp                       # 程序入口
├── launch/
│   ├── hwt9053_driver.launch.py       # 单驱动启动
│   └── imu_system.launch.py           # 系统启动
├── config/
│   └── hwt9053_params.yaml            # 参数配置
└── test/
    └── (未来扩展)
```

---

## 🚨 常见错误速解

| 错误                                         | 原因           | 解决                                         |
| -------------------------------------------- | -------------- | -------------------------------------------- |
| `CMake Error: Could not find ros2_socketcan` | 包未安装       | `sudo apt install ros-humble-ros2-socketcan` |
| `Device or resource busy`                    | CAN 接口被占用 | `sudo ip link set can0 down` 后重新配置      |
| `No such device`                             | CAN 接口不存在 | 检查 `ip link show` 确认 `can0` 存在         |
| IMU 值为零                                   | 无 CAN 数据    | 检查 `candump can0` 是否有帧                 |
| 话题找不到                                   | 节点未启动     | 检查 `ros2 node list`                        |

---

## 🧪 验证清单

- [ ] `ros2 --version` 显示 Humble
- [ ] `ros2 pkg list \| grep hwt9053` 有输出
- [ ] `colcon build` 成功完成（无错误）
- [ ] `source install/setup.bash` 无错误
- [ ] `ros2 launch hwt9053_can_driver ...` 启动成功
- [ ] `ros2 topic list` 包含 `imu/data`
- [ ] `ros2 topic echo /robot/imu/data` 显示数据

---

## 💡 提示与技巧

### 提示 1: 自动完成

```bash
# 启用 ROS 2 命令自动完成
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/rosdep/rosdep.bash
```

### 提示 2: 工作区快速切换

```bash
# 在 ~/.bashrc 中添加
alias hypa_ws='cd /home/xinyu/Projects/HKU/hypa_ws && source install/setup.bash'

# 使用
hypa_ws
```

### 提示 3: 节点生命周期管理

```bash
# 检查节点状态
ros2 lifecycle list /hwt9053_can_driver

# 转移状态
ros2 lifecycle set /hwt9053_can_driver transition_configure 1
ros2 lifecycle set /hwt9053_can_driver transition_activate 1
```

### 提示 4: 并行多机器人

```bash
# 在 tmux 中运行（需要 sudo）
```bash
tmux new-session -d -s robot1
tmux send-keys -t robot1 'source /home/xinyu/Projects/HKU/hypa_ws/install/setup.bash && ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot1' Enter

tmux new-window -t robot1
tmux send-keys -t robot1 'source /home/xinyu/Projects/HKU/hypa_ws/install/setup.bash && ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot2' Enter
```
```

---

## 📖 学习资源

- **本地文档**: [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md) - 完整实施指南
- **官方文档**: http://docs.ros.org/en/humble/
- **代码规范**: [CONTRIBUTING.md](../../CONTRIBUTING.md)
- **代理指南**: [AGENTS.md](../../AGENTS.md)

---

## 🔗 相关包

- **hypa_drivers**: 驱动元包（包含本驱动）
- **hypa_msgs**: 自定义消息类型
- **zmc432_driver**: 运动控制驱动（兄弟驱动）

---

## 📞 获取帮助

遇到问题？按顺序尝试：

1. **检查文档**: [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md) 的常见问题章节
2. **查看日志**: `ros2 launch ... log_debug:=true`
3. **验证硬件**: `candump can0 -c -n 20`
4. **重建清理**: `rm -rf build install log && colcon build`

---

**最后更新**: 2026-03-03  
**版本**: 0.0.1  
**维护**: Xinyu Sheng <sheng.xin.yu@faxmail.com>
