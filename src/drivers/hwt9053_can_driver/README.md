# HWT9053 CAN IMU 驱动

**版本**: v0.0.2 ✅ 生产就绪  
**最后更新**: 2026-03-16

这是一个完整的 ROS 2 驱动，用于 HWT9053 CAN IMU 传感器，支持多机器人、Gazebo 仿真和真机硬件。

## 🚀 快速开始

**⏱️ 5分钟上手？查看 [QUICKSTART.md](./docs/QUICKSTART.md)**

### 1. 安装依赖

```bash
# 系统依赖
sudo apt install -y can-utils

# ROS 2 包
sudo apt install -y \
  ros-humble-ros2-socketcan \
  ros-humble-ros2-socketcan-msgs \
  ros-humble-can-msgs
```

### 2. 构建

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hwt9053_can_driver --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

### 3. 运行

```bash
# 配置 CAN 接口
sudo ip link set can0 up type can bitrate 500000

# 启动驱动
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot can_interface:=can0
```

### 4. 验证

```bash
# 查看 IMU 数据
ros2 topic echo /robot/imu/data

# 运行硬件测试脚本
python3 src/drivers/hwt9053_can_driver/scripts/hardware_test.py --robot robot1 --duration 30
```

## 📚 详细文档

| 文档                                                      | 内容                          | 目标用户 |
| --------------------------------------------------------- | ----------------------------- | -------- |
| [QUICKSTART.md](./docs/QUICKSTART.md)                     | ⚡ 5分钟快速开始、基本验证     | 新用户   |
| [API_REFERENCE.md](./docs/API_REFERENCE.md)               | 📖 C++ API详细说明、代码示例   | 开发者   |
| [HARDWARE_INTEGRATION.md](./docs/HARDWARE_INTEGRATION.md) | 🔧 硬件部署、CAN配置、多机器人 | 系统集成 |
| [TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)           | 🚨 故障排查、诊断指南          | 技术支持 |
| [RELEASE_NOTES.md](./docs/RELEASE_NOTES.md)               | 📝 v0.0.2版本改进汇总          | 维护者   |

**新用户请从 [QUICKSTART.md](./docs/QUICKSTART.md) 开始！**

完整的实施指导请参考 [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)（中文）

## 📦 包含内容

- **Parser**: HWT9053 CAN 帧解析库（PIMPL 模式）
- **Driver Node**: ROS 2 生命周期节点，支持参数化和多机器人
- **Launch Files**: 灵活的启动配置，支持仿真/真机切换
- **Config Files**: YAML 参数配置文件

## 🎯 主要特性

✅ **生产级代码质量** (v0.0.2)
- ✅ 所有P0/P1级审计问题已修复
- ✅ 线程安全（mutex保护临界区）
- ✅ 完整的硬件时间戳支持
- ✅ 磁场数据完整利用

✅ **遵循 HYPA 代码规范**
- 使用 `this->` 访问成员
- 函数参数下划线前缀
- 严格的命名规范

✅ **多机器人支持**
- 自动命名空间隔离
- 参数化 `robot_name`
- 共享CAN总线的多传感器协调

✅ **灵活配置**
- 支持自定义 CAN 接口
- 调整 IMU frame id
- 调试日志选项
- 仿真时钟支持 (use_sim_time)

✅ **生产就绪**
- 生命周期节点管理
- 异常处理和错误日志
- 完整的数据校验
- 自动化硬件测试脚本

## 🔗 话题接口

### 订阅

- `{robot_name}/from_can_bus` (`can_msgs::msg::Frame`) - CAN 总线数据

### 发布

- `{robot_name}/imu/data` (`sensor_msgs::msg::Imu`) - 标准 IMU 消息

## 🛠️ 开发

### 代码结构

```
hwt9053_can_driver/
├── include/hwt9053_can_driver/
│   ├── hwt9053_parser.hpp           # 数据解析库
│   └── hwt9053_can_driver_node.hpp  # ROS 节点
├── src/
│   ├── hwt9053_parser.cpp           # 解析实现
│   ├── hwt9053_can_driver_node.cpp  # 节点实现
│   └── main.cpp                     # 主程序入口
├── launch/
│   ├── hwt9053_driver.launch.py     # 单驱动启动
│   └── imu_system.launch.py         # 系统集成启动
└── config/
    └── hwt9053_params.yaml          # 默认参数
```

### 编译调试

```bash
# 启用详细编译信息
colcon build --packages-select hwt9053_can_driver \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# 运行调试日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  log_debug:=true
```

## 📋 API 文档

### HWT9053Parser

```cpp
// 解析 CAN 帧
parser.ParseCANFrame(0x50, data_array, 8);

// 转换为 IMU 消息
sensor_msgs::msg::Imu imu_msg = parser.ToIMUMessage();

// 获取原始数据
const HWT9053Data& data = parser.GetData();
```

### HWT9053CANDriverNode

启动参数：
| 参数             | 默认值         | 说明                       |
| ---------------- | -------------- | -------------------------- |
| `robot_name`     | `robot`        | 机器人标识（用于命名空间） |
| `can_interface`  | `can0`         | CAN 接口名称               |
| `imu_frame_id`   | `imu_link`     | TF frame id                |
| `imu_topic_name` | `imu/data`     | IMU 话题名                 |
| `can_bus_topic`  | `from_can_bus` | CAN 总线话题               |
| `use_sim_time`   | `false`        | 使用仿真时钟               |
| `log_debug`      | `false`        | 启用调试日志               |

## 🧪 测试

```bash
# 监控 IMU 发布频率
ros2 topic hz /robot/imu/data

# 查看完整的 IMU 消息
ros2 topic echo /robot/imu/data --no-arr

# 检查 CAN 帧
candump can0 -c -n 100
```

## ⚙️ CAN 配置

### HWT9053 CAN ID 映射

| ID  | 数据类型 | 范围 | 分辨率 |
| --- | -------- |+-----+--------|
| 0x50 | 加速度 | ±16g | ~0.005 m/s² |
| 0x51 | 角速度 | ±2000°/s | ~0.001 rad/s |
| 0x52 | 欧拉角 | ±180° | ~0.0005 rad |
| 0x53 | 磁场 | ±4900 µT | ~0.3 µT |
| 0x54 | 温度 | -40~125°C | 0.01°C |

## 🐛 常见问题

见 [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md) 的"常见问题"章节

## 📞 支持

项目主页: `/home/xinyu/Projects/HKU/hypa_ws`

维护者: Xinyu Sheng <sheng.xin.yu@faxmail.com>

**获取帮助**: 
- 遇到问题？查看 [TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)
- 运行硬件测试: `python3 scripts/hardware_test.py --robot robot1`

## 📄 许可证

Apache License 2.0

---

**版本**: v0.0.2  
**状态**: ✅ 生产就绪  
**最后更新**: 2026-03-16
