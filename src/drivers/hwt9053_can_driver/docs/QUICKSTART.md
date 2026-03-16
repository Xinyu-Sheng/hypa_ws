# HWT9053 CAN IMU 驱动 - 快速开始指南

**版本**: v0.0.2  
**最后更新**: 2026-03-16

> 本指南帮助您快速启动和验证HWT9053 CAN IMU驱动。完整文档见[API_REFERENCE.md](./docs/API_REFERENCE.md)和[HARDWARE_INTEGRATION.md](./docs/HARDWARE_INTEGRATION.md)。

---

## 🎯 5分钟快速开始

### 前置条件

✅ ROS 2 Humble 环境已安装  
✅ CAN硬件和USB-CAN适配器就位  
✅ HWT9053 IMU传感器已接线

### 步骤1: 编译驱动

```bash
# 进入工作区
cd ~/hypa_ws

# 编译驱动
colcon build --packages-select hwt9053_can_driver \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 加载环境
source install/setup.bash
```

**预期输出**:
```
Finished <<< hwt9053_can_driver [7.78s]
```

### 步骤2: 配置CAN接口

```bash
# 将CAN接口UP并设置为500kbps（标准波特率）
sudo ip link set can0 up
sudo ip link set can0 type can bitrate 500000

# 验证配置
ip link show can0

# 预期输出:
# 2: can0: <UP,LOWER_UP> mtu 16
#    CAN  bitrate 500000
```

### 步骤3: 启动驱动

```bash
# 启动IMU驱动节点
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot

# 预期日志:
# [INFO] hwt9053_can_driver: Configured HWT9053 CAN driver
# [INFO] hwt9053_can_driver: Activated IMU publisher
```

### 步骤4: 验证数据

**新终端**:
```bash
# 查看话题列表
ros2 topic list | grep imu

# 预期输出:
# /robot/imu/data
# /robot/imu/data_mag (可选)

# 查看消息频率
ros2 topic hz /robot/imu/data

# 预期输出:
# average rate: 200.00 Hz
```

---

## 📊 数据验证

### 查看原始消息

```bash
ros2 topic echo /robot/imu/data | head -30
```

**典型输出**:
```yaml
header:
  stamp:
    sec: 1710664616
    nanosec: 123456789
  frame_id: imu_link
orientation:
  x: 0.0
  y: 0.0
  z: 0.707
  w: 0.707
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
angular_velocity:
  x: 0.001
  y: -0.002
  z: 0.005
angular_velocity_covariance: [5.8e-08, 0.0, 0.0, 0.0, 5.8e-08, 0.0, 0.0, 0.0, 5.8e-08]
linear_acceleration:
  x: 0.234
  y: -0.445
  z: 9.812
linear_acceleration_covariance: [3.4e-05, 0.0, 0.0, 0.0, 3.4e-05, 0.0, 0.0, 0.0, 3.4e-05]
---
```

### 运行自动化测试

```bash
# 完整硬件测试（30秒）
python3 src/drivers/hwt9053_can_driver/scripts/hardware_test.py \
  --robot robot1 \
  --duration 30

# 预期结果:
# ✅ CAN interface check: PASS
# ✅ ROS2 nodes check: PASS  
# ✅ Topic publication check: PASS
# ✅ IMU data quality check: PASS
# ✅ Magnetic field check: PASS
# ✅ Message frequency check: PASS (200 Hz ±0.5%)
# Summary: 8/8 checks passed
```

---

## 🔧 常用配置

### 配置1: 更改机器人名称

```bash
# 启动时指定机器人名称
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot2

# 话题将变为: /robot2/imu/data
```

### 配置2: 更改CAN接口

```bash
# 如果使用不同的CAN接口（如can1）
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  can_interface:=can1

# 提前配置can1:
# sudo ip link set can1 up
# sudo ip link set can1 type can bitrate 500000
```

### 配置3: 仿真模式

```bash
# 启用ROS 2仿真时钟支持
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  use_sim_time:=true
```

### 配置4: 修改IMU坐标系

```bash
# 更改IMU frame_id（用于tf转换）
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  imu_frame_id:=imu_link_base

# 话题frame_id改为: imu_link_base
```

---

## 📡 多机器人启动

对于多个HWT9053传感器（在同一CAN总线上）：

```bash
# 启动多个机器人的驱动
ros2 launch hwt9053_can_driver imu_system.launch.py \
  robots:="robot1,robot2,robot3"

# 验证
ros2 topic list | grep imu
# 预期:
# /robot1/imu/data
# /robot2/imu/data
# /robot3/imu/data

# 监听robot1数据
ros2 topic echo /robot1/imu/data -n 1
```

> 多个传感器需要不同的CAN ID。详见[HARDWARE_INTEGRATION.md](./docs/HARDWARE_INTEGRATION.md)。

---

## 🚨 故障快速修复

### 问题: 无CAN消息

```bash
# 1. 检查接口状态
ip link show can0

# 2. 监听CAN总线
candump can0 -n 5

# 3. 如果接口DOWN，重新启动:
sudo ip link set can0 down
sudo ip link set can0 up

# 完整诊断见: docs/TROUBLESHOOTING.md
```

### 问题: 话题无消息

```bash
# 1. 检查节点状态
ros2 node list
ros2 node info /robot/hwt9053_can_driver

# 2. 查看日志
tail -30 ~/.ros/log/latest/hwt9053_can_driver/stdouterr.log

# 3. 启用调试模式
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  log_debug:=true
```

### 问题: 消息频率不稳定

```bash
# 检查CPU占用
top -p $(pgrep hwt9053_can_driver)

# 检查CAN错误
ip -s link show can0

# 尝试降低波特率（如果硬件支持）
sudo ip link set can0 type can bitrate 250000
```

完整故障排查指南见 [TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)。

---

## 📚 文档导航

| 任务        | 文档     | 链接                                                                 |
| ----------- | -------- | -------------------------------------------------------------------- |
| 快速开始    | 本文档   | [QUICKSTART.md](./README.md)                                         |
| API集成开发 | API参考  | [API_REFERENCE.md](./docs/API_REFERENCE.md)                          |
| 硬件部署    | 硬件集成 | [HARDWARE_INTEGRATION.md](./docs/HARDWARE_INTEGRATION.md)            |
| 故障诊断    | 故障排查 | [TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)                      |
| 版本信息    | 发行说明 | [RELEASE_NOTES.md](./docs/RELEASE_NOTES.md)                          |
| 审计报告    | 代码质量 | [HWT9053_UNIFIED_AUDIT_REPORT.md](./HWT9053_UNIFIED_AUDIT_REPORT.md) |

---

## 🧪 运行单元测试

```bash
# 编译并运行ROS 2测试
colcon test --packages-select hwt9053_can_driver

# 查看测试结果
colcon test-result --all
```

---

## 🔗 常用命令参考

```bash
# 启动驱动
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot

# 列出所有参数
ros2 node list

# 查看节点参数
ros2 param list /robot/hwt9053_can_driver

# 监听话题频率
ros2 topic hz /robot/imu/data

# 统计消息
ros2 topic echo /robot/imu/data | wc -l

# 记录消息包
ros2 bag record /robot/imu/data

# 重放消息包
ros2 bag play rosbag2_2026_03_16-14_30_45

# 监听CAN总线
candump can0

# 发送测试CAN帧
cansend can0 050#1122334455667788
```

---

## 💡 常见问题

**Q: 如何改变消息发布频率？**  
A: HWT9053硬件固定为200Hz。无法从ROS 2侧修改，需在硬件配置中调整。

**Q: 如何在多台计算机之间共享IMU数据？**  
A: 使用DDS桥接或ROS 2 Bridge。详见[HARDWARE_INTEGRATION.md](./docs/HARDWARE_INTEGRATION.md#多机器人系统部署)。

**Q: 是否支持其他CAN总线接口（如CAN FD）？**  
A: 当前版本支持标准CAN。CAN FD支持计划在未来版本。

**Q: 如何在Gazebo仿真中使用？**  
A: 启用`use_sim_time:=true`参数，并在Gazebo中发布模拟CAN消息。详见集成指南。

---

## 📞 获取帮助

遇到问题时：

1. **查看故障排查指南**: [TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)
2. **运行硬件测试**: `python3 scripts/hardware_test.py --robot robot1`
3. **收集诊断信息**: 见故障排查指南
4. **提交Issue**: 包含诊断信息和错误日志

---

## ✅ 下一步

- ✅ 基础验证完成 → 查看[API_REFERENCE.md](./docs/API_REFERENCE.md)进行自定义集成
- ✅ 多机器人部署 → 查看[HARDWARE_INTEGRATION.md](./docs/HARDWARE_INTEGRATION.md)
- ✅ 遇到问题 → 查看[TROUBLESHOOTING.md](./docs/TROUBLESHOOTING.md)

---

**版本**: v0.0.2  
**维护者**: Xinyu Sheng  
**邮箱**: sheng.xin.yu@faxmail.com

