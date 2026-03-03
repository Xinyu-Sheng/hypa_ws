# hypa_demos

运动控制演示包，展示 `zmc432_driver` 的 ROS2 Action 接口功能。

## 功能演示

本包包含 `motion_demo_client`，演示以下操作：

1. **单轴绝对运动** - 基本运动
2. **双轴直线插补** - 多轴协调
3. **三轴螺旋插补** - 复杂轨迹
4. **连续轨迹流式** - 点胶/焊接应用
5. **取消操作** - 中途停止运动
6. **错误处理** - 无效参数测试

## 依赖

- zmc432_driver (提供 action server)
- hypa_msgs (包含 MultiAxisMotion.action)
- rclcpp, rclcpp_action

## 编译

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hypa_demos
source install/setup.bash
```

## 运行

### 启动演示

确保 `zmc432_driver` 的 action server 已启动：

```bash
# 终端 1: 启动 action server
ros2 launch zmc432_driver motion_server.launch.py
```

然后运行 demo client：

```bash
# 终端 2: 运行演示
ros2 run hypa_demos motion_demo_client
```

### 输出示例

```
[INFO] [motion_demo_client]: Motion Demo Client created
[INFO] [motion_demo_client]: Waiting for action server...
[INFO] [motion_demo_client]: Action server is ready!

=== Demo 1: Single Axis Motion ===
[INFO] [motion_demo_client]: Sending goal for: Single Axis Motion
[INFO] [motion_demo_client]: Goal accepted by server
  Progress: 0.0%  Pos: [0.00]  Vel: [0.00]
  Progress: 25.3%  Pos: [25.30]  Vel: [10.00]
  ...
[INFO] [motion_demo_client]: [Single Axis] Goal succeeded!
  Final positions: [100.00]
```

## 自定义演示

你可以修改 `motion_demo_client.cpp` 中的参数，或创建自己的 client 来测试特定场景。

## 故障排除

- **找不到 action server**: 确保 `zmc432_driver` 已启动
- **连接失败**: 检查 ZMC432 IP 地址和网络连接
- **运动失败**: 检查轴配置和 EtherCAT 连接状态
