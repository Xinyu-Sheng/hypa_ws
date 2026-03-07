# 快速上手

本节提供最小步骤，帮助用户在系统准备好后立刻启动并发送运动命令。适合初学者。

## 依赖

```bash
# ROS2（兼容 Humble/Ironwall/Jazzy）
# ZMotion SDK，确保 libzmotion.so 与 zmotion.h 在包路径中
# hypa_msgs
```

## 构建

```bash
cd /home/xinyu/Projects/HKU/hypa_ws
colcon build --packages-select hypa_msgs zmc432_driver
source install/setup.bash
```

## 运行

```bash
ros2 launch zmc432_driver motion_server.launch.py
```

**常用启动参数**（可在命令行或 launch 文件中传递）：

| 参数                   | 类型    | 默认值           | 说明                  |
| ---------------------- | ------- | ---------------- | --------------------- |
| `controller_ip`        | string  | `192.168.0.11`   | ZMC432 控制器 IP 地址 |
| `default_units`        | float64 | `1.0`            | 默认脉冲当量          |
| `default_speed`        | float64 | `10.0`           | 默认速度              |
| `default_accel`        | float64 | `100.0`          | 默认加速度            |
| `default_decel`        | float64 | `100.0`          | 默认减速度            |
| `motion_command_topic` | string  | `motion_command` | 运动命令话题名        |
| `motion_status_topic`  | string  | `motion_status`  | 运动状态话题名        |

使用示例：

### 单轴绝对运动

```bash
ros2 topic pub /hypa/motion_command hypa_msgs/msg/MotionCommand \
  '{axes: [0], positions: [100.0], velocities: [10.0], motion_type: 1, accelerations: [100.0], decelerations: [100.0]}'
```

> 上述命令含义：在轴 0 上执行绝对位置为 100 的运动，速度 10，使用默认加减速。

### 状态监控

```bash
ros2 topic echo /hypa/motion_status
```

默认启动时轴 0-3 已使能。需要更改使能状态时请参考开发者指南中通过 API 或测试程序控制的方法。


## 故障排除指南

- **找不到 libzmotion.so**：确认文件放在 `zmc432_driver/lib/` 并在 LD_LIBRARY_PATH 中。
- **控制器无法连接**：检查 IP 和网络连通性。
- **运动命令被拒绝**：查看 `motion_status` 中的 `axis_statuses`，确保目标轴已使能。

如需更多调试信息，运行节点时加大日志级别（`ros2 run ... --ros-args -r __log_level:=debug`）。
