# 接口说明

本节详细列出 ROS 话题、消息字段和轴使能相关内容，帮助用户正确构造命令和理解状态。

## 话题

- `/[robot_name]/<motion_command_topic>` (`hypa_msgs/msg/MotionCommand`) - 发布运动请求。
- `/[robot_name]/<motion_status_topic>` (`hypa_msgs/msg/MotionStatus`) - 订阅运动状态反馈。

> 在默认 launch 中，`robot_name` 默认为 `hypa`，可通过参数修改。

## MotionCommand 字段

| 字段名               | 类型        | 必需/条件 | 说明                                            |
| -------------------- | ----------- | --------- | ----------------------------------------------- |
| `axes`               | `int32[]`   | 必需      | 轴号列表（0-31）                                |
| `positions`          | `float64[]` | 必需      | 目标位置（物理单位）                            |
| `velocities`         | `float64[]` | 必需      | 运动速度（物理单位/秒）                         |
| `accelerations`      | `float64[]` | 否        | 加速度                                          |
| `decelerations`      | `float64[]` | 否        | 减速度                                          |
| `motion_type`        | `uint8`     | 必需      | 1=单轴, 2=插补, 3=连续轨迹                      |
| `interpolation_mode` | `uint8`     | 条件      | 插补模式：0直线、1圆弧、2螺旋、3椭圆、4空间圆弧 |
| `circular_params`    | `float64[]` | 条件      | 圆弧/螺旋/椭圆参数                              |
| `wait_time`          | `float64`   | 否        | 运动完成后停留时间（ms）                        |

> **轴使能**：请通过 C++ API 或节点提供的服务/参数控制轴的使能状态，参考开发者指南。


## MotionStatus 字段

| 字段名               | 类型        | 说明                                 |
| -------------------- | ----------- | ------------------------------------ |
| `current_positions`  | `float64[]` | 规划位置（DPOS）                     |
| `feedback_positions` | `float64[]` | 反馈位置（MPOS）                     |
| `current_velocities` | `float64[]` | 当前速度                             |
| `axis_statuses`      | `uint32[]`  | 轴状态字（含告警、到位、使能位等）   |
| `axis_enabled`       | `bool[]`    | 实时使能状态（基础功能）             |
| `progress`           | `float64`   | 任务进度（0–100 %）                  |
| `executing_axis`     | `int32[]`   | 当前执行命令涉及的轴（无执行时为空） |

> 轴状态字由驱动器提供，若要解码请参考 AKD 手册。

## 参数说明

在 `launch` 或命令行中也可以修改以下参数：

- `default_units`：物理单位到脉冲的换算系数；
- `default_speed`/`default_accel`/`default_decel`：默认运动参数；
- `perform_ecat_init`：是否在节点启动时执行 EtherCAT 初始化（高级功能）。

其它参数见 `motion_node.cpp` 中的 `declare_parameter()` 调用。
