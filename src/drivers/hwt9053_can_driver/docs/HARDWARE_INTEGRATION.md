# HWT9053 硬件集成指南

**版本**: v0.0.2  
**适用版本**: ROS 2 Humble 及以上  
**最后更新**: 2026-03-16

---

## 目录

1. [硬件清单](#硬件清单)
2. [CAN接口配置](#can接口配置)
3. [电源接线](#电源接线)
4. [CAN总线配置](#can总线配置)
5. [多机器人集成](#多机器人集成)
6. [性能调优](#性能调优)

---

## 硬件清单

### 必需组件

| 组件          | 规格          | 数量 | 备注                 |
| ------------- | ------------- | ---- | -------------------- |
| HWT9053传感器 | 9轴IMU+CAN    | 1+   | 需要带CAN接口版本    |
| CAN收发器     | ISO 11898-2   | 1    | 如ZLG USBCAN等       |
| CAN总线       | 双绞线+屏蔽   | -    | 推荐CAT5e或专用CAN线 |
| 终端电阻      | 120Ω          | 2    | 长线必需（>5m）      |
| 24V电源供应器 | 5A+           | 1    | 根据传感器供电需求   |
| 计算机        | Ubuntu 20.04+ | 1    | 建议四核以上         |

### 可选组件

| 组件          | 用途             |
| ------------- | ---------------- |
| CAN信号分析仪 | 调试CAN总线      |
| 示波器        | 测量CAN波形      |
| 多机器人支架  | 多传感器同时测试 |

---

## CAN接口配置

### 选项1：USB-CAN适配器（推荐用于开发）

**支持的设备**:
- ZLG USBCAN系列
- PEAK PCAN-USB
- Kvaser Hybrid系列

**安装步骤**:
```bash
# 1. 连接USB-CAN适配器到计算机

# 2. 查看设备识别
lsusb | grep -i can

# 3. 验证SocketCAN支持
ip link show type can

# 4. 配置接口
sudo modprobe can_dev
sudo modprobe can_raw
```

**配置脚本** (`scripts/setup_can_interface.sh`):
```bash
#!/bin/bash
# 配置CAN接口

INTERFACE=${1:-can0}
BITRATE=${2:-500000}

echo "配置 $INTERFACE，波特率 $BITRATE bps"
sudo ip link set $INTERFACE down
sudo ip link set $INTERFACE type can bitrate $BITRATE
sudo ip link set $INTERFACE up

# 验证
ip link show $INTERFACE
echo "CAN接口配置完成"
```

**使用**:
```bash
chmod +x scripts/setup_can_interface.sh
./scripts/setup_can_interface.sh can0 500000
```

### 选项2：原生CAN接口（生产环境）

如计算机具有原生CAN接口（如工控机）：

```bash
# 配置原生CAN0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 配置多个接口
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up
```

### 验证CAN配置

```bash
# 1. 检查接口状态
ip link show can0
# 输出应包含: UP, RUNNING

# 2. 监听CAN总线
candump can0
# 应显示实时CAN消息: can0 050 [8] XX XX XX XX XX XX XX XX

# 3. 检查统计信息
cat /sys/class/net/can0/statistics/rx_packets
cat /sys/class/net/can0/statistics/tx_packets
```

---

## 电源接线

### HWT9053传感器供电

**规格检查**:
```bash
# 查询传感器规格（参考datasheet）
# 通常: 3.3V/5V, 功耗 <100mA
```

**接线方案**:

**方案A - 直接电源供給（推荐）**:
```
24V电源 (+)
    ↓
[降压模块 24V→5V, 2A]
    ↓
HWT9053 VCC --- 电源正 (红线)
HWT9053 GND --- 电源负 (黑线)
```

**方案B - 通过工控机电源**:
```
UPS/工控机 5V输出
    ↓
[功率分配板]
    ↓
├─ HWT9053 VCC (5V)
├─ CAN收发器电源
└─ 其他传感器
```

### CAN总线接线

**脚位定义**:
```
标准9针D-Sub接头:
  1: RS-485 B (CAN_H)
  2: RS-485 A (CAN_L)
  3: GND

或4针JST PH:
  1: CAN_H (接收器+)
  2: CAN_L (接收器-)
  3: GND
  4: +5V
```

**多节点接线图**:
```
┌─────────────────────────────────────────┐
│  HWT9053#1       HWT9053#2       HWT9053#3  │
│  CAN_H ───┐       CAN_H ───┐       CAN_H ───┐
│           │               │               │
│  CAN_L ───┼(120Ω)──┼(120Ω)──┼(120Ω)
│           │       │       │
│  GND ─────┴───┴───┴───┘
│
│  [CAN总线]
│
│  USB-CAN adapter (连接计算机)
└─────────────────────────────────────────┘

注: 120Ω终端电阻仅需在总线两端各安装一个
```

**接线检查清单**:
```
☐ CAN_H连接正确（所有节点相同颜色）
☐ CAN_L连接正确（所有节点相同颜色）
☐ GND连接正确，信号地与电源地连通
☐ 终端电阻两个，分别位于总线两端
☐ 所有连接器牢固，无松动
☐ 电源极性正确，无反接
☐ 总线长度 <10m（无屏蔽），或 >10m时使用屏蔽线
```

---

## CAN总线配置

### 波特率设置

HWT9053支持的波特率:
| 波特率  | CAN标准  | 适用场景         |
| ------- | -------- | ---------------- |
| 500kbps | CAN 2.0B | 标准配置（推荐） |
| 250kbps | CAN 2.0B | 长距离（>10m）   |
| 1Mbps   | CAN 2.0B | 高速应用         |

**设置波特率**:
```bash
# 500kbps（默认）
sudo ip link set can0 type can bitrate 500000

# 确认配置
ip link show can0 | grep bitrate
```

### CAN ID分配

**默认分配方案** `config/hwt9053_params.yaml`:
```yaml
# 单机器人
hwt9053_can_driver:
  ros__parameters:
    robot_name: "robot"
    can_interface: "can0"
    
# 多机器人
robot1:
  hwt9053_can_driver:
    ros__parameters:
      robot_name: "robot1"
      can_interface: "can0"
      
robot2:
  hwt9053_can_driver:
    ros__parameters:
      robot_name: "robot2"
      can_interface: "can0"  # 可共享同一CAN总线
```

**CAN帧ID**（固定，不可配置）:
```
0x50: 时间戳数据（可选）
0x51: 加速度数据（必需）
0x52: 角速度数据（必需）
0x53: 欧拉角数据（3帧）
0x54: 磁场数据（可选）
```

### 诊断CAN总线

```bash
# 1. 查看实时CAN消息
candump can0

# 2. 过滤特定ID
candump can0,0x51:0xFF

# 3. 显示统计信息
candump can0 -s 5  # 每5秒显示统计

# 4. 记录CAN数据（供离线分析）
candump can0 > can_data.log

# 5. 回放CAN数据
canplayer -I can_data.log

# 6. 发送测试帧（调试用）
cansend can0 050#1122334455667788  # 发送ID 0x50的数据
```

---

## 多机器人集成

### 场景：3个机器人共用一条CAN总线

**拓扑**:
```
┌─────────────────────────────────────────────┐
│  Robot1        Robot2        Robot3         │
│  HWT9053#1     HWT9053#2     HWT9053#3      │
│  (CAN0)        (CAN0)        (CAN0)         │
│                                             │
│  ─── 共享 CAN0 总线 ─── 500kbps ────        │
│                                             │
│  [计算机]                                    │
│  ├─ ros2_control (hwt9053_driver x3)       │
│  └─ [融合处理单元]                          │
└─────────────────────────────────────────────┘

消息隔离:
/robot1/imu/data   ← HWT9053#1 数据
/robot2/imu/data   ← HWT9053#2 数据
/robot3/imu/data   ← HWT9053#3 数据
```

### 启动脚本 (`launch/imu_system.launch.py`)

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明参数
    robots_arg = DeclareLaunchArgument(
        'robots',
        default_value='robot1,robot2,robot3',
        description='逗号分隔的机器人名称列表'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时钟'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN接口名称'
    )
    
    robots = LaunchConfiguration('robots')
    use_sim_time = LaunchConfiguration('use_sim_time')
    can_interface = LaunchConfiguration('can_interface')
    
    # 为每个机器人创建节点
    nodes = []
    for robot in robots.split(','):
        nodes.append(
            Node(
                package='hwt9053_can_driver',
                executable='hwt9053_can_driver_node',
                name='hwt9053_can_driver',
                namespace=robot.strip(),
                parameters=[{
                    'robot_name': robot.strip(),
                    'can_interface': can_interface,
                    'use_sim_time': use_sim_time,
                }],
                output='screen'
            )
        )
    
    return LaunchDescription([
        robots_arg,
        use_sim_time_arg,
        can_interface_arg,
        GroupAction(actions=nodes)
    ])
```

**使用**:
```bash
# 启动所有机器人
ros2 launch hwt9053_can_driver imu_system.launch.py

# 自定义机器人列表
ros2 launch hwt9053_can_driver imu_system.launch.py \
  robots:="robot1,robot2" \
  can_interface:=can0

# 仿真环境
ros2 launch hwt9053_can_driver imu_system.launch.py \
  use_sim_time:=true
```

### 多机器人数据验证

```bash
# 在不同终端中监听各个机器人的数据
ros2 topic echo /robot1/imu/data
ros2 topic echo /robot2/imu/data
ros2 topic echo /robot3/imu/data

# 验证时间戳同步
ros2 topic echo /robot1/imu/data | grep -A1 "stamp:"
ros2 topic echo /robot2/imu/data | grep -A1 "stamp:"

# 查看所有话题
ros2 topic list | grep imu
```

---

## 性能调优

### CAN总线优化

**减少反射和干扰**:
```bash
# 1. 验证波特率错误率
ip -s link show can0
# 检查: RX errors, TX errors应为0或接近0

# 2. 监控总线负载
watch -n 1 'ip -s link show can0 | tail -10'

# 3. 检查帧到达率
candump can0 | wc -l  # 应约为 5帧×200Hz = 1000 msg/s

# 4. 启用CAN FD（如硬件支持）
sudo ip link set can0 type can bitrate 500000 dbitrate 2000000 fd on
```

### 性能分析

**监控驱动性能**:
```bash
# 启用调试日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  log_debug:=true 2>&1 | tee imu_debug.log

# 分析日志
grep "发布 IMU" imu_debug.log | wc -l  # 应约为 200 (1秒200Hz)

# 检查处理延滞
grep -oP '(?<=accel=\[)[\d\.,\s-]+' imu_debug.log | head -10
```

**CPU和内存使用**:
```bash
# 查看单个节点资源占用
ps aux | grep hwt9053_can_driver

# 持续监控
watch -n 1 'ps aux | grep hwt9053_can_driver'

# 使用ROS 2诊断工具
ros2 run diagnostic_aggregator aggregator_node  # 如果可用

# 话题带宽监控
ros2 monitor /robot1/imu/data
```

### 数据流验证

**检查消息频率**:
```bash
# 应为200Hz（±10%正常）
ros2 topic hz /robot1/imu/data

# 应输出类似: average rate: 200.05 Hz ...
```

**检查延滞**:
```bash
# ROS 2延滞数据
ros2 run topic_monitor topic_monitor /robot1/imu/data

# 或自定义脚本监控
python3 << 'EOF'
import rclpy
from sensor_msgs.msg import Imu
import time

def imu_callback(msg):
    now = rclpy.utils.timestamp_to_sec(rclpy.clock.Clock().now())
    recv_time = rclpy.utils.timestamp_to_sec(msg.header.stamp)
    delay = (now - recv_time) * 1000
    print(f"消息延滞: {delay:.2f} ms")

rclpy.init()
node = rclpy.create_node('imu_monitor')
sub = node.create_subscription(Imu, '/robot1/imu/data', imu_callback, 10)
rclpy.spin(node)
EOF
```

---

## 故障排查

### 无法识别CAN接口

```bash
# 症状: ip link show can0 显示 "No such device"

# 检查模块
lsmod | grep can

# 加载CAN模块
sudo modprobe can
sudo modprobe can_dev
sudo modprobe can_raw

# 检查USB设备
lsusb | grep -i can
dmesg | tail -20  # 查看内核日志
```

### CAN消息丢失

```bash
# 症状: candump显示间断的消息或频率不足200Hz

# 1. 检查错误帧
ip -s link show can0
# 查看 RX errors, TX errors, CAN RX errors等

# 2. 降低波特率测试
sudo ip link set can0 type can bitrate 250000
candump can0  # 观察是否改善

# 3. 检查电源质量
# 使用万用表测量电源稳定性

# 4. 重新配置接口
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### 多机器人数据交叉

```bash
# 症状: /robot1/imu/data中包含robot2的数据

# 检查CAN总线上的重复ID
candump can0 | sort | uniq -c

# 物理检查电气连接
# 确保多个传感器的CAN_H、CAN_L、GND连接正确
```

---

**最后更新**: 2026-03-16  
**维护者**: Xinyu Sheng
