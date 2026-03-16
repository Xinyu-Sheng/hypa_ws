# HWT9053 驱动 - 故障排查指南

**版本**: v0.0.2  
**最后更新**: 2026-03-16

---

## 快速诊断流程

```
问题出现
    ↓
┌─ IMU数据为空或无消息？
│  └─ 是 → [诊断1: 消息接收问题]
│  └─ 否 → [诊断2: 数据质量问题]
│
└─ 多机器人系统？
   └─ 是 → [诊断3: 多机器人问题]
   └─ 否 → [诊断1]
```

---

## 诊断1: 消息接收问题

### 症状

- 话题出现消息为空
- `ros2 topic hz` 显示频率为0
- 节点状态为"Active"但无数据流

### 排查步骤

#### 1.1 验证CAN接口状态

```bash
# 步骤1: 检查接口
ip link show can0

# 预期输出:
# 2: can0: <LOOPBACK,UP,LOWER_UP,ECHO> mtu 16
#   CAN
#   bitrate 500000 sample-point 0.750

# 如果显示 "DOWN"，执行:
sudo ip link set can0 up

# 如果接口不存在，执行:
sudo ip link add can0 type can bitrate 500000
sudo ip link set can0 up
```

**诊断结果**:
- ✅ 接口UP且RUNNING → 进入步骤1.2
- ❌ 接口DOWN或不存在 → 检查CAN硬件连接

---

#### 1.2 监听CAN总线

```bash
# 连接USB-CAN或其他CAN接口
candump can0 -n 10

# 预期输出（应显示5条不同的CAN ID）:
# can0 050 [8] 48 D3 00 00 01 00 00 C8
# can0 051 [8] 12 34 56 78 9A BC DE F0
# can0 052 [8] AB CD EF 01 23 45 67 89
# can0 053 [8] 01 00 E8 03 00 00 00 00
# can0 054 [8] FF FF FF FF 00 00 00 00
```

**诊断问题**:

❌ **显示 "RTNETLINK answers: Cannot assign requested address"**
```bash
# 原因: CAN接口配置错误
# 修复:
sudo ip link set can0 down
sudo ip link del can0
sudo modprobe can
sudo modprobe can_dev
sudo ip link add can0 type can bitrate 500000
sudo ip link set can0 up
```

❌ **没有任何消息显示**
```bash
# 原因: CAN硬件未连接、总线无数据、或接线问题
# 排查:
  1. 检查CAN收发器电源: voltmeter → CAN_H vs GND（应为2.5V±1.5V）
  2. 检查总线连接: 所有节点的CAN_H/CAN_L是否正确连接
  3. 查看dmesg日志: dmesg | tail -20
  4. 尝试发送测试帧: cansend can0 050#1122334455667788
  5. 检查HWT9053传感器电源和CAN引脚是否正常
```

❌ **消息间断或频率低**
```bash
# 原因: CAN波特率不匹配、电磁干扰、或总线过载
# 排查:
  1. 验证HWT9053设置的波特率: 250kbps / 500kbps / 1Mbps
  2. 尝试降低波特率: sudo ip link set can0 type can bitrate 250000
  3. 用示波器测量CAN_H和CAN_L信号质量
  4. 检查 can0 上的错误: ip -s link show can0
```

✅ **显示连续的CAN消息** → 进入步骤1.3

---

#### 1.3 验证节点和话题

```bash
# 检查ROS 2环境
source ~/hypa_ws/install/setup.bash  # 确保环境正确

# 列出节点
ros2 node list

# 应包含:
# /robot/hwt9053_can_driver
# 或 /robot1/hwt9053_can_driver 等

# 如果节点不存在，启动驱动:
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot

# 列出话题
ros2 topic list

# 应包含:
# /{robot_name}/imu/data
# /{robot_name}/imu/data_mag (可选)
```

❌ **节点/话题不存在**
```bash
# 原因: 驱动未启动
# 修复: 启动驱动节点

# 或检查启动日志:
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py 2>&1 | grep -E "ERROR|WARN"
```

✅ **节点和话题正常** → 进入步骤1.4

---

#### 1.4 监听话题消息

```bash
# 监听IMU话题（1条消息后退出）
ros2 topic echo /{robot_name}/imu/data -n 1

# 示例输出:
# header:
#   stamp:
#     sec: 1710664616
#     nanosec: 123456789
#   frame_id: imu_link
# orientation:
#   x: 0.0
#   y: 0.0
#   z: 0.0
#   w: 1.0
# ...
```

❌ **没有消息打印（卡住等待）**
```bash
# 原因: 话题存在但未发布消息
# 排查:
  1. 启用调试日志:
     ros2 launch hwt9053_can_driver hwt9053_driver.launch.py log_debug:=true
     
  2. 观察日志输出:
     tail -f ~/.ros/log/latest/hwt9053_can_driver/...

  3. 检查CAN回调是否触发:
     grep "接收 CAN 帧" ~/.ros/log/latest/.../stdouterr.log
     
  4. 检查解析是否成功:
     grep "发布 IMU 消息" ~/.ros/log/latest/.../stdouterr.log
```

❌ **消息错误（header为空、数据全0）**
```bash
# 原因: CAN帧解析失败或传感器未初始化
# 排查:
  1. 查看CAN帧内容: candump can0 | head -20
  2. 与datasheet对比，检查字节序和缩放
  3. 断电后重新启动HWT9053传感器
  4. 检查CAN帧的DLC（应为8）
```

✅ **消息正常发布** → 诊断完成！

---

## 诊断2: 数据质量问题

### 症状

- 消息能收到但数据异常
- 数值超出合理范围
- 四元数未归一化
- 频率不稳定

### 排查步骤

#### 2.1 检查数据范围

运行硬件测试脚本:
```bash
cd ~/hypa_ws
python3 src/drivers/hwt9053_can_driver/scripts/hardware_test.py \
  --robot robot \
  --duration 10
```

**预期范围**:
| 数据       | 范围   | 单位  | 备注       |
| ---------- | ------ | ----- | ---------- |
| 加速度     | ±19.62 | m/s²  | ±2g量程    |
| 角速度     | ±34.91 | rad/s | ±2000°/s   |
| 四元数范数 | ~1.0   | -     | 允许±1%    |
| 磁场       | ±5200  | µT    | ±400µT量程 |

❌ **加速度数据超出范围**
```bash
# 原因: 缩放系数错误（通常是字节序问题）
# 检查: 查看 hwt9053_parser.cpp 中的 ACCEL_SCALE
# 应为: 2.0f * 2.0f * 9.81f / 32768.0f ≈ 0.001197

# 验证字节序:
# CAN接收: [LSB, MSB, ...]
# 转换: BytesToInt16(data[1], data[0])  # MSB在位置1

# 如果错误，修改:
# int16_t ax_raw = this->BytesToInt16(_data[1], _data[0]);  // 正确
# int16_t ax_raw = this->BytesToInt16(_data[0], _data[1]);  // 错误
```

❌ **四元数未归一化**
```bash
# 原因: 欧拉角数据丢失或计算错误
# 检查: 欧拉角是否都有效（roll, pitch, yaw非零）
ros2 topic echo /{robot_name}/imu/data | grep -E "roll|pitch|yaw"

# 如果都为0，检查0x53 CAN帧是否对齐:
candump can0 | grep 053

# 应显示3种数据类型:
# 053 [8] 01 00 ... (Roll)
# 053 [8] 02 00 ... (Pitch)
# 053 [8] 03 00 ... (Yaw)
```

#### 2.2 检查消息频率

```bash
# 测量实际频率
ros2 topic hz /{robot_name}/imu/data

# 期望输出: average rate: 200.00 Hz, max: 200 Hz, std dev: 0.0 Hz

# 允许范围: 200 ± 10Hz（±5%）
```

❌ **频率偏低（<180Hz）**
```bash
# 原因: CPU过载、ROS 2处理延滞、或CAN消息丢失
# 排查:
  1. 检查CPU占用: top -p $(pgrep hwt9053_can_driver)
  2. 检查ROS 2执行器: ros2 run rclcpp_components component_container  
  3. 查看CAN错误: ip -s link show can0 (RX errors, TX errors)
  4. 检查消息队列大小: ros2 topic info /{robot_name}/imu/data
```

❌ **频率波动大（std dev >5Hz）**
```bash
# 原因: 不稳定的消息流、或CAN总线干扰
# 排查:
  1. 更换CAN线（使用屏蔽线）
  2. 降低波特率: sudo ip link set can0 type can bitrate 250000
  3. 检查电源稳定性: voltmeter → CAN电源
  4. 查看CAN信号质量: 示波器 CAN_H vs CAN_L
```

✅ **频率正常** → 进入步骤2.3

#### 2.3 协方差验证

```bash
# 查看协方差值
ros2 topic echo /{robot_name}/imu/data | grep -A 9 "linear_acceleration_covariance"

# 期望值:
# linear_acceleration_covariance: [3.4e-05, 0.0, 0.0, 0.0, 3.4e-05, 0.0, 0.0, 0.0, 3.4e-05]
# angular_velocity_covariance: [5.8e-08, 0.0, 0.0, 0.0, 5.8e-08, 0.0, 0.0, 0.0, 5.8e-08]
```

⚠️ **协方差值不匹配**
```bash
# 可能原因: 在硬件测试中调整过协方差值
# 动作: 根据实际应用调整，或保持默认值
```

---

## 诊断3: 多机器人问题

### 症状

- 多个机器人时数据交叉混乱
- 话题命名空间冲突
- 部分机器人无数据

### 排查步骤

#### 3.1 验证命名空间隔离

```bash
# 启动多个机器人
ros2 launch hwt9053_can_driver imu_system.launch.py \
  robots:="robot1,robot2,robot3"

# 验证话题
ros2 topic list | grep imu

# 期望输出:
# /robot1/imu/data
# /robot1/imu/data_mag
# /robot2/imu/data
# /robot2/imu/data_mag
# /robot3/imu/data
# /robot3/imu/data_mag

# 不应contains:
# /imu/data (全局命名空间)
```

❌ **话题在全局命名空间**
```bash
# 原因: launch文件未正确设置namespace
# 检查: imu_system.launch.py 中的 namespace 字段
# 修复示例:
  Node(
    package='hwt9053_can_driver',
    namespace=robot_name,  # 添加此行
    ...
  )
```

#### 3.2 验证每个机器人的数据

```bash
# 独立监听每个机器人
ros2 topic echo /robot1/imu/data --rate 1 > /tmp/robot1.log &
ros2 topic echo /robot2/imu/data --rate 1 > /tmp/robot2.log &
sleep 10
killall ros2

# 对比数据
diff /tmp/robot1.log /tmp/robot2.log

# 应显示完全不同的数据（不是相同值重复）
# 如果完全相同，检查CAN ID分配
```

❌ **某个机器人无数据**
```bash
# 原因: 对应的CAN收发器故障、或总线连接中断
# 排查:
  1. 单独启动该机器人: ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot2
  2. 用candump监听: candump can0
  3. 检查该传感器的CAN消息是否存在
  4. 验证电源连接和总线接线
```

#### 3.3 验证共享CAN总线

```bash
# 验证所有消息在同一条CAN总线上
candump can0 | head -20

# 应显示多个相同ID但内容不同的消息（来自不同传感器）

# 计算消息ID分布
candump can0 -n 100 | awk '{print $2}' | sort | uniq -c

# 期望: 0x50, 0x51, 0x52, 0x53, 0x54 各出现多次
```

⚠️ **消息来自不同的CAN总线**
```bash
# 如果使用多条CAN总线（can0, can1）：
candump can0 can1

# 验证每个驱动使用正确的接口:
grep "can_interface" ~/.ros/log/.../stdouterr.log
```

---

## 通用修复方法

### 方法A: 重新初始化系统

```bash
# 1. 停止所有ROS 2节点
killall -9 ros2_daemon

# 2. 重新配置CAN接口
sudo ip link set can0 down
sudo ip link del can0
sudo ip link add can0 type can bitrate 500000
sudo ip link set can0 up

# 3. 重启驱动
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py robot_name:=robot

# 4. 验证
candump can0 -n 5
ros2 topic echo /{robot_name}/imu/data -n 1
```

### 方法B: 使用硬件测试脚本

```bash
# 全面诊断
python3 src/drivers/hwt9053_can_driver/scripts/hardware_test.py \
  --can-interface can0 \
  --robot robot1 \
  --duration 30

# 仅检查CAN
python3 src/drivers/hwt9053_can_driver/scripts/hardware_test.py \
  --can-interface can0 \
  --test can
```

### 方法C: 启用详细日志

```bash
# 启用驱动调试日志
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py \
  robot_name:=robot \
  log_debug:=true \
  2>&1 | tee debug_output.log

# 保存日志供后续分析
grep "ERROR\|WARN" debug_output.log
```

---

## 问题排查表

| 问题          | 可能原因          | 检查方法            | 修复方法               |
| ------------- | ----------------- | ------------------- | ---------------------- |
| CAN接口不存在 | 驱动未加载        | `lsmod \| grep can` | `modprobe can can_dev` |
| 无CAN消息     | 硬件故障/接线错误 | `candump can0`      | 重新接线/更换硬件      |
| 消息中断      | 波特率不匹配      | 查看HWT9053配置     | 调整 `bitrate` 参数    |
| 文档缺失      | CAN线太长         | 测量总线长度        | 添加终端电阻           |
| 数据错误      | 字节序/缩放       | 对比datasheet       | 修改解析代码           |
| 频率低        | CPU过载           | `top -p $(pgrep...` | 优化代码/增加硬件      |
| 多机器人混乱  | namespace冲突     | `ros2 topic list`   | 修改launch配置         |

---

## 获取技术支持

### 收集诊断信息

遇到问题时，请收集以下信息：

```bash
# 1. 系统信息
uname -a
ros2 --version

# 2. CAN状态
ip link show can0
ip -s link show can0

# 3. ROS 2信息
ros2 node list
ros2 topic list -t

# 4. 日志文件
ls -la ~/.ros/log/latest/

# 5. 硬件测试结果
python3 src/drivers/hwt9053_can_driver/scripts/hardware_test.py

# 打包所有信息:
tar czf hwt9053_diagnostics.tar.gz \
  ~/.ros/log/ \
  hardware_test_output.log
```

### 联系方式

- **维护者**: Xinyu Sheng
- **邮箱**: sheng.xin.yu@faxmail.com
- **GitHub**: [hypa_workspace](https://github.com/example/hypa_ws)

### 提交Bug报告

在提交issue时，请包含：
1. 问题描述和重现步骤
2. 上述诊断信息
3. 期望行为 vs 实际行为
4. 硬件和软件版本

---

**最后更新**: 2026-03-16  
**维护者**: Xinyu Sheng
