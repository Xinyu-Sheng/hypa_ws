# HWT9053 驱动 - API 参考文档

**版本**: v0.0.2  
**最后更新**: 2026-03-16

---

## 目录

1. [HWT9053Parser](#hwt9053parser)
2. [HWT9053CANDriverNode](#hwt9053candrivernode)
3. [消息类型](#消息类型)
4. [数据结构](#数据结构)
5. [常量定义](#常量定义)

---

## HWT9053Parser

### 类定义

```cpp
namespace hwt9053_can_driver {

class HWT9053Parser {
public:
  // CAN ID 常量
  static constexpr uint32_t CAN_ID_TIME = 0x50;     // 时间戳数据
  static constexpr uint32_t CAN_ID_ACCEL = 0x51;    // 加速度数据
  static constexpr uint32_t CAN_ID_GYRO = 0x52;     // 陀螺仪数据
  static constexpr uint32_t CAN_ID_ANGLE = 0x53;    // 欧拉角数据
  static constexpr uint32_t CAN_ID_MAGN = 0x54;     // 磁场数据

  // 构造和析构
  HWT9053Parser();
  ~HWT9053Parser();

  // 主要接口
  bool ParseCANFrame(uint32_t _can_id, const std::array<uint8_t, 8> &_data, uint8_t _dlc);
  sensor_msgs::msg::Imu ToIMUMessage() const;
  const HWT9053Data &GetData() const;
  struct MagneticFieldData { float mag_x, mag_y, mag_z; bool mag_valid; };
  MagneticFieldData GetMagneticFieldData() const;
  void Reset();

private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
  
  // 内部辅助函数
  int16_t BytesToInt16(uint8_t _high, uint8_t _low) const;
  uint16_t BytesToUInt16(uint8_t _high, uint8_t _low) const;
  int32_t BytesToInt32(uint8_t _byte0, uint8_t _byte1, uint8_t _byte2, uint8_t _byte3) const;
  float Int16ToFloat(int16_t _value, float _scale) const;
};

} // namespace hwt9053_can_driver
```

### 成员函数详解

#### ParseCANFrame()

**功能**: 解析单帧CAN数据

```cpp
bool ParseCANFrame(uint32_t _can_id, const std::array<uint8_t, 8> &_data, uint8_t _dlc);
```

**参数**:
- `_can_id` (uint32_t): CAN ID (0x50-0x54)
- `_data` (const std::array<uint8_t, 8>&): 8字节CAN数据
- `_dlc` (uint8_t): 数据长度代码（必须为8）

**返回值**:
- `true`: 解析成功
- `false`: 解析失败（如DLC≠8或角度类型无效）

**异常**: 无异常抛出，通过返回值表示失败

**示例**:
```cpp
std::array<uint8_t, 8> data = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
bool success = parser.ParseCANFrame(0x51, data, 8);
if (!success) {
  RCLCPP_WARN(logger, "Failed to parse CAN frame");
}
```

---

#### ToIMUMessage()

**功能**: 获取当前解析数据对应的IMU消息

```cpp
sensor_msgs::msg::Imu ToIMUMessage() const;
```

**返回值**: 标准ROS 2 IMU消息

**特性**:
- 线程安全（使用互斥锁保护）
- 包含完整的线性加速度、角速度、四元数
- 包含规格验证过的协方差矩阵

**四元数转换**:
```
使用ZYX欧拉角顺序：
q.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2)
q.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2)
q.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2)
q.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2)
```

**示例**:
```cpp
auto imu_msg = parser.ToIMUMessage();
// 填充header
imu_msg.header.stamp = now();
imu_msg.header.frame_id = "imu_link";
// 发布
imu_publisher->publish(imu_msg);
```

---

#### GetMagneticFieldData()

**功能**: 获取磁场数据（线程安全）

```cpp
struct MagneticFieldData {
  float mag_x;        // X轴磁场 (µT)
  float mag_y;        // Y轴磁场 (µT)
  float mag_z;        // Z轴磁场 (µT)
  bool mag_valid;     // 数据有效标志
};

MagneticFieldData GetMagneticFieldData() const;
```

**返回值**: MagneticFieldData 结构体副本

**特性**:
- 线程安全（原子操作）
- 返回数据副本，避免悬挂指针
- 包含有效性标志

**示例**:
```cpp
auto mag_data = parser.GetMagneticFieldData();
if (mag_data.mag_valid) {
  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.magnetic_field.x = mag_data.mag_x;
  mag_msg.magnetic_field.y = mag_data.mag_y;
  mag_msg.magnetic_field.z = mag_data.mag_z;
  mag_publisher->publish(mag_msg);
}
```

---

#### GetData()

**功能**: 获取传感器完整原始数据

```cpp
const HWT9053Data &GetData() const;
```

**返回值**: HWT9053Data结构体的const引用

**警告**: 
- 返回引用，调用者需谨慎处理多线程访问
- 不推荐在多executor环境中直接使用
- 建议使用 `GetMagneticFieldData()` 替代

**示例**:
```cpp
const auto &raw_data = parser.GetData();
std::cout << "Accel X: " << raw_data.accel_x << " m/s²" << std::endl;
```

---

#### Reset()

**功能**: 重置所有传感器数据

```cpp
void Reset();
```

**效果**:
- 所有数据值初始化为0
- 所有有效标志设置为false
- 角度缓冲计数器重置

**使用场景**:
- 节点启动时
- 传感器故障后恢复
- 需要清空历史数据时

---

## HWT9053CANDriverNode

### 类定义

```cpp
class HWT9053CANDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit HWT9053CANDriverNode(const rclcpp::NodeOptions &_options = rclcpp::NodeOptions());
  ~HWT9053CANDriverNode();

  // 生命周期回调
  CallbackReturn on_configure(const rclcpp_lifecycle::State &_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &_state) override;

private:
  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr _msg);
};
```

### 生命周期状态机

```
┌─────────┐
│Unconfigured│
└─────────┘
    │(on_configure)
    ▼
┌─────────┐
│Inactive│ ◄─────────────┐
└─────────┘              │
    │(on_activate)       │(on_deactivate)
    ▼                    │
┌─────────┐              │
│Active   │──────────────┘
└─────────┘
    │(on_shutdown/on_cleanup)
    ▼
┌ ─────────       ┐
│Finalized│
└─────────┘
```

### 节点参数

| 参数             | 类型   | 默认值         | 必需 | 描述            |
| ---------------- | ------ | -------------- | ---- | --------------- |
| `robot_name`     | string | "robot"        | 否   | 机器人标识符    |
| `can_interface`  | string | "can0"         | 否   | CAN接口名称     |
| `imu_frame_id`   | string | "imu_link"     | 否   | IMU坐标系ID     |
| `imu_topic_name` | string | "imu/data"     | 否   | IMU话题相对路径 |
| `can_bus_topic`  | string | "from_can_bus" | 否   | CAN总线话题     |
| `use_sim_time`   | bool   | false          | 否   | 使用仿真时钟    |
| `log_debug`      | bool   | false          | 否   | 调试日志输出    |

### 话题接口

**订阅**:
- `from_can_bus` (can_msgs/Frame): CAN总线消息

**发布**:
- `imu/data` (sensor_msgs/Imu): IMU测量数据 (200Hz)
- `imu/data_mag` (sensor_msgs/MagneticField): 磁场数据 (200Hz，有效时)

---

## 消息类型

### sensor_msgs/Imu

标准ROS 2 IMU消息，包含：

```yaml
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**协方差矩阵排列** (行优先):
```
[xx, xy, xz,
 yx, yy, yz,
 zx, zy, zz]
```

对于本驱动，仅对角线元素有效：
- `linear_acceleration_covariance`: [3.4e-5, 0, 0, 0, 3.4e-5, 0, 0, 0, 3.4e-5]
- `angular_velocity_covariance`: [5.8e-8, 0, 0, 0, 5.8e-8, 0, 0, 0, 5.8e-8]

---

### sensor_msgs/MagneticField

```yaml
std_msgs/Header header
geometry_msgs/Vector3 magnetic_field       # µT
float64[9] magnetic_field_covariance       # (µT)²
```

---

## 数据结构

### HWT9053Data

```cpp
struct HWT9053Data {
  // 加速度 (m/s²)
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;

  // 角速度 (rad/s)
  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;

  // 欧拉角 (rad)
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  // 磁场 (µT)
  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;

  // 温度 (°C)
  float temperature = 0.0f;

  // 硬件时间戳 (ms)
  uint32_t hw_timestamp = 0;

  // 数据有效标志
  bool accel_valid = false;
  bool gyro_valid = false;
  bool angle_valid = false;  // 三个欧拉角齐全时设置
  bool mag_valid = false;
};
```

---

## 常量定义

### CAN ID

| 常量         | 值   | 含义       |
| ------------ | ---- | ---------- |
| CAN_ID_TIME  | 0x50 | 时间戳数据 |
| CAN_ID_ACCEL | 0x51 | 加速度数据 |
| CAN_ID_GYRO  | 0x52 | 角速度数据 |
| CAN_ID_ANGLE | 0x53 | 欧拉角数据 |
| CAN_ID_MAGN  | 0x54 | 磁场数据   |

### 缩放系数

```cpp
// 加速度: ±2g
constexpr float ACCEL_SCALE = 2.0f * 2.0f * 9.81f / 32768.0f;  // ≈ 0.001197 m/s²

// 角速度: ±2000°/s
constexpr float GYRO_SCALE = 2000.0f * M_PI / 180.0f / 32768.0f;  // ≈ 0.00106 rad/s

// 欧拉角
constexpr float ANGLE_SCALE = 1.0f / 1000.0f * M_PI / 180.0f;  // rad

// 磁场: ±400µT
constexpr float MAG_SCALE = 0.013f;  // µT/LSB
```

### 协方差值

```cpp
// 线性加速度协方差 (硬件规格)
constexpr double ACCEL_COVARIANCE = 3.4e-5;  // (m/s²)²

// 角速度协方差 (硬件规格)
constexpr double GYRO_COVARIANCE = 5.8e-8;  // (rad/s)²

// 磁场协方差 (估计)
constexpr double MAG_COVARIANCE = 0.0001;   // (µT)²
```

---

## 线程安全

### Pimpl 实现

```cpp
class HWT9053Parser::Impl {
private:
  HWT9053Data data;
  mutable std::mutex data_mutex;  // 保护并发访问
  
  // 欧拉角缓冲状态（仅在单线程ParseCANFrame中使用）
  bool roll_ready, pitch_ready, yaw_ready;
};
```

### 线程安全保证

| 函数                 | 安全性   | 说明                     |
| -------------------- | -------- | ------------------------ |
| ParseCANFrame        | ✅ 安全   | 单线程CAN回调中调用      |
| ToIMUMessage         | ✅ 安全   | 使用lock_guard保护读操作 |
| GetMagneticFieldData | ✅ 安全   | 返回数据副本             |
| GetData              | ⚠️ 有风险 | 返回引用，多线程需小心   |
| Reset                | ✅ 安全   | 受互斥锁保护             |

---

## 错误处理

### ParseCANFrame 返回值

```cpp
bool success = parser.ParseCANFrame(can_id, data, dlc);

if (!success) {
  // 可能的失败原因：
  // 1. DLC != 8 (非法数据长度)
  // 2. angle_type 无效（不是 0x01/0x02/0x03）
}
```

### 日志输出

启用调试日志:
```bash
ros2 launch hwt9053_can_driver hwt9053_driver.launch.py log_debug:=true
```

日志示例：
```
[INFO] 正在配置 HWT9053 CAN 驱动节点
[INFO] 参数设置: robot_name=robot1, can_interface=can0
[DEBUG] 接收 CAN 帧 ID=0x51, DLC=8
[DEBUG] 发布 IMU 消息: accel=[0.123, 0.456, 9.810] m/s^2
[WARN] CAN 帧解析失败: ID=0x53, DLC=7 (期望 DLC=8)
```

---

## 性能优化

### CPU占用

- 解析开销: ~0.1ms/帧
- 消息转换: ~0.05ms
- 互斥锁: ~0.01ms

**总计**: ~0.2ms @200Hz ≈ 4% 单核CPU

### 内存占用

- 对象大小: ~200字节
- 为每帧分配: ~100字节
- ROS 2框架: ~20MB
- 驱动总计: ~20.2MB

---

## 扩展开发

### 添加新的CAN ID

1. 在 `hwt9053_parser.hpp` 中添加常量：
```cpp
static constexpr uint32_t CAN_ID_CUSTOM = 0x55;
```

2. 在 `ParseCANFrame()` 中添加case分支

3. 更新 `HWT9053Data` 结构体添加新字段

4. 在 `ToIMUMessage()` 中处理新数据（如需要）

### 自定义协方差

编辑 `hwt9053_parser.cpp`:
```cpp
// 修改这些常量
constexpr double ACCEL_COVARIANCE = 3.4e-5;   // 您的测试值
constexpr double GYRO_COVARIANCE = 5.8e-8;    // 您的测试值
```

---

**最后更新**: 2026-03-16  
**维护者**: Xinyu Sheng
