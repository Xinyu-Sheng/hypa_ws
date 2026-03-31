#ifndef HWT9053_CAN_DRIVER__HWT9053_PARSER_HPP_
#define HWT9053_CAN_DRIVER__HWT9053_PARSER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>

#include "sensor_msgs/msg/imu.hpp"

namespace hwt9053_can_driver
{

// HWT9053 传感器数据结构
struct HWT9053Data
{
  // 加速度 (m/s^2)
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

  // 磁场 (T)
  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;

  // 温度 (°C)
  float temperature = 0.0f;

  // 硬件时间 (年月日时分秒)
  uint8_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;

  // 数据有效标志
  bool accel_valid{false};
  bool gyro_valid{false};
  bool angle_valid{false};
  bool mag_valid{false};
};

// HWT9053 数据解析器类（使用 PIMPL 模式）
class HWT9053Parser
{
  public:
  // WIT/TYPE 定义（按 High_Precision_Sensor_CAN_Protocol.txt）
  // 注意：驱动已修正为严格解析 WIT 封装（payload[0] == 0x55，payload[1] 为
  // TYPE）。常量名使用 `WIT_TYPE_*` 更能反映它们表示的是 WIT 帧中的 TYPE 字段，
  // 而不是 CAN ID。
  static constexpr uint8_t WIT_TYPE_TIME = 0x50;   // 时间数据 (TYPE)
  static constexpr uint8_t WIT_TYPE_ACCEL = 0x51;  // 加速度数据 (TYPE)
  static constexpr uint8_t WIT_TYPE_GYRO = 0x52;   // 角速度数据 (TYPE)
  static constexpr uint8_t WIT_TYPE_ANGLE = 0x53;  // 角度数据 (TYPE)
  static constexpr uint8_t WIT_TYPE_MAGN = 0x54;   // 磁场数据 (TYPE)

  // 解析权重常量 (物理量转换系数)
  static constexpr float ACCEL_SCALE =
      16.0f * 9.81f / 32768.0f;  // m/s^2 per LSB
  static constexpr float GYRO_SCALE =
      2000.0f * 3.14159265358979323846f / 180.0f / 32768.0f;  // rad/s per LSB
  static constexpr float ANGLE_SCALE =
      1.0f / 1000.0f * 3.14159265358979323846f / 180.0f;  // rad per LSB
  static constexpr float MAG_SCALE = 13.0e-9f;            // T per LSB

  HWT9053Parser();
  ~HWT9053Parser();

  /**
   * @brief 解析 WIT 封装的 CAN 帧数据（要求 payload[0] == 0x55，payload[1] 为
   * TYPE）
   * @param _data CAN 数据字节数组（通常长度为 8）
   * @param _dlc 数据长度
   * @return 解析成功返回 true，失败返回 false（如 DLC 无效或 TYPE 不支持）
   */
  bool ParseCANFrame(const std::array<uint8_t, 8> &_data, uint8_t _dlc);

  /**
   * @brief 将解析的数据转换为 IMU 消息
   * @return sensor_msgs::msg::Imu 消息
   */
  sensor_msgs::msg::Imu ToIMUMessage() const;

  /**
   * @brief 获取传感器原始数据
   * @return HWT9053Data 结构体
   */
  HWT9053Data GetData() const;

  /**
   * @brief 获取磁场数据快照（线程安全）
   * @return 返回磁场数据副本
   */
  struct MagneticFieldData
  {
    float mag_x;
    float mag_y;
    float mag_z;
    bool mag_valid;
  };

  MagneticFieldData GetMagneticFieldData() const;

  /**
   * @brief 重置数据
   */
  void Reset();

  /**
   * @brief 设置线性加速度方差（variance）
   * @param _variance 方差值 (m/s^2)^2
   */
  void SetAccelVariance(double _variance);

  /**
   * @brief 设置数据过期超时（毫秒）。当某类数据超过此阈值未更新时，相关 *_valid
   * 会被视为 false。
   * @param _ms 超时时间，单位毫秒
   */
  void SetDataTimeoutMs(uint32_t _ms);

  /**
   * @brief 设置角度组帧窗口（毫秒）。三轴角度时间戳最大差值超过该窗口时，
   * angle_valid 会被判定为 false。
   * @param _ms 组帧窗口，单位毫秒
   */
  void SetAngleAssemblyWindowMs(uint32_t _ms);

  /**
   * @brief 设置角速度方差（variance）
   * @param _variance 方差值 (rad/s)^2
   */
  void SetGyroVariance(double _variance);

  private:
  // PIMPL：私有数据实现
  class Impl;
  std::unique_ptr<Impl> pimpl_;

  // 内部辅助函数
  int16_t BytesToInt16(uint8_t _high, uint8_t _low) const;
  uint16_t BytesToUInt16(uint8_t _high, uint8_t _low) const;
  int32_t BytesToInt32(uint8_t _byte0, uint8_t _byte1, uint8_t _byte2,
                       uint8_t _byte3) const;
  float Int16ToFloat(int16_t _value, float _scale) const;
};

}  // namespace hwt9053_can_driver

#endif  // HWT9053_CAN_DRIVER__HWT9053_PARSER_HPP_
