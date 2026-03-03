// Copyright 2026 Xinyu Sheng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HWT9053_CAN_DRIVER__HWT9053_PARSER_HPP_
#define HWT9053_CAN_DRIVER__HWT9053_PARSER_HPP_

#include <cstdint>
#include <array>
#include <memory>

#include "sensor_msgs/msg/imu.hpp"

namespace hwt9053_can_driver
{

// HWT9053 传感器数据结构
struct HWT9053Data
{
  // 加速度 (m/s^2)
  float accel_x;
  float accel_y;
  float accel_z;

  // 角速度 (rad/s)
  float gyro_x;
  float gyro_y;
  float gyro_z;

  // 欧拉角 (rad)
  float roll;
  float pitch;
  float yaw;

  // 磁场 (uT)
  float mag_x;
  float mag_y;
  float mag_z;

  // 温度 (°C)
  float temperature;

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
  // HWT9053 CAN ID 定义
  static constexpr uint32_t CAN_ID_ACCEL = 0x50;   // 加速度数据
  static constexpr uint32_t CAN_ID_GYRO = 0x51;    // 角速度数据
  static constexpr uint32_t CAN_ID_ANGLE = 0x52;   // 角度数据
  static constexpr uint32_t CAN_ID_MAGN = 0x53;    // 磁场数据
  static constexpr uint32_t CAN_ID_STATUS = 0x54;  // 温度和状态

  HWT9053Parser();
  ~HWT9053Parser();

  /**
   * @brief 解析 CAN 帧数据
   * @param _can_id CAN ID
   * @param _data CAN 数据字节数组
   * @param _dlc 数据长度
   */
  void ParseCANFrame(uint32_t _can_id, const std::array<uint8_t, 8>& _data,
                     uint8_t _dlc);

  /**
   * @brief 将解析的数据转换为 IMU 消息
   * @return sensor_msgs::msg::Imu 消息
   */
  sensor_msgs::msg::Imu ToIMUMessage() const;

  /**
   * @brief 获取传感器原始数据
   * @return HWT9053Data 结构体
   */
  const HWT9053Data& GetData() const;

  /**
   * @brief 重置数据
   */
  void Reset();

 private:
  // PIMPL：私有数据实现
  class Impl;
  std::unique_ptr<Impl> pimpl_;

  // 内部辅助函数
  int16_t BytesToInt16(uint8_t _high, uint8_t _low) const;
  uint16_t BytesToUInt16(uint8_t _high, uint8_t _low) const;
  float Int16ToFloat(int16_t _value, float _scale) const;
};

}  // namespace hwt9053_can_driver

#endif  // HWT9053_CAN_DRIVER__HWT9053_PARSER_HPP_
