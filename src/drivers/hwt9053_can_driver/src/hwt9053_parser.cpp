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

#include "hwt9053_can_driver/hwt9053_parser.hpp"

#include <cmath>

namespace hwt9053_can_driver
{

// PIMPL 实现
class HWT9053Parser::Impl
{
  public:
  HWT9053Data data;
};

HWT9053Parser::HWT9053Parser() : pimpl_(std::make_unique<Impl>())
{
}

HWT9053Parser::~HWT9053Parser() = default;

int16_t HWT9053Parser::BytesToInt16(uint8_t _high, uint8_t _low) const
{
  int16_t value = static_cast<int16_t>((_high << 8) | _low);
  return value;
}

uint16_t HWT9053Parser::BytesToUInt16(uint8_t _high, uint8_t _low) const
{
  uint16_t value = static_cast<uint16_t>((_high << 8) | _low);
  return value;
}

int32_t HWT9053Parser::BytesToInt32(uint8_t _byte0, uint8_t _byte1,
                                    uint8_t _byte2, uint8_t _byte3) const
{
  // 按协议格式组合32位有符号整数：byte0为低位，byte3为高位
  int32_t value = (static_cast<int32_t>(_byte3) << 24) |
                  (static_cast<int32_t>(_byte2) << 16) |
                  (static_cast<int32_t>(_byte1) << 8) |
                  (static_cast<int32_t>(_byte0));
  return value;
}

float HWT9053Parser::Int16ToFloat(int16_t _value, float _scale) const
{
  return _value * _scale;
}

void HWT9053Parser::ParseCANFrame(uint32_t _can_id,
                                  const std::array<uint8_t, 8> &_data,
                                  uint8_t _dlc)
{
  if (_dlc != 8)
  {
    return;
  }

  switch (_can_id)
  {
    case HWT9053Parser::CAN_ID_TIME:
    {
      // 时间数据（当前并不使用）
      break;
    }

    case HWT9053Parser::CAN_ID_ACCEL:
    {
      // 加速度数据格式: [AxL, AxH, AyL, AyH, AzL, AzH, ?, ?]
      // 量程: ±2g, 分辨率: 1/16384 * 4g = 0.000244 m/s^2
      // 正确计算: (2g * 2 * 9.81) / 32768 = ~0.001197 m/s^2
      constexpr float ACCEL_SCALE =
          2.0f * 2.0f * 9.81f / 32768.0f;  // ~0.001197 m/s^2

      // 低字节在 Data[0], 高字节在 Data[1]（按 CAN 协议）
      int16_t ax_raw = this->BytesToInt16(_data[1], _data[0]);
      int16_t ay_raw = this->BytesToInt16(_data[3], _data[2]);
      int16_t az_raw = this->BytesToInt16(_data[5], _data[4]);

      this->pimpl_->data.accel_x = this->Int16ToFloat(ax_raw, ACCEL_SCALE);
      this->pimpl_->data.accel_y = this->Int16ToFloat(ay_raw, ACCEL_SCALE);
      this->pimpl_->data.accel_z = this->Int16ToFloat(az_raw, ACCEL_SCALE);
      this->pimpl_->data.accel_valid = true;
      break;
    }

    case HWT9053Parser::CAN_ID_GYRO:
    {
      // 角速度数据格式: [GxL, GxH, GyL, GyH, GzL, GzH, ?, ?]
      // 量程: ±2000°/s, 分辨率: 1/16.4 °/s = 0.061 °/s = 0.00106 rad/s
      constexpr float GYRO_SCALE =
          2000.0f * M_PI / 180.0f / 32768.0f;  // ~0.00106 rad/s

      // 低字节在 Data[0], 高字节在 Data[1]（按 CAN 协议）
      int16_t gx_raw = this->BytesToInt16(_data[1], _data[0]);
      int16_t gy_raw = this->BytesToInt16(_data[3], _data[2]);
      int16_t gz_raw = this->BytesToInt16(_data[5], _data[4]);

      this->pimpl_->data.gyro_x = this->Int16ToFloat(gx_raw, GYRO_SCALE);
      this->pimpl_->data.gyro_y = this->Int16ToFloat(gy_raw, GYRO_SCALE);
      this->pimpl_->data.gyro_z = this->Int16ToFloat(gz_raw, GYRO_SCALE);
      this->pimpl_->data.gyro_valid = true;
      break;
    }

    case HWT9053Parser::CAN_ID_ANGLE:
    {
      // 欧拉角数据格式（协议: High_Precision_Sensor_CAN_Protocol.txt）
      // CAN 0x53 帧使用第1字节作为数据指示符:
      //   0x01: Roll 数据 - _data = [0x01, 0x00, LRollL, LRollH, HRollL,
      //   HRollH, ?, ?] 0x02: Pitch 数据 - _data = [0x02, 0x00, LPitchL,
      //   LPitchH, HPitchL, HPitchH, ?, ?] 0x03: Yaw 数据 - _data = [0x03,
      //   0x00, LYawL, LYawH, HYawL, HYawH, ?, ?]
      // 计算公式: 角度 = ((byte5<<24) | (byte4<<16) | (byte3<<8) | byte2) /
      // 1000.0 量程: ±180° (主要用于 9轴算法)

      constexpr float ANGLE_SCALE = 1.0f / 1000.0f * M_PI / 180.0f;  // rad

      uint8_t angle_type = _data[0];  // 获取数据指示符 (0x01/0x02/0x03)

      // 有效数据从 _data[2] 开始，_data[1] 是保留字节
      int32_t angle_raw =
          this->BytesToInt32(_data[2], _data[3], _data[4], _data[5]);

      if (angle_type == 0x01)
      {
        // Roll 数据
        this->pimpl_->data.roll = static_cast<float>(angle_raw) * ANGLE_SCALE;
      }
      else if (angle_type == 0x02)
      {
        // Pitch 数据
        this->pimpl_->data.pitch = static_cast<float>(angle_raw) * ANGLE_SCALE;
      }
      else if (angle_type == 0x03)
      {
        // Yaw 数据
        this->pimpl_->data.yaw = static_cast<float>(angle_raw) * ANGLE_SCALE;
      }

      this->pimpl_->data.angle_valid = true;
      break;
    }

    case HWT9053Parser::CAN_ID_MAGN:
    {
      // 磁场数据格式: [MxL, MxH, MyL, MyH, MzL, MzH, ?, ?]
      // 量程: ±400uT, 分辨率: 13nT/LSB = 0.013 μT/LSB (按 HWT9053 产品规格)
      // 正确计算: (400uT * 2) / 32768 = 0.0244 μT (2半量程)
      // 但根据产业规格，分辨率应为 0.013 μT/LSB
      constexpr float MAG_SCALE = 0.013f;  // uT/LSB

      // 低字节在 Data[0], 高字节在 Data[1]（按 CAN 协议）
      int16_t mx_raw = this->BytesToInt16(_data[1], _data[0]);
      int16_t my_raw = this->BytesToInt16(_data[3], _data[2]);
      int16_t mz_raw = this->BytesToInt16(_data[5], _data[4]);

      this->pimpl_->data.mag_x = this->Int16ToFloat(mx_raw, MAG_SCALE);
      this->pimpl_->data.mag_y = this->Int16ToFloat(my_raw, MAG_SCALE);
      this->pimpl_->data.mag_z = this->Int16ToFloat(mz_raw, MAG_SCALE);
      this->pimpl_->data.mag_valid = true;
      break;
    }

      // CAN ID 0x54 按协议应为磁场数据，温度数据更新不常（暂不处理）

    default:
      // 忽略不支持的 CAN ID
      break;
  }
}

sensor_msgs::msg::Imu HWT9053Parser::ToIMUMessage() const
{
  sensor_msgs::msg::Imu msg;

  // 线性加速度 (m/s^2)
  msg.linear_acceleration.x = this->pimpl_->data.accel_x;
  msg.linear_acceleration.y = this->pimpl_->data.accel_y;
  msg.linear_acceleration.z = this->pimpl_->data.accel_z;

  // 角速度 (rad/s)
  msg.angular_velocity.x = this->pimpl_->data.gyro_x;
  msg.angular_velocity.y = this->pimpl_->data.gyro_y;
  msg.angular_velocity.z = this->pimpl_->data.gyro_z;

  // 协方差矩阵（基于数据手册的典型精度设置）
  // 线性加速度协方差
  constexpr double ACCEL_COVARIANCE = 0.0001;  // (m/s^2)^2
  for (int i = 0; i < 9; ++i)
  {
    if (i % 4 == 0)
    {
      msg.linear_acceleration_covariance[i] = ACCEL_COVARIANCE;
    }
    else
    {
      msg.linear_acceleration_covariance[i] = 0.0;
    }
  }

  // 角速度协方差
  constexpr double GYRO_COVARIANCE = 0.00001;  // (rad/s)^2
  for (int i = 0; i < 9; ++i)
  {
    if (i % 4 == 0)
    {
      msg.angular_velocity_covariance[i] = GYRO_COVARIANCE;
    }
    else
    {
      msg.angular_velocity_covariance[i] = 0.0;
    }
  }

  // 方向四元数（从欧拉角转换）
  // 简化计算：使用Yaw作为主要方向
  float cy = std::cos(this->pimpl_->data.yaw * 0.5f);
  float sy = std::sin(this->pimpl_->data.yaw * 0.5f);
  float cp = std::cos(this->pimpl_->data.pitch * 0.5f);
  float sp = std::sin(this->pimpl_->data.pitch * 0.5f);
  float cr = std::cos(this->pimpl_->data.roll * 0.5f);
  float sr = std::sin(this->pimpl_->data.roll * 0.5f);

  msg.orientation.w = cr * cp * cy + sr * sp * sy;
  msg.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.orientation.z = cr * cp * sy - sr * sp * cy;

  return msg;
}

const HWT9053Data &HWT9053Parser::GetData() const
{
  return this->pimpl_->data;
}

void HWT9053Parser::Reset()
{
  this->pimpl_->data = HWT9053Data();
}

}  // namespace hwt9053_can_driver
