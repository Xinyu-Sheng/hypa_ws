#include "hwt9053_can_driver/hwt9053_parser.hpp"

#include <chrono>
#include <cmath>

namespace hwt9053_can_driver
{

// PIMPL 实现
class HWT9053Parser::Impl
{
  public:
  HWT9053Data data;
  mutable std::mutex data_mutex;  // 保护数据访问的互斥锁

  // 欧拉角缓冲状态
  bool roll_ready{false};
  bool pitch_ready{false};
  bool yaw_ready{false};
  std::chrono::steady_clock::time_point last_angle_time;  // 用于老化机制

  // 协方差值
  double accel_covariance{3.4e-5};  // (m/s^2)^2
  double gyro_covariance{5.8e-8};   // (rad/s)^2

  // 重置缓冲状态
  void ResetAngleBuffer()
  {
    this->roll_ready = false;
    this->pitch_ready = false;
    this->yaw_ready = false;
  }

  // 检查三个角度是否都已准备好
  bool IsAngleDataComplete() const
  {
    return this->roll_ready && this->pitch_ready && this->yaw_ready;
  }

  // 获取数据（线程安全）
  HWT9053Data GetDataSnapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex);
    return data;
  }

  // 设置数据（线程安全）
  void SetData(const HWT9053Data &_data)
  {
    std::lock_guard<std::mutex> lock(data_mutex);
    data = _data;
  }
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

bool HWT9053Parser::ParseCANFrame(uint32_t _can_id,
                                  const std::array<uint8_t, 8> &_data,
                                  uint8_t _dlc)
{
  if (_dlc != 8)
  {
    return false;  // DLC 无效，解析失败
  }
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);

  // 支持两种封装：
  // 1) 传统 HWT9053 CAN（CAN ID 表示类型，数据直接在 payload[0..7]）
  // 2) WIT SDK 风格：payload[0] == 0x55，payload[1] 为类型（WIT_*），有效数据位于 payload[2..7]
  bool is_wit_payload = (_data[0] == 0x55);
  uint32_t msg_type = _can_id;

  if (is_wit_payload)
  {
    msg_type = static_cast<uint32_t>(_data[1]);
  }

  switch (msg_type)
  {
    case HWT9053Parser::CAN_ID_TIME:
    {
      if (is_wit_payload)
      {
        // WIT 常见封装：payload[2..7] 存放时间或寄存器压缩值
        this->pimpl_->data.year = _data[2];
        this->pimpl_->data.month = _data[3];
        this->pimpl_->data.day = _data[4];
        this->pimpl_->data.hour = _data[5];
        this->pimpl_->data.minute = _data[6];
        this->pimpl_->data.second = _data[7];
        this->pimpl_->data.millisecond = 0;
      }
      else
      {
        // 原始 HWT9053 时间格式: [YY, MM, DD, HH, MN, SS, msl, msh]
        this->pimpl_->data.year = _data[0];
        this->pimpl_->data.month = _data[1];
        this->pimpl_->data.day = _data[2];
        this->pimpl_->data.hour = _data[3];
        this->pimpl_->data.minute = _data[4];
        this->pimpl_->data.second = _data[5];
        this->pimpl_->data.millisecond = this->BytesToUInt16(_data[7], _data[6]);
      }
      break;
    }

    case HWT9053Parser::CAN_ID_ACCEL:
    {
      int16_t ax_raw, ay_raw, az_raw;
      if (is_wit_payload)
      {
        // WIT CAN: data in payload[2..7] as three 16-bit values (low,high order)
        ax_raw = this->BytesToInt16(_data[3], _data[2]);
        ay_raw = this->BytesToInt16(_data[5], _data[4]);
        az_raw = this->BytesToInt16(_data[7], _data[6]);
      }
      else
      {
        // 原始 HWT9053: [AxL, AxH, AyL, AyH, AzL, AzH, ?, ?]
        ax_raw = this->BytesToInt16(_data[1], _data[0]);
        ay_raw = this->BytesToInt16(_data[3], _data[2]);
        az_raw = this->BytesToInt16(_data[5], _data[4]);
      }

      this->pimpl_->data.accel_x = this->Int16ToFloat(ax_raw, HWT9053Parser::ACCEL_SCALE);
      this->pimpl_->data.accel_y = this->Int16ToFloat(ay_raw, HWT9053Parser::ACCEL_SCALE);
      this->pimpl_->data.accel_z = this->Int16ToFloat(az_raw, HWT9053Parser::ACCEL_SCALE);
      this->pimpl_->data.accel_valid = true;
      break;
    }

    case HWT9053Parser::CAN_ID_GYRO:
    {
      int16_t gx_raw, gy_raw, gz_raw;
      if (is_wit_payload)
      {
        gx_raw = this->BytesToInt16(_data[3], _data[2]);
        gy_raw = this->BytesToInt16(_data[5], _data[4]);
        gz_raw = this->BytesToInt16(_data[7], _data[6]);
      }
      else
      {
        gx_raw = this->BytesToInt16(_data[1], _data[0]);
        gy_raw = this->BytesToInt16(_data[3], _data[2]);
        gz_raw = this->BytesToInt16(_data[5], _data[4]);
      }

      this->pimpl_->data.gyro_x = this->Int16ToFloat(gx_raw, HWT9053Parser::GYRO_SCALE);
      this->pimpl_->data.gyro_y = this->Int16ToFloat(gy_raw, HWT9053Parser::GYRO_SCALE);
      this->pimpl_->data.gyro_z = this->Int16ToFloat(gz_raw, HWT9053Parser::GYRO_SCALE);
      this->pimpl_->data.gyro_valid = true;
      break;
    }

    case HWT9053Parser::CAN_ID_ANGLE:
    {
      // 支持两种常见角度封装：
      // - 原始 HWT9053: data[0]=angle_type, data[2..5] 为 32-bit 角度 (低字节优先)
      // - WIT 905x_CAN: payload[0]=0x55, payload[1]=0x53, payload[2]=angle_type, payload[4..7] 为 32-bit 角度
      // 老化机制
      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->pimpl_->last_angle_time);
      if (duration.count() > 50)
      {
        this->pimpl_->ResetAngleBuffer();
      }
      this->pimpl_->last_angle_time = now;

      if (is_wit_payload)
      {
        uint8_t angle_type = _data[2];
        if (angle_type == 0x01 || angle_type == 0x02 || angle_type == 0x03)
        {
          int32_t angle_raw = this->BytesToInt32(_data[4], _data[5], _data[6], _data[7]);
          if (angle_type == 0x01)
          {
            this->pimpl_->data.roll = static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
            this->pimpl_->roll_ready = true;
          }
          else if (angle_type == 0x02)
          {
            this->pimpl_->data.pitch = static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
            this->pimpl_->pitch_ready = true;
          }
          else if (angle_type == 0x03)
          {
            this->pimpl_->data.yaw = static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
            this->pimpl_->yaw_ready = true;
          }

          if (this->pimpl_->IsAngleDataComplete())
          {
            this->pimpl_->data.angle_valid = true;
            this->pimpl_->ResetAngleBuffer();
          }
        }
        else
        {
          return false;
        }
      }
      else
      {
        uint8_t angle_type = _data[0];
        int32_t angle_raw = this->BytesToInt32(_data[2], _data[3], _data[4], _data[5]);
        if (angle_type == 0x01)
        {
          this->pimpl_->data.roll = static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
          this->pimpl_->roll_ready = true;
        }
        else if (angle_type == 0x02)
        {
          this->pimpl_->data.pitch = static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
          this->pimpl_->pitch_ready = true;
        }
        else if (angle_type == 0x03)
        {
          this->pimpl_->data.yaw = static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
          this->pimpl_->yaw_ready = true;
        }
        else
        {
          return false;
        }

        if (this->pimpl_->IsAngleDataComplete())
        {
          this->pimpl_->data.angle_valid = true;
          this->pimpl_->ResetAngleBuffer();
        }
      }
      break;
    }

    case HWT9053Parser::CAN_ID_MAGN:
    {
      int16_t mx_raw, my_raw, mz_raw;
      if (is_wit_payload)
      {
        mx_raw = this->BytesToInt16(_data[3], _data[2]);
        my_raw = this->BytesToInt16(_data[5], _data[4]);
        mz_raw = this->BytesToInt16(_data[7], _data[6]);
      }
      else
      {
        mx_raw = this->BytesToInt16(_data[1], _data[0]);
        my_raw = this->BytesToInt16(_data[3], _data[2]);
        mz_raw = this->BytesToInt16(_data[5], _data[4]);
      }

      this->pimpl_->data.mag_x = this->Int16ToFloat(mx_raw, HWT9053Parser::MAG_SCALE);
      this->pimpl_->data.mag_y = this->Int16ToFloat(my_raw, HWT9053Parser::MAG_SCALE);
      this->pimpl_->data.mag_z = this->Int16ToFloat(mz_raw, HWT9053Parser::MAG_SCALE);
      this->pimpl_->data.mag_valid = true;
      break;
    }

    default:
      // 不支持的消息类型（既不是 HWT9053 的 CAN ID，也不是 Wit payload 的 type）
      return false;
  }

  return true;
}

sensor_msgs::msg::Imu HWT9053Parser::ToIMUMessage() const
{
  // 获取数据快照（线程安全）
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  HWT9053Data data_snapshot = this->pimpl_->data;
  // 锁在这里自动释放

  sensor_msgs::msg::Imu msg;

  // 线性加速度 (m/s^2)
  msg.linear_acceleration.x = data_snapshot.accel_x;
  msg.linear_acceleration.y = data_snapshot.accel_y;
  msg.linear_acceleration.z = data_snapshot.accel_z;

  // 角速度 (rad/s)
  msg.angular_velocity.x = data_snapshot.gyro_x;
  msg.angular_velocity.y = data_snapshot.gyro_y;
  msg.angular_velocity.z = data_snapshot.gyro_z;

  // 协方差矩阵（基于HWT9053硬件规格）
  // 线性加速度协方差：规格值~3.4e-5 (m/s^2)^2
  double accel_covariance = this->pimpl_->accel_covariance;
  for (int i = 0; i < 9; ++i)
  {
    if (i % 4 == 0)
    {
      msg.linear_acceleration_covariance[i] = accel_covariance;
    }
    else
    {
      msg.linear_acceleration_covariance[i] = 0.0;
    }
  }

  // 角速度协方差：规格值~5.8e-8 (rad/s)^2
  double gyro_covariance = this->pimpl_->gyro_covariance;
  for (int i = 0; i < 9; ++i)
  {
    if (i % 4 == 0)
    {
      msg.angular_velocity_covariance[i] = gyro_covariance;
    }
    else
    {
      msg.angular_velocity_covariance[i] = 0.0;
    }
  }

  if (data_snapshot.angle_valid)
  {
    float cy = std::cos(data_snapshot.yaw * 0.5f);
    float sy = std::sin(data_snapshot.yaw * 0.5f);
    float cp = std::cos(data_snapshot.pitch * 0.5f);
    float sp = std::sin(data_snapshot.pitch * 0.5f);
    float cr = std::cos(data_snapshot.roll * 0.5f);
    float sr = std::sin(data_snapshot.roll * 0.5f);

    msg.orientation.w = cr * cp * cy + sr * sp * sy;
    msg.orientation.x = sr * cp * cy - cr * sp * sy;
    msg.orientation.y = cr * sp * cy + sr * cp * sy;
    msg.orientation.z = cr * cp * sy - sr * sp * cy;
  }
  else
  {
    // 未获取到完整角度数据时显式声明 orientation 不可用。
    msg.orientation_covariance[0] = -1.0;
  }

  return msg;
}

HWT9053Data HWT9053Parser::GetData() const
{
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  return this->pimpl_->data;
}

HWT9053Parser::MagneticFieldData HWT9053Parser::GetMagneticFieldData() const
{
  // 获取磁场数据快照（线程安全）
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  return {this->pimpl_->data.mag_x, this->pimpl_->data.mag_y,
          this->pimpl_->data.mag_z, this->pimpl_->data.mag_valid};
}

void HWT9053Parser::Reset()
{
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  this->pimpl_->data = HWT9053Data();
  this->pimpl_->ResetAngleBuffer();
}

void HWT9053Parser::SetAccelCovariance(double _covariance)
{
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  this->pimpl_->accel_covariance = _covariance;
}

void HWT9053Parser::SetGyroCovariance(double _covariance)
{
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  this->pimpl_->gyro_covariance = _covariance;
}

}  // namespace hwt9053_can_driver
