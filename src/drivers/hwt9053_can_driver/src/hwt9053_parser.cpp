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
  std::chrono::steady_clock::time_point last_angle_time{};  // 用于老化机制

  // 各物理量最后接收时间，用于过期判断
  std::chrono::steady_clock::time_point last_accel_time{};
  std::chrono::steady_clock::time_point last_gyro_time{};
  std::chrono::steady_clock::time_point last_mag_time{};

  // 数据过期阈值（毫秒）
  uint32_t data_timeout_ms{500};

  // 协方差值（语义：方差）
  double accel_covariance{1.18e-9};  // (m/s^2)^2
  double gyro_covariance{2.39e-7};   // (rad/s)^2

  // 重置缓冲状态并清除 angle_valid
  void ResetAngleBuffer()
  {
    this->roll_ready = false;
    this->pitch_ready = false;
    this->yaw_ready = false;
    this->data.angle_valid = false;
    this->last_angle_time = std::chrono::steady_clock::time_point{};
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
  uint16_t u =
      (static_cast<uint16_t>(_high) << 8) | static_cast<uint16_t>(_low);
  return static_cast<int16_t>(u);
}

uint16_t HWT9053Parser::BytesToUInt16(uint8_t _high, uint8_t _low) const
{
  uint16_t value =
      (static_cast<uint16_t>(_high) << 8) | static_cast<uint16_t>(_low);
  return value;
}

int32_t HWT9053Parser::BytesToInt32(uint8_t _byte0, uint8_t _byte1,
                                    uint8_t _byte2, uint8_t _byte3) const
{
  // 按协议格式组合32位有符号整数：byte0为低位，byte3为高位
  // 先使用无符号运算避免有符号左移的未定义行为
  uint32_t u = (static_cast<uint32_t>(_byte3) << 24) |
               (static_cast<uint32_t>(_byte2) << 16) |
               (static_cast<uint32_t>(_byte1) << 8) |
               static_cast<uint32_t>(_byte0);
  return static_cast<int32_t>(u);
}

float HWT9053Parser::Int16ToFloat(int16_t _value, float _scale) const
{
  return _value * _scale;
}

bool HWT9053Parser::ParseCANFrame(const std::array<uint8_t, 8> &_data,
                                  uint8_t _dlc)
{
  if (_dlc != 8)
  {
    return false;  // DLC 无效，解析失败
  }
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);

  const auto now = std::chrono::steady_clock::now();

  // 严格按照 WIT 封装解析：payload[0] == 0x55, payload[1] == TYPE
  if (_data[0] != 0x55)
  {
    return false;  // 非 WIT 封装，拒绝解析
  }

  uint8_t type = _data[1];

  switch (type)
  {
    case HWT9053Parser::WIT_TYPE_TIME:
    {
      // WIT 时间输出：payload[2..7] = [YY, MM, DD, HH, MN, SS]
      this->pimpl_->data.year = _data[2];
      this->pimpl_->data.month = _data[3];
      this->pimpl_->data.day = _data[4];
      this->pimpl_->data.hour = _data[5];
      this->pimpl_->data.minute = _data[6];
      this->pimpl_->data.second = _data[7];
      this->pimpl_->data.millisecond = 0;
      break;
    }

    case HWT9053Parser::WIT_TYPE_ACCEL:
    {
      // WIT 加速度：payload[2..7] = AxL,AxH,AyL,AyH,AzL,AzH
      int16_t ax_raw = this->BytesToInt16(_data[3], _data[2]);
      int16_t ay_raw = this->BytesToInt16(_data[5], _data[4]);
      int16_t az_raw = this->BytesToInt16(_data[7], _data[6]);

      this->pimpl_->data.accel_x =
          this->Int16ToFloat(ax_raw, HWT9053Parser::ACCEL_SCALE);
      this->pimpl_->data.accel_y =
          this->Int16ToFloat(ay_raw, HWT9053Parser::ACCEL_SCALE);
      this->pimpl_->data.accel_z =
          this->Int16ToFloat(az_raw, HWT9053Parser::ACCEL_SCALE);
      this->pimpl_->data.accel_valid = true;
      this->pimpl_->last_accel_time = now;
      break;
    }

    case HWT9053Parser::WIT_TYPE_GYRO:
    {
      // WIT 角速度：payload[2..7] = GxL,GxH,GyL,GyH,GzL,GzH
      int16_t gx_raw = this->BytesToInt16(_data[3], _data[2]);
      int16_t gy_raw = this->BytesToInt16(_data[5], _data[4]);
      int16_t gz_raw = this->BytesToInt16(_data[7], _data[6]);

      this->pimpl_->data.gyro_x =
          this->Int16ToFloat(gx_raw, HWT9053Parser::GYRO_SCALE);
      this->pimpl_->data.gyro_y =
          this->Int16ToFloat(gy_raw, HWT9053Parser::GYRO_SCALE);
      this->pimpl_->data.gyro_z =
          this->Int16ToFloat(gz_raw, HWT9053Parser::GYRO_SCALE);
      this->pimpl_->data.gyro_valid = true;
      this->pimpl_->last_gyro_time = now;
      break;
    }

    case HWT9053Parser::WIT_TYPE_ANGLE:
    {
      // WIT 角度：payload[2] = angle_type(0x01/0x02/0x03), payload[4..7] 为
      // 32-bit angle
      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - this->pimpl_->last_angle_time);
      if (duration.count() > 50)
      {
        this->pimpl_->ResetAngleBuffer();
      }
      this->pimpl_->last_angle_time = now;

      uint8_t angle_type = _data[2];
      if (angle_type != 0x01 && angle_type != 0x02 && angle_type != 0x03)
      {
        return false;
      }

      int32_t angle_raw =
          this->BytesToInt32(_data[4], _data[5], _data[6], _data[7]);
      if (angle_type == 0x01)
      {
        this->pimpl_->data.roll =
            static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
        this->pimpl_->roll_ready = true;
      }
      else if (angle_type == 0x02)
      {
        this->pimpl_->data.pitch =
            static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
        this->pimpl_->pitch_ready = true;
      }
      else if (angle_type == 0x03)
      {
        this->pimpl_->data.yaw =
            static_cast<float>(angle_raw) * HWT9053Parser::ANGLE_SCALE;
        this->pimpl_->yaw_ready = true;
      }

      if (this->pimpl_->IsAngleDataComplete())
      {
        this->pimpl_->data.angle_valid = true;
        this->pimpl_->ResetAngleBuffer();
      }
      break;
    }

    case HWT9053Parser::WIT_TYPE_MAGN:
    {
      // WIT 磁场：payload[2..7] = HxL,HxH,HyL,HyH,HzL,HzH
      int16_t mx_raw = this->BytesToInt16(_data[3], _data[2]);
      int16_t my_raw = this->BytesToInt16(_data[5], _data[4]);
      int16_t mz_raw = this->BytesToInt16(_data[7], _data[6]);

      this->pimpl_->data.mag_x =
          this->Int16ToFloat(mx_raw, HWT9053Parser::MAG_SCALE);
      this->pimpl_->data.mag_y =
          this->Int16ToFloat(my_raw, HWT9053Parser::MAG_SCALE);
      this->pimpl_->data.mag_z =
          this->Int16ToFloat(mz_raw, HWT9053Parser::MAG_SCALE);
      this->pimpl_->data.mag_valid = true;
      this->pimpl_->last_mag_time = now;
      break;
    }

    default:
      return false;
  }

  return true;
}

sensor_msgs::msg::Imu HWT9053Parser::ToIMUMessage() const
{
  // 仅在临界区内复制需要的数据，缩小锁范围
  HWT9053Data data_snapshot;
  double accel_covariance = 0.0;
  double gyro_covariance = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
    data_snapshot = this->pimpl_->data;
    accel_covariance = this->pimpl_->accel_covariance;
    gyro_covariance = this->pimpl_->gyro_covariance;
  }

  sensor_msgs::msg::Imu msg;

  // 线性加速度 (m/s^2)
  msg.linear_acceleration.x = data_snapshot.accel_x;
  msg.linear_acceleration.y = data_snapshot.accel_y;
  msg.linear_acceleration.z = data_snapshot.accel_z;

  // 角速度 (rad/s)
  msg.angular_velocity.x = data_snapshot.gyro_x;
  msg.angular_velocity.y = data_snapshot.gyro_y;
  msg.angular_velocity.z = data_snapshot.gyro_z;

  // 协方差矩阵：注意这里语义为方差 (已经由节点/配置统一为方差)
  for (int i = 0; i < 9; ++i)
  {
    msg.linear_acceleration_covariance[i] =
        (i % 4 == 0) ? accel_covariance : 0.0;
    msg.angular_velocity_covariance[i] = (i % 4 == 0) ? gyro_covariance : 0.0;
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

    // 归一化四元数以避免数值漂移
    double norm = std::sqrt(msg.orientation.w * msg.orientation.w +
                            msg.orientation.x * msg.orientation.x +
                            msg.orientation.y * msg.orientation.y +
                            msg.orientation.z * msg.orientation.z);
    if (norm > 1e-12)
    {
      msg.orientation.w /= norm;
      msg.orientation.x /= norm;
      msg.orientation.y /= norm;
      msg.orientation.z /= norm;
    }
    else
    {
      // 归一化失败 -> 标记 orientation 不可用
      msg.orientation_covariance[0] = -1.0;
      msg.orientation.w = 0.0;
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
    }
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
  HWT9053Data snapshot = this->pimpl_->data;
  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(this->pimpl_->data_timeout_ms);

  // 加速度过期检测
  if (snapshot.accel_valid)
  {
    if (this->pimpl_->last_accel_time.time_since_epoch().count() == 0 ||
        (now - this->pimpl_->last_accel_time) > timeout)
    {
      snapshot.accel_valid = false;
    }
  }

  // 角速度过期检测
  if (snapshot.gyro_valid)
  {
    if (this->pimpl_->last_gyro_time.time_since_epoch().count() == 0 ||
        (now - this->pimpl_->last_gyro_time) > timeout)
    {
      snapshot.gyro_valid = false;
    }
  }

  // 角度过期检测
  if (snapshot.angle_valid)
  {
    if (this->pimpl_->last_angle_time.time_since_epoch().count() == 0 ||
        (now - this->pimpl_->last_angle_time) > timeout)
    {
      snapshot.angle_valid = false;
    }
  }

  // 磁场过期检测
  if (snapshot.mag_valid)
  {
    if (this->pimpl_->last_mag_time.time_since_epoch().count() == 0 ||
        (now - this->pimpl_->last_mag_time) > timeout)
    {
      snapshot.mag_valid = false;
    }
  }

  return snapshot;
}

HWT9053Parser::MagneticFieldData HWT9053Parser::GetMagneticFieldData() const
{
  // 获取磁场数据快照（线程安全）
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  MagneticFieldData snapshot = {
      this->pimpl_->data.mag_x, this->pimpl_->data.mag_y,
      this->pimpl_->data.mag_z, this->pimpl_->data.mag_valid};
  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(this->pimpl_->data_timeout_ms);
  if (snapshot.mag_valid)
  {
    if (this->pimpl_->last_mag_time.time_since_epoch().count() == 0 ||
        (now - this->pimpl_->last_mag_time) > timeout)
    {
      snapshot.mag_valid = false;
    }
  }
  return snapshot;
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

void HWT9053Parser::SetDataTimeoutMs(uint32_t _ms)
{
  std::lock_guard<std::mutex> lock(this->pimpl_->data_mutex);
  this->pimpl_->data_timeout_ms = _ms;
}

}  // namespace hwt9053_can_driver
