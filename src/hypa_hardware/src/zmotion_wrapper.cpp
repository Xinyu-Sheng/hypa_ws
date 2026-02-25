#include "hypa_hardware/zmotion_wrapper.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>

namespace hypa_hardware
{

ZMotionWrapper::ZMotionWrapper() : axis_configs_(32)
{
  // 初始化 32 个轴的配置缓存
}

ZMotionWrapper::~ZMotionWrapper() { disconnect(); }

std::optional<std::string> ZMotionWrapper::connect(const std::string& ip)
{
  if (handle_ != nullptr)
  {
    disconnect();
  }

  // 调用 ZMotion API 连接
  int32_t ret = ZMC_OpenEth(const_cast<char*>(ip.c_str()), &handle_);
  if (ret != ERR_OK || handle_ == nullptr)
  {
    last_error_ = "Failed to connect to ZMC432 at " + ip +
                  " (error code: " + std::to_string(ret) + ")";
    return last_error_;
  }

  // 设置默认超时时间（5秒）
  // ZMC_SetTimeout 如果需要的话

  return std::nullopt;  // 成功
}

void ZMotionWrapper::disconnect()
{
  if (handle_ != nullptr)
  {
    ZMC_Close(handle_);
    handle_ = nullptr;
  }
}

std::optional<std::string> ZMotionWrapper::set_units(int axis, double units)
{
  if (!ensure_axis_configured(axis))
  {
    return "Axis " + std::to_string(axis) + " not configured";
  }

  // 使用 ZAux_Direct_SetUnits 设置 units
  int32_t ret = ZAux_Direct_SetUnits(handle_, axis, static_cast<float>(units));
  if (ret != ERR_OK)
  {
    last_error_ = "Set units failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  axis_configs_[axis].units = units;
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::set_speed(int axis, double speed)
{
  int32_t ret = ZAux_Direct_SetSpeed(handle_, axis, static_cast<float>(speed));
  if (ret != ERR_OK)
  {
    last_error_ = "Set speed failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::set_acceleration(int axis,
                                                            double accel)
{
  int32_t ret = ZAux_Direct_SetAccel(handle_, axis, static_cast<float>(accel));
  if (ret != ERR_OK)
  {
    last_error_ = "Set acceleration failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::set_deceleration(int axis,
                                                            double decel)
{
  int32_t ret = ZAux_Direct_SetDecel(handle_, axis, static_cast<float>(decel));
  if (ret != ERR_OK)
  {
    last_error_ = "Set deceleration failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_absolute(int axis,
                                                         double position)
{
  if (!ensure_axis_configured(axis))
  {
    return "Axis " + std::to_string(axis) + " not configured";
  }

  int64_t pulses = physical_to_pulses(axis, position);
  int axis_list[1] = {axis};
  float distance_list[1] = {static_cast<float>(pulses)};
  int32_t ret = ZAux_Direct_MoveAbs(handle_, 1, axis_list, distance_list);
  if (ret != ERR_OK)
  {
    last_error_ = "Move absolute failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_relative(int axis,
                                                         double distance)
{
  if (!ensure_axis_configured(axis))
  {
    return "Axis " + std::to_string(axis) + " not configured";
  }

  int64_t pulses = physical_to_pulses(axis, distance);
  int axis_list[1] = {axis};
  float distance_list[1] = {static_cast<float>(pulses)};
  int32_t ret = ZAux_Direct_Move(handle_, 1, axis_list, distance_list);
  if (ret != ERR_OK)
  {
    last_error_ = "Move relative failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_velocity(int axis,
                                                         double velocity)
{
  if (!ensure_axis_configured(axis))
  {
    return "Axis " + std::to_string(axis) + " not configured";
  }

  // 使用连续运动模式 - ZAux_Direct_Single_Vmove
  // 注意：这个函数使用方向参数，正数为正向，负数为反向
  int64_t pulses_per_sec = physical_to_pulses(axis, std::abs(velocity));
  int direction = (velocity >= 0) ? 1 : -1;
  int32_t ret = ZAux_Direct_Single_Vmove(handle_, axis, direction);
  if (ret != ERR_OK)
  {
    last_error_ = "Move velocity failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  // 还需要设置速度参数
  ret = ZAux_Direct_SetSpeed(handle_, axis,
                             static_cast<float>(std::abs(velocity)));
  if (ret != ERR_OK)
  {
    last_error_ = "Set speed failed for velocity move on axis " +
                  std::to_string(axis) + " (error: " + std::to_string(ret) +
                  ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_line_absolute(
    const std::vector<int>& axes, const std::vector<double>& positions)
{
  if (axes.empty() || positions.size() != axes.size())
  {
    return "Invalid axes/positions size mismatch";
  }

  // 检查所有轴是否已配置
  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  // 构建命令字符串: "MOVEABS(axis0, pos0, axis1, pos1, ...)"
  std::stringstream cmd;
  cmd << "MOVEABS(";
  for (size_t i = 0; i < axes.size(); ++i)
  {
    if (i > 0) cmd << ",";
    int64_t pulses = physical_to_pulses(axes[i], positions[i]);
    cmd << axes[i] << "," << pulses;
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Move line absolute failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_line_relative(
    const std::vector<int>& axes, const std::vector<double>& distances)
{
  if (axes.empty() || distances.size() != axes.size())
  {
    return "Invalid axes/distances size mismatch";
  }

  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  std::stringstream cmd;
  cmd << "MOVE(";
  for (size_t i = 0; i < axes.size(); ++i)
  {
    if (i > 0) cmd << ",";
    int64_t pulses = physical_to_pulses(axes[i], distances[i]);
    cmd << axes[i] << "," << pulses;
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Move line relative failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_circular_absolute(
    const std::vector<int>& axes, const std::vector<double>& positions,
    const std::vector<double>& circular_params)
{
  if (axes.size() < 2 || axes.size() > 3)
  {
    return "Circular interpolation requires 2 or 3 axes";
  }
  if (positions.size() != axes.size())
  {
    return "Positions size mismatch with axes";
  }

  // 检查所有轴已配置
  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  // 构建命令: MOVECIRCABS(axes..., positions..., circular_params...)
  std::stringstream cmd;
  cmd << "MOVECIRCABS(";
  for (size_t i = 0; i < axes.size(); ++i)
  {
    if (i > 0) cmd << ",";
    int64_t pulses = physical_to_pulses(axes[i], positions[i]);
    cmd << axes[i] << "," << pulses;
  }
  // 添加圆心参数（已为物理单位）
  for (size_t i = 0; i < circular_params.size(); ++i)
  {
    if (i < axes.size())
      cmd << ",";  // 继续用逗号分隔
    else
      cmd << ",";
    // 圆心坐标不需要转换为脉冲（已经是物理单位，但 ZMotion 可能也需要转换）
    // 假设圆心也是物理单位，需要转换为脉冲
    if (i < circular_params.size())
    {
      // 圆心参数对应前几个轴
      int axis_idx = i % axes.size();
      double param_val = circular_params[i];
      int64_t pulses = physical_to_pulses(axes[axis_idx], param_val);
      cmd << pulses;
    }
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Move circular absolute failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_spiral_absolute(
    const std::vector<int>& axes, const std::vector<double>& positions,
    const std::vector<double>& spiral_params)
{
  if (axes.size() != 3)
  {
    return "Spiral interpolation requires exactly 3 axes";
  }
  if (positions.size() != 3)
  {
    return "Spiral positions must have 3 values";
  }
  if (spiral_params.size() < 4)
  {
    return "Spiral requires at least 4 parameters: [radius, pitch, turns, "
           "end_angle]";
  }

  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  // 命令格式: MOVESPIRALABS(axis0, axis1, axis2, x, y, z, radius, pitch, turns,
  // end_angle)
  std::stringstream cmd;
  cmd << "MOVESPIRALABS(";
  for (size_t i = 0; i < axes.size(); ++i)
  {
    if (i > 0) cmd << ",";
    cmd << axes[i];
  }
  for (size_t i = 0; i < positions.size(); ++i)
  {
    cmd << ",";
    int64_t pulses = physical_to_pulses(axes[i], positions[i]);
    cmd << pulses;
  }
  // 螺旋参数: radius, pitch, turns, end_angle
  // radius 和 pitch 需要转换为脉冲单位
  for (size_t i = 0; i < spiral_params.size(); ++i)
  {
    cmd << ",";
    if (i < 2)
    {  // radius, pitch 需要单位转换
      int64_t pulses =
          physical_to_pulses(axes[0], spiral_params[i]);  // 使用第一个轴的单位
      cmd << pulses;
    }
    else
    {
      // turns 和 end_angle 是纯数值，不转换
      cmd << spiral_params[i];
    }
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Move spiral absolute failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_eclipse_absolute(
    const std::vector<int>& axes, const std::vector<double>& positions,
    const std::vector<double>& eclipse_params)
{
  if (axes.size() != 2)
  {
    return "Eclipse interpolation requires exactly 2 axes";
  }
  if (positions.size() != 2)
  {
    return "Eclipse positions must have 2 values";
  }
  if (eclipse_params.size() < 6)
  {
    return "Eclipse requires 6 parameters: [center_x, center_y, major_axis, "
           "minor_axis, start_angle, end_angle]";
  }

  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  // 命令格式: MOVECLIPSEABS(axis0, axis1, x, y, center_x, center_y, major_axis,
  // minor_axis, start_angle, end_angle)
  std::stringstream cmd;
  cmd << "MOVECLIPSEABS(";
  cmd << axes[0] << "," << axes[1] << ",";
  int64_t pulses_x = physical_to_pulses(axes[0], positions[0]);
  int64_t pulses_y = physical_to_pulses(axes[1], positions[1]);
  cmd << pulses_x << "," << pulses_y << ",";

  // 椭圆参数: center_x, center_y, major_axis, minor_axis, start_angle,
  // end_angle 位置参数需要转换为脉冲
  for (size_t i = 0; i < eclipse_params.size(); ++i)
  {
    if (i > 0) cmd << ",";
    if (i < 4)
    {  // center_x, center_y, major_axis, minor_axis 需要转换
      int axis_idx =
          (i < 2) ? i
                  : 0;  // center_x->axis0, center_y->axis1, major/minor->axis0
      int64_t pulses = physical_to_pulses(axes[axis_idx], eclipse_params[i]);
      cmd << pulses;
    }
    else
    {
      // 角度参数不转换
      cmd << eclipse_params[i];
    }
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Move eclipse absolute failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::move_spherical_absolute(
    const std::vector<int>& axes, const std::vector<double>& positions,
    const std::vector<double>& spherical_params)
{
  if (axes.size() != 3)
  {
    return "Spherical interpolation requires exactly 3 axes";
  }
  if (positions.size() != 3)
  {
    return "Spherical positions must have 3 values";
  }
  if (spherical_params.size() < 8)
  {
    return "Spherical requires 8 parameters: [center_x, center_y, center_z, "
           "radius, start_theta, end_theta, start_phi, end_phi]";
  }

  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  // 命令格式: MOVESPHERICALABS(axis0, axis1, axis2, x, y, z, center_x,
  // center_y, center_z, radius, start_theta, end_theta, start_phi, end_phi)
  std::stringstream cmd;
  cmd << "MOVESPHERICALABS(";
  for (size_t i = 0; i < axes.size(); ++i)
  {
    if (i > 0) cmd << ",";
    cmd << axes[i];
  }
  for (size_t i = 0; i < positions.size(); ++i)
  {
    cmd << ",";
    int64_t pulses = physical_to_pulses(axes[i], positions[i]);
    cmd << pulses;
  }
  // 空间圆弧参数: center_x, center_y, center_z, radius, start_theta, end_theta,
  // start_phi, end_phi
  for (size_t i = 0; i < spherical_params.size(); ++i)
  {
    cmd << ",";
    if (i < 4)
    {  // center_x, center_y, center_z, radius 需要转换
      int axis_idx = (i < 3) ? i : 0;  // radius 使用 axis0 的单位
      int64_t pulses = physical_to_pulses(axes[axis_idx], spherical_params[i]);
      cmd << pulses;
    }
    else
    {
      // 角度参数不转换
      cmd << spherical_params[i];
    }
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Move spherical absolute failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::buffer_move(
    const std::vector<int>& axes, const std::vector<double>& positions)
{
  if (axes.empty() || positions.size() != axes.size())
  {
    return "Invalid axes/positions size mismatch";
  }

  for (int axis : axes)
  {
    if (!ensure_axis_configured(axis))
    {
      return "Axis " + std::to_string(axis) + " not configured";
    }
  }

  // 使用 BUFFERMOVE 命令
  std::stringstream cmd;
  cmd << "BUFFERMOVE(";
  for (size_t i = 0; i < axes.size(); ++i)
  {
    if (i > 0) cmd << ",";
    int64_t pulses = physical_to_pulses(axes[i], positions[i]);
    cmd << axes[i] << "," << pulses;
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle_, const_cast<char*>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ = "Buffer move failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::start_continuous()
{
  // 使用 CONTINUE 命令启动连续插补模式
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle_, "CONTINUE", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ =
        "Start continuous failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::stop_continuous()
{
  // 停止连续插补模式
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle_, "STOP", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ = "Stop continuous failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }

  return std::nullopt;
}

std::optional<double> ZMotionWrapper::get_position(int axis)
{
  float dpos = 0.0f;
  int32_t ret = ZAux_Direct_GetDpos(handle_, axis, &dpos);
  if (ret != ERR_OK)
  {
    last_error_ = "Get position failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<double>(dpos);  // DPOS 已经是物理单位
}

std::optional<double> ZMotionWrapper::get_feedback(int axis)
{
  float mpos = 0.0f;
  int32_t ret = ZAux_Direct_GetMpos(handle_, axis, &mpos);
  if (ret != ERR_OK)
  {
    last_error_ = "Get feedback failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<double>(mpos);  // MPOS 已经是物理单位
}

std::optional<double> ZMotionWrapper::get_speed(int axis)
{
  float mspeed = 0.0f;
  int32_t ret = ZAux_Direct_GetMspeed(handle_, axis, &mspeed);
  if (ret != ERR_OK)
  {
    last_error_ = "Get speed failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<double>(mspeed);  // MSpeed 已经是物理单位
}

std::optional<uint32_t> ZMotionWrapper::get_axis_status(int axis)
{
  int32_t status = 0;  // ZAux_Direct_GetAxisStatus 使用 int32_t 指针
  int32_t ret = ZAux_Direct_GetAxisStatus(handle_, axis, &status);
  if (ret != ERR_OK)
  {
    last_error_ = "Get axis status failed for axis " + std::to_string(axis) +
                  " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<uint32_t>(status);
}

bool ZMotionWrapper::is_axis_moving(int axis)
{
  auto status_opt = get_axis_status(axis);
  if (!status_opt) return false;

  // 根据 zmotion.h 中的 AXISSTATUS 位定义判断
  // 假设 BIT0 表示运动状态（需要查阅 zmotion.h 确认）
  // 这里使用常见的位定义：BIT0=IDLE, BIT1=MOVING, BIT2=ERROR 等
  uint32_t status = status_opt.value();
  // 如果 BIT1 为 1 表示正在运动
  return (status & 0x00000002) != 0;
}

std::optional<std::string> ZMotionWrapper::stop_all()
{
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle_, "STOP ALL", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ = "Stop all failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::emergency_stop()
{
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle_, "EMERGENCY STOP", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error_ = "Emergency stop failed (error: " + std::to_string(ret) + ")";
    return last_error_;
  }
  return std::nullopt;
}

// Private methods

int64_t ZMotionWrapper::physical_to_pulses(int axis, double physical_position)
{
  double units = get_current_units(axis);
  return static_cast<int64_t>(physical_position / units);
}

double ZMotionWrapper::pulses_to_physical(int axis, int64_t pulses)
{
  double units = get_current_units(axis);
  return pulses * units;
}

std::optional<std::string> ZMotionWrapper::ensure_axis_configured(int axis)
{
  if (axis < 0 || axis >= 32)
  {
    return "Axis number out of range (0-31)";
  }

  // 如果已经配置过，直接返回成功
  if (axis_configs_[axis].configured)
  {
    return std::nullopt;
  }

  // 尝试读取当前 units 值
  // 使用 ZAux_Direct_GetUnits 查询
  float units_val = 0.0f;
  int32_t ret = ZAux_Direct_GetUnits(handle_, axis, &units_val);
  if (ret != ERR_OK)
  {
    // 如果查询失败，设置默认值 1.0
    axis_configs_[axis].units = 1.0;
    axis_configs_[axis].configured = true;
    // 不返回错误，允许继续
    return std::nullopt;
  }

  axis_configs_[axis].units = static_cast<double>(units_val);
  axis_configs_[axis].configured = true;
  return std::nullopt;
}

double ZMotionWrapper::get_current_units(int axis)
{
  if (axis >= 0 && axis < 32 && axis_configs_[axis].configured)
  {
    return axis_configs_[axis].units;
  }
  return 1.0;  // 默认值
}

}  // namespace hypa_hardware
