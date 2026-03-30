#include "zmc432_driver/zmotion_wrapper.hpp"

#include <iostream>
#include <sstream>

namespace zmc432_driver
{

// Public interface implementations
ZMotionWrapper::ZMotionWrapper()
    : pimpl_(std::make_unique<ZMotionWrapperPrivate>())
{
}

ZMotionWrapper::~ZMotionWrapper()
{
  disconnect();
}

ZMotionWrapper::ZMotionWrapperPrivate::ZMotionWrapperPrivate()
    : axis_configs(32)
{
}

ZMotionWrapper::ZMotionWrapperPrivate::~ZMotionWrapperPrivate() = default;

std::optional<std::string> ZMotionWrapper::connect(const std::string &_ip)
{
  return pimpl_->connect(_ip);
}

void ZMotionWrapper::disconnect()
{
  pimpl_->disconnect();
}

bool ZMotionWrapper::is_connected() const
{
  return pimpl_->is_connected();
}

std::optional<std::string> ZMotionWrapper::set_units(int _axis, double _units)
{
  return pimpl_->set_units(_axis, _units);
}

std::optional<std::string> ZMotionWrapper::set_speed(int _axis, double _speed)
{
  return pimpl_->set_speed(_axis, _speed);
}

std::optional<std::string> ZMotionWrapper::set_acceleration(int _axis,
                                                            double _accel)
{
  return pimpl_->set_acceleration(_axis, _accel);
}

std::optional<std::string> ZMotionWrapper::set_deceleration(int _axis,
                                                            double _decel)
{
  return pimpl_->set_deceleration(_axis, _decel);
}

std::optional<std::string> ZMotionWrapper::move_absolute(int _axis,
                                                         double _position)
{
  return pimpl_->move_absolute(_axis, _position);
}

std::optional<std::string> ZMotionWrapper::move_relative(int _axis,
                                                         double _distance)
{
  return pimpl_->move_relative(_axis, _distance);
}

std::optional<std::string> ZMotionWrapper::move_velocity(int _axis,
                                                         double _velocity)
{
  return pimpl_->move_velocity(_axis, _velocity);
}

std::optional<std::string> ZMotionWrapper::move_line_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions)
{
  return pimpl_->move_line_absolute(_axes, _positions);
}

std::optional<std::string> ZMotionWrapper::move_line_relative(
    const std::vector<int> &_axes, const std::vector<double> &_distances)
{
  return pimpl_->move_line_relative(_axes, _distances);
}

std::optional<std::string> ZMotionWrapper::move_circular_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_circular_params)
{
  return pimpl_->move_circular_absolute(_axes, _positions, _circular_params);
}

std::optional<std::string> ZMotionWrapper::move_spiral_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_spiral_params)
{
  return pimpl_->move_spiral_absolute(_axes, _positions, _spiral_params);
}

std::optional<std::string> ZMotionWrapper::move_eclipse_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_eclipse_params)
{
  return pimpl_->move_eclipse_absolute(_axes, _positions, _eclipse_params);
}

std::optional<std::string> ZMotionWrapper::move_spherical_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_spherical_params)
{
  return pimpl_->move_spherical_absolute(_axes, _positions, _spherical_params);
}

std::optional<std::string> ZMotionWrapper::buffer_move(
    const std::vector<int> &_axes, const std::vector<double> &_positions)
{
  return pimpl_->buffer_move(_axes, _positions);
}

std::optional<std::string> ZMotionWrapper::start_continuous()
{
  return pimpl_->start_continuous();
}

std::optional<std::string> ZMotionWrapper::stop_continuous()
{
  return pimpl_->stop_continuous();
}

std::optional<std::string> ZMotionWrapper::set_dpos(int _axis, double _position)
{
  return pimpl_->set_dpos(_axis, _position);
}

std::optional<std::string> ZMotionWrapper::set_mpos(int _axis, double _position)
{
  return pimpl_->set_mpos(_axis, _position);
}

std::optional<int> ZMotionWrapper::get_atype(int _axis) const
{
  return pimpl_->get_atype(_axis);
}

std::optional<std::string> ZMotionWrapper::set_atype(int _axis, int _atype)
{
  return pimpl_->set_atype(_axis, _atype);
}

std::optional<double> ZMotionWrapper::CommandedPosition(int _axis) const
{
  return pimpl_->CommandedPosition(_axis);
}

std::optional<double> ZMotionWrapper::MeasuredPosition(int _axis) const
{
  return pimpl_->MeasuredPosition(_axis);
}

std::optional<double> ZMotionWrapper::MeasuredSpeed(int _axis) const
{
  return pimpl_->MeasuredSpeed(_axis);
}

std::optional<uint32_t> ZMotionWrapper::AxisStatus(int _axis) const
{
  return pimpl_->AxisStatus(_axis);
}

std::optional<bool> ZMotionWrapper::is_axis_idle(int _axis) const
{
  return pimpl_->is_axis_idle(_axis);
}

bool ZMotionWrapper::is_axis_moving(int _axis) const  // ✅ 修复#8：添加const
{
  return pimpl_->is_axis_moving(_axis);
}

// 运动缓冲 / 段查询
std::optional<int> ZMotionWrapper::get_moves_buffered(int _axis) const
{
  return pimpl_->get_moves_buffered(_axis);
}

std::optional<int> ZMotionWrapper::get_remain_buffer(int _axis) const
{
  return pimpl_->get_remain_buffer(_axis);
}

std::optional<int> ZMotionWrapper::get_move_curmark(int _axis) const
{
  return pimpl_->get_move_curmark(_axis);
}

std::optional<std::string> ZMotionWrapper::set_axis_enable(int _axis,
                                                           bool _enable)
{
  return pimpl_->set_axis_enable(_axis, _enable);
}

std::optional<bool> ZMotionWrapper::get_axis_enable(int _axis) const
{
  return pimpl_->get_axis_enable(_axis);
}

std::optional<std::string> ZMotionWrapper::set_op(int _bit, bool _state)
{
  return pimpl_->set_op(_bit, _state);
}

std::optional<std::string> ZMotionWrapper::set_brake(int _axis, bool _release)
{
  return pimpl_->set_brake(_axis, _release);
}

std::optional<std::string> ZMotionWrapper::stop_all()
{
  return pimpl_->stop_all();
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::connect(
    const std::string &_ip)
{
  // 调用 ZMotion API 连接
  int32_t ret = ZMC_OpenEth(const_cast<char *>(_ip.c_str()), &handle);
  if (ret != ERR_OK || handle == nullptr)
  {
    last_error = "Failed to connect to ZMC432 at " + _ip +
                 " (error code: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

void ZMotionWrapper::ZMotionWrapperPrivate::disconnect()
{
  if (handle != nullptr)
  {
    ZMC_Close(handle);
    handle = nullptr;
  }
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_units(
    int _axis, double _units)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  if (_axis < 0 || _axis >= static_cast<int>(axis_configs.size()))
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return last_error;
  }

  int32_t ret = ZAux_Direct_SetUnits(handle, _axis, static_cast<float>(_units));
  if (ret != ERR_OK)
  {
    last_error = "Set units failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  axis_configs[_axis].units = _units;
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_speed(
    int _axis, double _speed)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  if (_axis < 0 || _axis >= static_cast<int>(axis_configs.size()))
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return last_error;
  }

  int32_t ret = ZAux_Direct_SetSpeed(handle, _axis, static_cast<float>(_speed));
  if (ret != ERR_OK)
  {
    last_error = "Set speed failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::set_acceleration(int _axis,
                                                        double _accel)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  if (_axis < 0 || _axis >= static_cast<int>(axis_configs.size()))
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return last_error;
  }

  int32_t ret = ZAux_Direct_SetAccel(handle, _axis, static_cast<float>(_accel));
  if (ret != ERR_OK)
  {
    last_error = "Set acceleration failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::set_deceleration(int _axis,
                                                        double _decel)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  if (_axis < 0 || _axis >= static_cast<int>(axis_configs.size()))
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return last_error;
  }

  int32_t ret = ZAux_Direct_SetDecel(handle, _axis, static_cast<float>(_decel));
  if (ret != ERR_OK)
  {
    last_error = "Set deceleration failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}
// EtherCAT initialization implementation
std::optional<std::string> ZMotionWrapper::ecat_init(int slot_id,
                                                     const EcatInitInfo &info,
                                                     int timeout_ms)
{
  if (!is_connected())
  {
    return "not connected";
  }
  EcatInitInfoSet cinfo = info.toC();
  int32_t ret =
      ZAux_BusCmd_EcatInit(pimpl_->handle, slot_id, cinfo, timeout_ms);
  if (ret < 0)
  {
    return "ecat init error " + std::to_string(ret);
  }
  if (ret > 0)
  {
    return "aux command returned " + std::to_string(ret);
  }
  return std::nullopt;
}
std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::move_absolute(
    int _axis, double _position)
{
  auto config_err = ensure_axis_configured(_axis);
  if (config_err)
  {
    return config_err;
  }

  int64_t pulses = physical_to_pulses(_axis, _position);
  int axis_list[1] = {_axis};
  float distance_list[1] = {static_cast<float>(pulses)};
  int32_t ret = ZAux_Direct_MoveAbs(handle, 1, axis_list, distance_list);
  if (ret != ERR_OK)
  {
    last_error = "Move absolute failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::move_relative(
    int _axis, double _distance)
{
  auto config_err = ensure_axis_configured(_axis);
  if (config_err)
  {
    return config_err;
  }

  int64_t pulses = physical_to_pulses(_axis, _distance);
  int axis_list[1] = {_axis};
  float distance_list[1] = {static_cast<float>(pulses)};
  int32_t ret = ZAux_Direct_Move(handle, 1, axis_list, distance_list);
  if (ret != ERR_OK)
  {
    last_error = "Move relative failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::move_velocity(
    int _axis, double _velocity)
{
  auto config_err = ensure_axis_configured(_axis);
  if (config_err)
  {
    return config_err;
  }

  int direction = (_velocity >= 0) ? 1 : -1;
  int32_t ret = ZAux_Direct_Single_Vmove(handle, _axis, direction);
  if (ret != ERR_OK)
  {
    last_error = "Move velocity failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  ret = ZAux_Direct_SetSpeed(handle, _axis,
                             static_cast<float>(std::abs(_velocity)));
  if (ret != ERR_OK)
  {
    last_error = "Set speed failed for velocity move on axis " +
                 std::to_string(_axis) + " (error: " + std::to_string(ret) +
                 ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::move_line_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions)
{
  if (_axes.empty() || _positions.size() != _axes.size())
  {
    return "Invalid axes/positions size mismatch";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  std::vector<float> distance_list;
  distance_list.reserve(_axes.size());
  for (size_t i = 0; i < _axes.size(); ++i)
  {
    int64_t pulses = physical_to_pulses(_axes[i], _positions[i]);
    distance_list.push_back(static_cast<float>(pulses));
  }

  int32_t ret = ZAux_Direct_MoveAbs(handle, static_cast<int>(_axes.size()),
                                    const_cast<int *>(_axes.data()),
                                    distance_list.data());
  if (ret != ERR_OK)
  {
    last_error =
        "Move line absolute failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::move_line_relative(
    const std::vector<int> &_axes, const std::vector<double> &_distances)
{
  if (_axes.empty() || _distances.size() != _axes.size())
  {
    return "Invalid axes/distances size mismatch";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  std::vector<float> distance_list;
  distance_list.reserve(_axes.size());
  for (size_t i = 0; i < _axes.size(); ++i)
  {
    int64_t pulses = physical_to_pulses(_axes[i], _distances[i]);
    distance_list.push_back(static_cast<float>(pulses));
  }

  int32_t ret =
      ZAux_Direct_Move(handle, static_cast<int>(_axes.size()),
                       const_cast<int *>(_axes.data()), distance_list.data());
  if (ret != ERR_OK)
  {
    last_error =
        "Move line relative failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::move_circular_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_circular_params)
{
  if (_axes.size() != 2)
  {
    return "Circular interpolation currently requires exactly 2 axes";
  }
  if (_positions.size() != _axes.size())
  {
    return "Positions size mismatch with axes";
  }
  if (_circular_params.size() < 3)
  {
    return "Circular requires parameters: [center1, center2, direction]";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  const float end1 =
      static_cast<float>(physical_to_pulses(_axes[0], _positions[0]));
  const float end2 =
      static_cast<float>(physical_to_pulses(_axes[1], _positions[1]));
  const float center1 =
      static_cast<float>(physical_to_pulses(_axes[0], _circular_params[0]));
  const float center2 =
      static_cast<float>(physical_to_pulses(_axes[1], _circular_params[1]));
  const int direction = (_circular_params[2] >= 0.5) ? 1 : 0;

  int32_t ret = ZAux_Direct_MoveCircAbs(handle, static_cast<int>(_axes.size()),
                                        const_cast<int *>(_axes.data()), end1,
                                        end2, center1, center2, direction);
  if (ret != ERR_OK)
  {
    last_error =
        "Move circular absolute failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::move_spiral_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_spiral_params)
{
  if (_axes.size() < 2 || _axes.size() > 4)
  {
    return "Spiral interpolation requires 2 to 4 axes";
  }
  if (_positions.size() != _axes.size())
  {
    return "Spiral positions size mismatch with axes";
  }
  if (_spiral_params.size() < 4)
  {
    return "Spiral requires at least 4 parameters: [center1, center2, circles, "
           "pitch]";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  const float center1 =
      static_cast<float>(physical_to_pulses(_axes[0], _spiral_params[0]));
  const float center2 =
      static_cast<float>(physical_to_pulses(_axes[1], _spiral_params[1]));
  const float circles = static_cast<float>(_spiral_params[2]);
  const float pitch =
      static_cast<float>(physical_to_pulses(_axes[0], _spiral_params[3]));

  float distance3 = 0.0F;
  float distance4 = 0.0F;
  if (_axes.size() >= 3)
  {
    distance3 = static_cast<float>(physical_to_pulses(_axes[2], _positions[2]));
  }
  else if (_spiral_params.size() > 4)
  {
    distance3 = static_cast<float>(_spiral_params[4]);
  }

  if (_axes.size() >= 4)
  {
    distance4 = static_cast<float>(physical_to_pulses(_axes[3], _positions[3]));
  }
  else if (_spiral_params.size() > 5)
  {
    distance4 = static_cast<float>(_spiral_params[5]);
  }

  int32_t ret = ZAux_Direct_MoveSpiral(
      handle, static_cast<int>(_axes.size()), const_cast<int *>(_axes.data()),
      center1, center2, circles, pitch, distance3, distance4);
  if (ret != ERR_OK)
  {
    last_error =
        "Move spiral absolute failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::move_eclipse_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_eclipse_params)
{
  if (_axes.size() != 2)
  {
    return "Eclipse interpolation requires exactly 2 axes";
  }
  if (_positions.size() != 2)
  {
    return "Eclipse positions must have 2 values";
  }
  if (_eclipse_params.size() < 5)
  {
    return "Eclipse requires 5 parameters: [center1, center2, direction, adis, "
           "bdis]";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  const float end1 =
      static_cast<float>(physical_to_pulses(_axes[0], _positions[0]));
  const float end2 =
      static_cast<float>(physical_to_pulses(_axes[1], _positions[1]));
  const float center1 =
      static_cast<float>(physical_to_pulses(_axes[0], _eclipse_params[0]));
  const float center2 =
      static_cast<float>(physical_to_pulses(_axes[1], _eclipse_params[1]));
  const int direction = (_eclipse_params[2] >= 0.5) ? 1 : 0;
  const float adis =
      static_cast<float>(physical_to_pulses(_axes[0], _eclipse_params[3]));
  const float bdis =
      static_cast<float>(physical_to_pulses(_axes[1], _eclipse_params[4]));

  int32_t ret = ZAux_Direct_MEclipseAbs(
      handle, static_cast<int>(_axes.size()), const_cast<int *>(_axes.data()),
      end1, end2, center1, center2, direction, adis, bdis);
  if (ret != ERR_OK)
  {
    last_error =
        "Move eclipse absolute failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::move_spherical_absolute(
    const std::vector<int> &_axes, const std::vector<double> &_positions,
    const std::vector<double> &_spherical_params)
{
  if (_axes.size() != 3)
  {
    return "Spherical interpolation requires exactly 3 axes";
  }
  if (_positions.size() != 3)
  {
    return "Spherical positions must have 3 values";
  }
  if (_spherical_params.size() < 4)
  {
    return "Spherical requires at least 4 parameters: [center1, center2, "
           "center3, mode, ...]";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  const float end1 =
      static_cast<float>(physical_to_pulses(_axes[0], _positions[0]));
  const float end2 =
      static_cast<float>(physical_to_pulses(_axes[1], _positions[1]));
  const float end3 =
      static_cast<float>(physical_to_pulses(_axes[2], _positions[2]));
  const float center1 =
      static_cast<float>(physical_to_pulses(_axes[0], _spherical_params[0]));
  const float center2 =
      static_cast<float>(physical_to_pulses(_axes[1], _spherical_params[1]));
  const float center3 =
      static_cast<float>(physical_to_pulses(_axes[2], _spherical_params[2]));
  const int mode = static_cast<int>(_spherical_params[3]);
  const float center4 = (_spherical_params.size() > 4)
                            ? static_cast<float>(_spherical_params[4])
                            : 0.0F;
  const float center5 = (_spherical_params.size() > 5)
                            ? static_cast<float>(_spherical_params[5])
                            : 0.0F;

  int32_t ret = ZAux_Direct_MSpherical(
      handle, static_cast<int>(_axes.size()), const_cast<int *>(_axes.data()),
      end1, end2, end3, center1, center2, center3, mode, center4, center5);
  if (ret != ERR_OK)
  {
    last_error =
        "Move spherical absolute failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::buffer_move(
    const std::vector<int> &_axes, const std::vector<double> &_positions)
{
  if (_axes.empty() || _positions.size() != _axes.size())
  {
    return "Invalid axes/positions size mismatch";
  }

  for (int axis : _axes)
  {
    auto config_err = ensure_axis_configured(axis);
    if (config_err)
    {
      return config_err;
    }
  }

  std::stringstream cmd;
  cmd << "BUFFERMOVE(";
  for (size_t i = 0; i < _axes.size(); ++i)
  {
    if (i > 0)
      cmd << ",";
    int64_t pulses = physical_to_pulses(_axes[i], _positions[i]);
    cmd << _axes[i] << "," << pulses;
  }
  cmd << ")";

  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(
      handle, const_cast<char *>(cmd.str().c_str()), ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error = "Buffer move failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::start_continuous()
{
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle, "CONTINUE", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error = "Start continuous failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::stop_continuous()
{
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle, "STOP", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error = "Stop continuous failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }

  return std::nullopt;
}

std::optional<double> ZMotionWrapper::ZMotionWrapperPrivate::CommandedPosition(
    int _axis) const
{
  float dpos = 0.0f;
  int32_t ret = ZAux_Direct_GetDpos(handle, _axis, &dpos);
  if (ret != ERR_OK)
  {
    last_error = "Get commanded position failed for axis " +
                 std::to_string(_axis) + " (error: " + std::to_string(ret) +
                 ")";
    return std::nullopt;
  }
  return static_cast<double>(dpos);
}

std::optional<double> ZMotionWrapper::ZMotionWrapperPrivate::MeasuredPosition(
    int _axis) const
{
  float mpos = 0.0f;
  int32_t ret = ZAux_Direct_GetMpos(handle, _axis, &mpos);
  if (ret != ERR_OK)
  {
    last_error = "Get measured position failed for axis " +
                 std::to_string(_axis) + " (error: " + std::to_string(ret) +
                 ")";
    return std::nullopt;
  }
  return static_cast<double>(mpos);
}

std::optional<double> ZMotionWrapper::ZMotionWrapperPrivate::MeasuredSpeed(
    int _axis) const
{
  float mspeed = 0.0f;
  int32_t ret = ZAux_Direct_GetMspeed(handle, _axis, &mspeed);
  if (ret != ERR_OK)
  {
    last_error = "Get measured speed failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<double>(mspeed);
}

std::optional<uint32_t> ZMotionWrapper::ZMotionWrapperPrivate::AxisStatus(
    int _axis) const
{
  int32_t status = 0;
  int32_t ret = ZAux_Direct_GetAxisStatus(handle, _axis, &status);
  if (ret != ERR_OK)
  {
    last_error = "Get axis status failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<uint32_t>(status);
}

std::optional<bool> ZMotionWrapper::ZMotionWrapperPrivate::is_axis_idle(
    int _axis) const
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return std::nullopt;
  }

  int32_t value = 0;
  int32_t ret = ZAux_Direct_GetIfIdle(handle, _axis, &value);
  if (ret != ERR_OK)
  {
    last_error = "Get idle status failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }

  // ZMC: 0 表示运动中，-1 表示停止/空闲
  return (value == -1);
}

bool ZMotionWrapper::ZMotionWrapperPrivate::is_axis_moving(
    int _axis) const  // ✅ 修复#8：添加const
{
  auto status_opt = AxisStatus(_axis);
  if (!status_opt)
    return false;

  uint32_t status = status_opt.value();
  return (status & 0x00000002) != 0;
}

std::optional<int> ZMotionWrapper::ZMotionWrapperPrivate::get_moves_buffered(
    int _axis) const
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return std::nullopt;
  }

  int32_t value = 0;
  int32_t ret = ZAux_Direct_GetMovesBuffered(handle, _axis, &value);
  if (ret != ERR_OK)
  {
    last_error = "Get moves buffered failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<int>(value);
}

std::optional<int> ZMotionWrapper::ZMotionWrapperPrivate::get_remain_buffer(
    int _axis) const
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return std::nullopt;
  }

  int32_t value = 0;
  int32_t ret = ZAux_Direct_GetRemain_Buffer(handle, _axis, &value);
  if (ret != ERR_OK)
  {
    last_error = "Get remain buffer failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<int>(value);
}

std::optional<int> ZMotionWrapper::ZMotionWrapperPrivate::get_move_curmark(
    int _axis) const
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return std::nullopt;
  }

  int32_t value = 0;
  int32_t ret = ZAux_Direct_GetMoveCurmark(handle, _axis, &value);
  if (ret != ERR_OK)
  {
    last_error = "Get move curmark failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return static_cast<int>(value);
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::stop_all()
{
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle, "STOP ALL", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error = "Stop all failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::emergency_stop()
{
  char ack[2048] = {0};
  int32_t ret = ZAux_DirectCommand(handle, "EMERGENCY STOP", ack, sizeof(ack));
  if (ret != ERR_OK)
  {
    last_error = "Emergency stop failed (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_dpos(
    int _axis, double _position)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  int32_t ret =
      ZAux_Direct_SetDpos(handle, _axis, static_cast<float>(_position));
  if (ret != ERR_OK)
  {
    last_error = "Set DPOS failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_mpos(
    int _axis, double _position)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  int32_t ret =
      ZAux_Direct_SetMpos(handle, _axis, static_cast<float>(_position));
  if (ret != ERR_OK)
  {
    last_error = "Set MPOS failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<int> ZMotionWrapper::ZMotionWrapperPrivate::get_atype(
    int _axis) const
{
  if (!handle)
  {
    return std::nullopt;
  }

  int atype = 0;
  int32_t ret = ZAux_Direct_GetAtype(handle, _axis, &atype);
  if (ret != ERR_OK)
  {
    last_error = "Get atype failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return atype;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_atype(
    int _axis, int _atype)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  int32_t ret = ZAux_Direct_SetAtype(handle, _axis, _atype);
  if (ret != ERR_OK)
  {
    last_error = "Set atype failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

int64_t ZMotionWrapper::ZMotionWrapperPrivate::physical_to_pulses(
    int _axis, double _physical_position) const
{
  double units = CurrentUnits(_axis);
  return static_cast<int64_t>(_physical_position / units);
}

double ZMotionWrapper::ZMotionWrapperPrivate::pulses_to_physical(
    int _axis, int64_t _pulses) const
{
  double units = CurrentUnits(_axis);
  return _pulses * units;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::ensure_axis_configured(int _axis)
{
  if (_axis < 0 || _axis >= static_cast<int>(axis_configs.size()))
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return last_error;
  }

  if (!axis_configs[_axis].configured)
  {
    // 使用默认参数配置轴
    auto result = set_units(_axis, axis_configs[_axis].units);
    if (result)
      return result;

    result = set_speed(_axis, axis_configs[_axis].speed);
    if (result)
      return result;

    result = set_acceleration(_axis, axis_configs[_axis].acceleration);
    if (result)
      return result;

    result = set_deceleration(_axis, axis_configs[_axis].deceleration);
    if (result)
      return result;

    axis_configs[_axis].configured = true;
  }
  return std::nullopt;
}

std::optional<std::string>
ZMotionWrapper::ZMotionWrapperPrivate::set_axis_enable(int _axis, bool _enable)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }

  if (_axis < 0 || _axis >= 32)
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return last_error;
  }

  int32_t ret = ZAux_Direct_SetAxisEnable(handle, _axis, _enable ? 1 : 0);
  if (ret != ERR_OK)
  {
    last_error = "Set axis enable failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<bool> ZMotionWrapper::ZMotionWrapperPrivate::get_axis_enable(
    int _axis) const
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return std::nullopt;
  }

  if (_axis < 0 || _axis >= 32)
  {
    last_error = "Axis index out of range: " + std::to_string(_axis);
    return std::nullopt;
  }

  int enable_state = 0;
  int32_t ret = ZAux_Direct_GetAxisEnable(handle, _axis, &enable_state);
  if (ret != ERR_OK)
  {
    last_error = "Get axis enable failed for axis " + std::to_string(_axis) +
                 " (error: " + std::to_string(ret) + ")";
    return std::nullopt;
  }
  return enable_state != 0;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_op(
    int _bit, bool _state)
{
  if (!handle)
  {
    last_error = "Controller not connected";
    return last_error;
  }
  int32_t ret = ZAux_Direct_SetOp(handle, _bit, _state ? 1 : 0);
  if (ret != ERR_OK)
  {
    last_error = "Set OP failed for bit " + std::to_string(_bit) +
                 " (error: " + std::to_string(ret) + ")";
    return last_error;
  }
  return std::nullopt;
}

std::optional<std::string> ZMotionWrapper::ZMotionWrapperPrivate::set_brake(
    int _axis, bool _release)
{
  // 假设我们将 IO 映射规则为: 刹车 IO = 起始 IO位 + 轴号
  // 后续如果参数化可以传入一个 map，这里给出基础的机械/电气抱闸控制逻辑
  int brake_io = 0 + _axis;  // TEMP mapping
  return set_op(brake_io, _release);
}

double ZMotionWrapper::ZMotionWrapperPrivate::CurrentUnits(int _axis) const
{
  if (_axis >= 0 && _axis < 32 && axis_configs[_axis].configured)
  {
    return axis_configs[_axis].units;
  }
  return 1.0;
}

void ZMotionWrapper::ZMotionWrapperPrivate::mark_axis_configured(int _axis)
{
  if (_axis >= 0 && _axis < static_cast<int>(axis_configs.size()))
  {
    axis_configs[_axis].configured = true;
  }
}

void ZMotionWrapper::mark_axis_configured(int _axis)
{
  pimpl_->mark_axis_configured(_axis);
}

}  // namespace zmc432_driver
