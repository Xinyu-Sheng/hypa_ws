#ifndef ZMOTION_WRAPPER_HPP
#define ZMOTION_WRAPPER_HPP

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include "zmc432_driver/zmotion.h"
#include "zmc432_driver/zmcaux.h"
#include "zmc432_driver/zmotion_ecat.h"
#include "zmc432_driver/ecat_init.hpp"

namespace zmc432_driver
{

/**
 * @brief ZMotion SDK 的 C++ 封装类
 *
 * 提供面向对象的接口，封装 ZMC432 控制器的底层 API 调用。
 * 所有方法线程安全（假设单线程调用，由 MotionController 保证）。
 * 使用 PIMPL 模式隐藏实现细节。
 */
class ZMotionWrapper
{
 public:
  ZMotionWrapper();
  ~ZMotionWrapper();

  // 禁止拷贝
  ZMotionWrapper(const ZMotionWrapper&) = delete;
  ZMotionWrapper& operator=(const ZMotionWrapper&) = delete;

  class ZMotionWrapperPrivate
  {
   public:
    ZMotionWrapperPrivate();
    ~ZMotionWrapperPrivate();

    // 禁止拷贝
    ZMotionWrapperPrivate(const ZMotionWrapperPrivate&) = delete;
    ZMotionWrapperPrivate& operator=(const ZMotionWrapperPrivate&) = delete;

    ZMC_HANDLE handle{nullptr};
    mutable std::string last_error;

    // 轴配置缓存
    struct AxisConfigCache
    {
      double units = 1.0;           // 默认 1.0（脉冲单位）
      double speed = 10.0;          // 默认速度
      double acceleration = 100.0;  // 默认加速度
      double deceleration = 100.0;  // 默认减速度
      bool configured = false;
    };
    std::vector<AxisConfigCache> axis_configs;

    // 实现方法（公共，供 ZMotionWrapper 调用）
    std::optional<std::string> connect(const std::string& _ip);
    void disconnect();
    bool is_connected() const { return handle != nullptr; }
    std::optional<std::string> set_units(int _axis, double _units);
    std::optional<std::string> set_speed(int _axis, double _speed);
    std::optional<std::string> set_acceleration(int _axis, double _accel);
    std::optional<std::string> set_deceleration(int _axis, double _decel);
    std::optional<std::string> move_absolute(int _axis, double _position);
    std::optional<std::string> move_relative(int _axis, double _distance);
    std::optional<std::string> move_velocity(int _axis, double _velocity);
    std::optional<std::string> move_line_absolute(
        const std::vector<int>& _axes, const std::vector<double>& _positions);
    std::optional<std::string> move_line_relative(
        const std::vector<int>& _axes, const std::vector<double>& _distances);
    std::optional<std::string> move_circular_absolute(
        const std::vector<int>& _axes, const std::vector<double>& _positions,
        const std::vector<double>& _circular_params);
    std::optional<std::string> move_spiral_absolute(
        const std::vector<int>& _axes, const std::vector<double>& _positions,
        const std::vector<double>& _spiral_params);
    std::optional<std::string> move_eclipse_absolute(
        const std::vector<int>& _axes, const std::vector<double>& _positions,
        const std::vector<double>& _eclipse_params);
    std::optional<std::string> move_spherical_absolute(
        const std::vector<int>& _axes, const std::vector<double>& _positions,
        const std::vector<double>& _spherical_params);
    std::optional<std::string> buffer_move(
        const std::vector<int>& _axes, const std::vector<double>& _positions);
    std::optional<std::string> start_continuous();
    std::optional<std::string> stop_continuous();
    std::optional<std::string> stop_all();
    std::optional<std::string> emergency_stop();
    std::optional<double> Position(int _axis) const;
    std::optional<double> Feedback(int _axis) const;
    std::optional<double> Speed(int _axis) const;
    std::optional<uint32_t> AxisStatus(int _axis) const;
    bool is_axis_moving(int _axis);
    std::optional<std::string> set_axis_enable(int _axis, bool _enable);
    std::optional<bool> get_axis_enable(int _axis) const;

    // internal implementation for EtherCAT 总线初始化
    std::optional<std::string> ecat_init(int slot_id, const EcatInitInfo& info,
                                         int timeout_ms);

   private:
    int64_t physical_to_pulses(int _axis, double _physical_position) const;
    double pulses_to_physical(int _axis, int64_t _pulses) const;
    std::optional<std::string> ensure_axis_configured(int _axis);
    double CurrentUnits(int _axis) const;
  };

  std::unique_ptr<ZMotionWrapperPrivate> pimpl_;

  // 公共接口方法（委托给 pimpl_）
  std::optional<std::string> connect(const std::string& _ip);
  void disconnect();
  bool is_connected() const;
  std::optional<std::string> set_units(int _axis, double _units);
  std::optional<std::string> set_speed(int _axis, double _speed);
  std::optional<std::string> set_acceleration(int _axis, double _accel);
  std::optional<std::string> set_deceleration(int _axis, double _decel);
  std::optional<std::string> move_absolute(int _axis, double _position);
  std::optional<std::string> move_relative(int _axis, double _distance);
  std::optional<std::string> move_velocity(int _axis, double _velocity);
  std::optional<std::string> move_line_absolute(
      const std::vector<int>& _axes, const std::vector<double>& _positions);
  std::optional<std::string> move_line_relative(
      const std::vector<int>& _axes, const std::vector<double>& _distances);
  std::optional<std::string> move_circular_absolute(
      const std::vector<int>& _axes, const std::vector<double>& _positions,
      const std::vector<double>& _circular_params);
  std::optional<std::string> move_spiral_absolute(
      const std::vector<int>& _axes, const std::vector<double>& _positions,
      const std::vector<double>& _spiral_params);
  std::optional<std::string> move_eclipse_absolute(
      const std::vector<int>& _axes, const std::vector<double>& _positions,
      const std::vector<double>& _eclipse_params);
  std::optional<std::string> move_spherical_absolute(
      const std::vector<int>& _axes, const std::vector<double>& _positions,
      const std::vector<double>& _spherical_params);
  std::optional<std::string> buffer_move(const std::vector<int>& _axes,
                                         const std::vector<double>& _positions);
  std::optional<std::string> start_continuous();
  std::optional<std::string> stop_continuous();
  std::optional<std::string> stop_all();
  std::optional<std::string> emergency_stop();
  std::optional<double> Position(int _axis) const;
  std::optional<double> Feedback(int _axis) const;
  std::optional<double> Speed(int _axis) const;
  std::optional<uint32_t> AxisStatus(int _axis) const;
  bool is_axis_moving(int _axis);
  std::optional<std::string> set_axis_enable(int _axis, bool _enable);
  std::optional<bool> get_axis_enable(int _axis) const;

  // EtherCAT bus initialization
  std::optional<std::string> ecat_init(int slot_id, const EcatInitInfo& info,
                                       int timeout_ms);

  [[deprecated("use ecat_init instead")]]
  std::optional<std::string> ecat_scan(int slot_id, const EcatInitInfo& info,
                                       int timeout_ms)
  {
    return ecat_init(slot_id, info, timeout_ms);
  }
};

}  // namespace zmc432_driver

#endif  // ZMOTION_WRAPPER_HPP
