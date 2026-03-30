#ifndef ZMOTION_WRAPPER_HPP
#define ZMOTION_WRAPPER_HPP

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "zmc432_driver/ecat_init.hpp"
#include "zmc432_driver/zmcaux.h"
#include "zmc432_driver/zmotion.h"

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
  ZMotionWrapper(const ZMotionWrapper &) = delete;
  ZMotionWrapper &operator=(const ZMotionWrapper &) = delete;

  class ZMotionWrapperPrivate
  {
    public:
    ZMotionWrapperPrivate();
    ~ZMotionWrapperPrivate();

    // 禁止拷贝
    ZMotionWrapperPrivate(const ZMotionWrapperPrivate &) = delete;
    ZMotionWrapperPrivate &operator=(const ZMotionWrapperPrivate &) = delete;

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
    std::optional<std::string> connect(const std::string &_ip);
    void disconnect();  // 断开连接
    bool is_connected() const
    {
      return handle != nullptr;
    }  // 通过检查句柄检查是否连接
    std::optional<std::string> set_units(
        int _axis, double _units);  // 写入units转换到控制器
    std::optional<std::string> set_speed(
        int _axis, double _speed);  // 写入最大速度到控制器
    std::optional<std::string> set_acceleration(
        int _axis, double _accel);  // 写入最大加速度到控制器
    std::optional<std::string> set_deceleration(
        int _axis, double _decel);  // 写入最大减速度到控制器
    std::optional<std::string> move_absolute(
        int _axis,
        double _position);  // TODO：更换到ZAux_Direct_Single_MoveAbs
                            // 单轴相对移动
    std::optional<std::string> move_relative(
        int _axis,
        double _distance);  // TODO：更换到ZAux_Direct_Single_Move 单轴绝对移动
    std::optional<std::string> move_velocity(
        int _axis,
        double
            _velocity);  // 保留接口，没有用到，因为设置轴为位置控制，最终可删除。
    std::optional<std::string> move_line_absolute(
        const std::vector<int> &_axes,
        const std::vector<double> &_positions);  // 多轴绝对位置移动
    std::optional<std::string> move_line_relative(
        const std::vector<int> &_axes,
        const std::vector<double> &_distances);  // 多轴相对位置移动
    std::optional<std::string> move_circular_absolute(
        const std::vector<int> &_axes, const std::vector<double> &_positions,
        const std::vector<double> &
            _circular_params);  // 保留接口，没有用到，因为目前没有这个运动需求，最终可删除。
    std::optional<std::string> move_spiral_absolute(
        const std::vector<int> &_axes, const std::vector<double> &_positions,
        const std::vector<double> &
            _spiral_params);  // 保留接口，没有用到，因为目前没有这个运动需求，最终可删除。
    std::optional<std::string> move_eclipse_absolute(
        const std::vector<int> &_axes, const std::vector<double> &_positions,
        const std::vector<double> &
            _eclipse_params);  // 保留接口，没有用到，因为目前没有这个运动需求，最终可删除。
    std::optional<std::string> move_spherical_absolute(
        const std::vector<int> &_axes, const std::vector<double> &_positions,
        const std::vector<double> &
            _spherical_params);  // 保留接口，没有用到，因为目前没有这个运动需求，最终可删除。
    std::optional<std::string> buffer_move(
        const std::vector<int> &_axes,
        const std::vector<double> &_positions);  // TODO：根本不存在！
    std::optional<std::string>
    start_continuous();  // TODO: 由：单轴：ZAux_Direct_MoveResume替代
    std::optional<std::string>
    stop_continuous();  // TODO:由：ZAux_Direct_MovePause
    std::optional<std::string>
    stop_all();  // TODO:
                 // 本意是稳定停止，但是调用ZBASIC不存在的指令，由ZAux_Direct_MoveStop替代
    std::optional<std::string>
    emergency_stop_all();  // TODO: 由ZAux_Direct_QuickStopAll替代
    std::optional<std::string> set_dpos(
        int _axis,
        double _position);  // 只使用绝对运动指令以机械零点为基准，完全不依赖
                            // DPOS，这是相对位置计算需要的：发 3
                            // 条连续相对运动指令：X+50 → X+30 →
                            // X+20；控制器需要通过 DPOS 连续计算： 第 1
                            // 条终点：DPOS = 50 → 第 2 条起点 = 50 → 终点 = 80
                            // → 第 3 条起点 = 80 → 终点 = 100
    std::optional<std::string> set_mpos(
        int _axis, double _position);               // TODO：到底怎么回事？
    std::optional<int> get_atype(int _axis) const;  // 获取轴当前类型
    std::optional<std::string> set_atype(
        int _axis,
        int _atype);  // 设置轴状态，EtherCAT 总线轴为65：周期位置模式
    std::optional<double> CommandedPosition(int _axis) const;
    std::optional<double> MeasuredPosition(
        int _axis) const;                                  // 获取轴当前实际位置
    std::optional<double> MeasuredSpeed(int _axis) const;  // 获取当前轴运行速度
    std::optional<uint32_t> AxisStatus(int _axis) const;   // 获取轴状态，参考表
    std::optional<bool> is_axis_idle(
        int _axis) const;  // 运动缓冲区为空、最后一条运动指令物理执行完成
    bool is_axis_moving(int _axis)
        const;  // 从轴状态判断是否正在运动，参考表
                // "Moving位"：即使缓冲区有，但是暂停了也不算moving：只有电机在运动才是

    std::optional<int> get_moves_buffered(
        int _axis) const;  // 已经排队、等待执行的运动指令 有多少条
    std::optional<int> get_remain_buffer(
        int _axis) const;  // 还能存入多少条新的运动指令
    std::optional<int> get_move_curmark(int _axis) const;

    std::optional<std::string> set_axis_enable(
        int _axis, bool _enable);  // 伺服电机通电、抱闸松开
    std::optional<bool> get_axis_enable(
        int _axis) const;  // 查看是否是enable状态

    // IO and Brake control
    std::optional<std::string> set_op(int _bit,
                                      bool _state);  // TODO: 暂时仍不控制外接闸
    std::optional<std::string> set_brake(
        int _axis, bool _release);  // TODO: 暂时仍不控制外接闸

    void mark_axis_configured(int _axis);  // 确保当前轴已经配置了

    // internal implementation for EtherCAT 总线初始化
    std::optional<std::string> ecat_init(int slot_id, const EcatInitInfo &info,
                                         int timeout_ms);

    // TODO：添加API替换/配合
    // 暂停【暂停】（暂停运动，保持伺服使能，后续可继续跑）
    // ✅ 多轴插补运动（必用）：ZAux_Direct_MovePauseGroup
    // ✅ 单轴运动：ZAux_Direct_MovePause
    // 暂停后【继续运动】
    // ✅ 多轴：ZAux_Direct_MoveResumeGroup
    // ✅ 单轴：ZAux_Direct_MoveResume
    // 彻底停止（终止运动，清空指令，不能恢复）
    // ✅ 多轴：ZAux_Direct_MoveStopGroup
    // ✅ 单轴：ZAux_Direct_MoveStop
    // 紧急停止（所有轴同步停、最快响应、抱闸锁定）
    // ✅ 单/多轴：ZAux_Direct_QuickStopAll	紧急情况（安全优先）

    private:
    int64_t physical_to_pulses(int _axis, double _physical_position)
        const;  // TODO：应该不需要，因为只需要发送单位量，不用转换
    double pulses_to_physical(int _axis, int64_t _pulses)
        const;  // TODO：应该不需要，因为只需要发送单位量，不用转换
    std::optional<std::string> ensure_axis_configured(
        int _axis);  // 给没有配置过的，配置默认值，保证可以运行（不保证正确运行）
    double CurrentUnits(int _axis) const;
  };

  std::unique_ptr<ZMotionWrapperPrivate> pimpl_;

  // 公共接口方法（委托给 pimpl_）
  std::optional<std::string> connect(const std::string &_ip);
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
      const std::vector<int> &_axes, const std::vector<double> &_positions);
  std::optional<std::string> move_line_relative(
      const std::vector<int> &_axes, const std::vector<double> &_distances);
  std::optional<std::string> move_circular_absolute(
      const std::vector<int> &_axes, const std::vector<double> &_positions,
      const std::vector<double> &_circular_params);
  std::optional<std::string> move_spiral_absolute(
      const std::vector<int> &_axes, const std::vector<double> &_positions,
      const std::vector<double> &_spiral_params);
  std::optional<std::string> move_eclipse_absolute(
      const std::vector<int> &_axes, const std::vector<double> &_positions,
      const std::vector<double> &_eclipse_params);
  std::optional<std::string> move_spherical_absolute(
      const std::vector<int> &_axes, const std::vector<double> &_positions,
      const std::vector<double> &_spherical_params);
  std::optional<std::string> buffer_move(const std::vector<int> &_axes,
                                         const std::vector<double> &_positions);
  std::optional<std::string> start_continuous();
  std::optional<std::string> stop_continuous();
  std::optional<std::string> set_dpos(int _axis, double _position);
  std::optional<std::string> set_mpos(int _axis, double _position);
  std::optional<int> get_atype(int _axis) const;
  std::optional<std::string> set_atype(int _axis, int _atype);
  std::optional<std::string> stop_all();
  std::optional<std::string> emergency_stop_all();
  std::optional<double> CommandedPosition(int _axis) const;
  std::optional<double> MeasuredPosition(int _axis) const;
  std::optional<double> MeasuredSpeed(int _axis) const;
  std::optional<uint32_t> AxisStatus(int _axis) const;
  std::optional<bool> is_axis_idle(int _axis) const;
  bool is_axis_moving(int _axis) const;

  // 运动缓冲 / 段查询
  std::optional<int> get_moves_buffered(int _axis) const;
  std::optional<int> get_remain_buffer(int _axis) const;
  std::optional<int> get_move_curmark(int _axis) const;

  std::optional<std::string> set_axis_enable(int _axis, bool _enable);
  std::optional<bool> get_axis_enable(int _axis) const;

  // IO and Brake control
  std::optional<std::string> set_op(int _bit, bool _state);
  std::optional<std::string> set_brake(int _axis, bool _release);

  void mark_axis_configured(int _axis);

  // EtherCAT bus initialization
  std::optional<std::string> ecat_init(int slot_id, const EcatInitInfo &info,
                                       int timeout_ms);

  [[deprecated("use ecat_init instead")]]
  std::optional<std::string> ecat_scan(int slot_id, const EcatInitInfo &info,
                                       int timeout_ms)
  {
    return ecat_init(slot_id, info, timeout_ms);
  }
};

}  // namespace zmc432_driver

#endif  // ZMOTION_WRAPPER_HPP
