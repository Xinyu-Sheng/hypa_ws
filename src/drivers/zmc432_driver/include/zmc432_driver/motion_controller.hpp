#ifndef MOTION_CONTROLLER_HPP
#define MOTION_CONTROLLER_HPP

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "zmc432_driver/zmotion_wrapper.hpp"

namespace zmc432_driver
{

/**
 * @brief 运动控制器
 *
 * 管理多轴运动协调、命令缓冲队列和执行线程。
 * 支持单轴独立运动、多轴插补运动、连续轨迹流式缓冲。
 * 使用 PIMPL 模式隐藏实现细节。
 */
class MotionController
{
  public:
  // 公有类型定义（外部需要使用的类型）
  enum MotionType : uint8_t
  {
    SINGLE_AXIS = 1,
    INTERPOLATED = 2,
    CONTINUOUS_TRAJECTORY = 3
  };

  enum InterpolationMode : uint8_t
  {
    LINEAR = 0,
    CIRCULAR = 1,
    SPIRAL = 2,
    ECLIPSE = 3,
    SPHERICAL = 4
  };

  struct MotionCommand
  {
    MotionType motion_type = SINGLE_AXIS;
    InterpolationMode interpolation_mode = LINEAR;
    std::vector<int> axes;
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    std::vector<double> decelerations;
    std::vector<double> circular_params;
    double wait_time = 0.0;
  };

  struct AxisStatus
  {
    double position = 0.0;     // 规划位置
    double feedback = 0.0;     // 反馈位置
    double speed = 0.0;        // 实时反馈速度 (MSPEED)
    uint32_t status_word = 0;  // 状态字
    int type = 0;              // 轴类型 (ATYPE)
    bool moving = false;       // 是否在运动
    bool error = false;        // 是否有错误
    bool enabled = false;      // 是否使能
  };

  struct ControllerStatus
  {
    std::map<int, AxisStatus> axis_statuses;
    std::vector<int> executing_axes;  // 当前执行命令涉及的轴
    bool executing = false;
    bool stop_requested = false;
    double progress = 0.0;  // 0-100%
  };

  MotionController();
  ~MotionController();

  // 禁止拷贝
  MotionController(const MotionController &) = delete;
  MotionController &operator=(const MotionController &) = delete;

  // PIMPL 私有实现类
  class MotionControllerPrivate;
  std::unique_ptr<MotionControllerPrivate> pimpl_;

  // 公共接口
  std::optional<std::string> initialize(const std::string &_controller_ip);
  std::optional<std::string> configure_axis(int _axis, double _units,
                                            double _speed, double _accel,
                                            double _decel);
  std::optional<std::string> reset_axis_position(int _axis,
                                                 double _position = 0.0);
  std::optional<std::string> enable_axis(int _axis, bool _enable);
  std::optional<std::string> enable_all_axes(bool _enable);
  std::optional<bool> get_axis_enable(int _axis) const;

  // EtherCAT bus initialization
  std::optional<std::string> initialize_bus(const EcatInitInfo &info,
                                            int slot = 0,
                                            int timeout_ms = 5000);
  bool start();
  void stop();
  bool queue_motion(const MotionCommand &_cmd);
  ControllerStatus CurrentStatus() const;
  void cancel_current_motion();
  bool is_executing() const;
  bool wait_for_completion(int _timeout_ms = 0);

  private:
  // 私有方法（委托给 pimpl_）
  void execution_loop();
  std::optional<std::string> execute_single_axis(const MotionCommand &_cmd);
  std::optional<std::string> execute_interpolated(const MotionCommand &_cmd);
  std::optional<std::string> execute_continuous_trajectory(
      const MotionCommand &_cmd);
  void update_status();
  double calculate_progress(const MotionCommand &_cmd) const;
  bool is_motion_complete(const MotionCommand &_cmd,
                           bool use_idle_check = true) const;
};

}  // namespace zmc432_driver

#endif  // MOTION_CONTROLLER_HPP
