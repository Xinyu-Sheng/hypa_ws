#include "zmc432_driver/motion_controller.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>

namespace zmc432_driver
{

// Private implementation class
class MotionController::MotionControllerPrivate
{
 public:
  MotionControllerPrivate() = default;
  ~MotionControllerPrivate() = default;

  // 禁止拷贝
  MotionControllerPrivate(const MotionControllerPrivate&) = delete;
  MotionControllerPrivate& operator=(const MotionControllerPrivate&) = delete;

  std::unique_ptr<ZMotionWrapper> zmotion;
  std::thread execution_thread;
  std::mutex buffer_mutex;
  std::condition_variable buffer_cv;
  std::deque<MotionController::MotionCommand> command_buffer;
  std::atomic<bool> running{false};
  std::atomic<bool> executing{false};
  std::atomic<bool> cancel_requested{false};

  // 轴配置
  struct AxisConfig
  {
    double units = 1.0;
    double speed = 10.0;
    double acceleration = 100.0;
    double deceleration = 100.0;
    bool configured = false;
  };
  std::map<int, AxisConfig> axis_configs;

  // 执行线程主循环
  void execution_loop(MotionController* _public_interface);

  // 执行方法
  std::optional<std::string> execute_single_axis(
      const MotionController::MotionCommand& _cmd);
  std::optional<std::string> execute_interpolated(
      const MotionController::MotionCommand& _cmd);
  std::optional<std::string> execute_continuous_trajectory(
      const MotionController::MotionCommand& _cmd);

  // 辅助方法
  void update_status();
  double calculate_progress(const MotionController::MotionCommand& _cmd) const;
  bool is_motion_complete(const MotionController::MotionCommand& _cmd) const;
};

// Public interface implementations
MotionController::MotionController()
    : pimpl_(std::make_unique<MotionControllerPrivate>())
{
}

MotionController::~MotionController() { stop(); }

std::optional<std::string> MotionController::initialize(
    const std::string& _controller_ip)
{
  pimpl_->zmotion = std::make_unique<ZMotionWrapper>();

  // 连接控制器
  auto connect_result = pimpl_->zmotion->connect(_controller_ip);
  if (connect_result)
  {
    return connect_result;
  }

  // 可以在这里配置一些默认轴参数
  // 具体配置由用户通过 configure_axis 完成

  return std::nullopt;
}

std::optional<std::string> MotionController::configure_axis(
    int _axis, double _units, double _speed, double _accel, double _decel)
{
  if (!pimpl_->zmotion || !pimpl_->zmotion->is_connected())
  {
    return "Controller not connected";
  }

  // 设置 units
  auto result = pimpl_->zmotion->set_units(_axis, _units);
  if (result) return result;

  // 设置速度、加速度、减速度
  result = pimpl_->zmotion->set_speed(_axis, _speed);
  if (result) return result;

  result = pimpl_->zmotion->set_acceleration(_axis, _accel);
  if (result) return result;

  result = pimpl_->zmotion->set_deceleration(_axis, _decel);
  if (result) return result;

  // 缓存配置
  MotionControllerPrivate::AxisConfig config;
  config.units = _units;
  config.speed = _speed;
  config.acceleration = _accel;
  config.deceleration = _decel;
  config.configured = true;
  pimpl_->axis_configs[_axis] = config;

  return std::nullopt;
}

bool MotionController::start()
{
  if (pimpl_->running)
  {
    return true;  // 已经在运行
  }

  if (!pimpl_->zmotion || !pimpl_->zmotion->is_connected())
  {
    return false;
  }

  pimpl_->running = true;
  pimpl_->executing = false;
  pimpl_->cancel_requested = false;

  // 启动执行线程
  pimpl_->execution_thread =
      std::thread(&MotionControllerPrivate::execution_loop, pimpl_.get(), this);

  return true;
}

void MotionController::stop()
{
  if (!pimpl_->running)
  {
    return;
  }

  pimpl_->running = false;
  pimpl_->cancel_requested = true;

  // 唤醒执行线程
  pimpl_->buffer_cv.notify_all();

  // 等待线程结束
  if (pimpl_->execution_thread.joinable())
  {
    pimpl_->execution_thread.join();
  }

  // 断开连接
  if (pimpl_->zmotion)
  {
    pimpl_->zmotion->disconnect();
  }
}

bool MotionController::queue_motion(const MotionCommand& _cmd)
{
  std::lock_guard<std::mutex> lock(pimpl_->buffer_mutex);

  // 检查命令有效性
  if (_cmd.axes.empty())
  {
    return false;
  }
  if (_cmd.positions.size() != _cmd.axes.size())
  {
    return false;
  }
  if (!_cmd.velocities.empty() && _cmd.velocities.size() != _cmd.axes.size())
  {
    return false;
  }
  if (!_cmd.accelerations.empty() &&
      _cmd.accelerations.size() != _cmd.axes.size())
  {
    return false;
  }
  if (!_cmd.decelerations.empty() &&
      _cmd.decelerations.size() != _cmd.axes.size())
  {
    return false;
  }

  // 检查所有轴是否已配置
  for (int axis : _cmd.axes)
  {
    if (pimpl_->axis_configs.find(axis) == pimpl_->axis_configs.end())
    {
      return false;
    }
  }

  // 检查连续轨迹模式下的参数
  if (_cmd.motion_type == CONTINUOUS_TRAJECTORY)
  {
    // 连续轨迹需要至少一个速度参数
    if (_cmd.velocities.empty())
    {
      return false;
    }
  }

  // 入队
  pimpl_->command_buffer.push_back(_cmd);

  // 唤醒执行线程
  pimpl_->buffer_cv.notify_one();

  return true;
}

MotionController::ControllerStatus MotionController::CurrentStatus() const
{
  ControllerStatus status;
  status.executing = pimpl_->executing;
  status.stop_requested = pimpl_->cancel_requested;

  // 读取所有已配置轴的状态
  for (const auto& [axis, config] : pimpl_->axis_configs)
  {
    AxisStatus axis_status;

    if (pimpl_->zmotion)
    {
      axis_status.position = pimpl_->zmotion->Position(axis).value_or(0.0);
      axis_status.feedback = pimpl_->zmotion->Feedback(axis).value_or(0.0);
      axis_status.speed = pimpl_->zmotion->Speed(axis).value_or(0.0);
      auto status_opt = pimpl_->zmotion->AxisStatus(axis);
      if (status_opt)
      {
        axis_status.status_word = status_opt.value();
        // 根据状态字判断是否在运动和错误状态
        // 这里需要根据 zmotion.h 中的 AXISSTATUS 位定义来解析
        axis_status.moving =
            (axis_status.status_word & 0x00000002) != 0;  // BIT1: MOVING
        axis_status.error =
            (axis_status.status_word & 0x00000004) != 0;  // BIT2: ERROR (假设)
      }
    }

    status.axis_statuses[axis] = axis_status;
  }

  status.progress = 0.0;
  // 如果有正在执行的命令，计算进度
  // 这里暂时返回 0，实际由 execute_goal 中的反馈更新

  return status;
}

void MotionController::cancel_current_motion()
{
  pimpl_->cancel_requested = true;

  // 立即停止所有轴
  if (pimpl_->zmotion)
  {
    pimpl_->zmotion->stop_all();
  }
}

bool MotionController::wait_for_completion(int _timeout_ms)
{
  auto start_time = std::chrono::steady_clock::now();

  while (is_executing())
  {
    if (pimpl_->cancel_requested)
    {
      return false;
    }

    if (_timeout_ms > 0)
    {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start_time)
                         .count();
      if (elapsed >= _timeout_ms)
      {
        return false;  // 超时
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return true;
}

bool MotionController::is_executing() const
{
  return pimpl_ && pimpl_->executing;
}

// Private method implementations (delegated to pimpl)
void MotionController::execution_loop() { pimpl_->execution_loop(this); }

std::optional<std::string> MotionController::execute_single_axis(
    const MotionCommand& _cmd)
{
  return pimpl_->execute_single_axis(_cmd);
}

std::optional<std::string> MotionController::execute_interpolated(
    const MotionCommand& _cmd)
{
  return pimpl_->execute_interpolated(_cmd);
}

std::optional<std::string> MotionController::execute_continuous_trajectory(
    const MotionCommand& _cmd)
{
  return pimpl_->execute_continuous_trajectory(_cmd);
}

void MotionController::update_status() { pimpl_->update_status(); }

double MotionController::calculate_progress(const MotionCommand& _cmd) const
{
  return pimpl_->calculate_progress(_cmd);
}

bool MotionController::is_motion_complete(const MotionCommand& _cmd) const
{
  return pimpl_->is_motion_complete(_cmd);
}

// MotionControllerPrivate method implementations
void MotionController::MotionControllerPrivate::execution_loop(
    MotionController* _public_interface)
{
  while (running)
  {
    MotionController::MotionCommand cmd;

    // 等待命令或停止信号
    {
      std::unique_lock<std::mutex> lock(buffer_mutex);
      buffer_cv.wait(lock,
                     [this]() { return !command_buffer.empty() || !running; });

      if (!running)
      {
        break;
      }

      if (command_buffer.empty())
      {
        continue;
      }

      cmd = command_buffer.front();
      command_buffer.pop_front();
      executing = true;
      cancel_requested = false;
    }

    // 执行命令
    std::optional<std::string> result;

    switch (cmd.motion_type)
    {
      case MotionController::SINGLE_AXIS:
        result = execute_single_axis(cmd);
        break;

      case MotionController::INTERPOLATED:
        result = execute_interpolated(cmd);
        break;

      case MotionController::CONTINUOUS_TRAJECTORY:
        result = execute_continuous_trajectory(cmd);
        break;

      default:
        result = "Unknown motion type: " +
                 std::to_string(static_cast<int>(cmd.motion_type));
        break;
    }

    if (result)
    {
      std::cerr << "Motion execution failed: " << result.value() << std::endl;
    }

    // 标记执行完成
    {
      std::lock_guard<std::mutex> lock(buffer_mutex);
      executing = false;
    }
  }
}

std::optional<std::string>
MotionController::MotionControllerPrivate::execute_single_axis(
    const MotionController::MotionCommand& _cmd)
{
  if (_cmd.axes.size() != 1)
  {
    return "SINGLE_AXIS requires exactly 1 axis";
  }

  int axis = _cmd.axes[0];
  double target_position = _cmd.positions[0];

  // 如果提供了速度，先设置
  if (!_cmd.velocities.empty())
  {
    auto speed_result = zmotion->set_speed(axis, _cmd.velocities[0]);
    if (speed_result) return speed_result;
  }

  // 如果提供了加速度/减速度，先设置
  if (!_cmd.accelerations.empty())
  {
    auto accel_result = zmotion->set_acceleration(axis, _cmd.accelerations[0]);
    if (accel_result) return accel_result;
  }
  if (!_cmd.decelerations.empty())
  {
    auto decel_result = zmotion->set_deceleration(axis, _cmd.decelerations[0]);
    if (decel_result) return decel_result;
  }

  // 执行绝对运动
  return zmotion->move_absolute(axis, target_position);
}

std::optional<std::string>
MotionController::MotionControllerPrivate::execute_interpolated(
    const MotionController::MotionCommand& _cmd)
{
  // 先设置各轴的速度和加速度（如果提供）
  for (size_t i = 0; i < _cmd.axes.size(); ++i)
  {
    int axis = _cmd.axes[i];

    if (!_cmd.velocities.empty())
    {
      auto result = zmotion->set_speed(axis, _cmd.velocities[i]);
      if (result) return result;
    }
    if (!_cmd.accelerations.empty())
    {
      auto result = zmotion->set_acceleration(axis, _cmd.accelerations[i]);
      if (result) return result;
    }
    if (!_cmd.decelerations.empty())
    {
      auto result = zmotion->set_deceleration(axis, _cmd.decelerations[i]);
      if (result) return result;
    }
  }

  // 根据插补模式调用相应的函数
  switch (_cmd.interpolation_mode)
  {
    case MotionController::LINEAR:
      return zmotion->move_line_absolute(_cmd.axes, _cmd.positions);

    case MotionController::CIRCULAR:
      return zmotion->move_circular_absolute(_cmd.axes, _cmd.positions,
                                             _cmd.circular_params);

    case MotionController::SPIRAL:
      return zmotion->move_spiral_absolute(_cmd.axes, _cmd.positions,
                                           _cmd.circular_params);

    case MotionController::ECLIPSE:
      return zmotion->move_eclipse_absolute(_cmd.axes, _cmd.positions,
                                            _cmd.circular_params);

    case MotionController::SPHERICAL:
      return zmotion->move_spherical_absolute(_cmd.axes, _cmd.positions,
                                              _cmd.circular_params);

    default:
      return "Unsupported interpolation mode: " +
             std::to_string(static_cast<int>(_cmd.interpolation_mode));
  }
}

std::optional<std::string>
MotionController::MotionControllerPrivate::execute_continuous_trajectory(
    const MotionController::MotionCommand& _cmd)
{
  // 连续轨迹模式：先启动连续模式，然后缓冲所有点

  // 1. 设置各轴速度（如果提供）
  for (size_t i = 0; i < _cmd.axes.size(); ++i)
  {
    int axis = _cmd.axes[i];
    if (!_cmd.velocities.empty())
    {
      auto result = zmotion->set_speed(axis, _cmd.velocities[i]);
      if (result) return result;
    }
    // 连续轨迹通常使用默认加速度
  }

  // 2. 启动连续插补模式
  auto start_result = zmotion->start_continuous();
  if (start_result) return start_result;

  // 3. 缓冲第一个点（起始点）
  auto buffer_result = zmotion->buffer_move(_cmd.axes, _cmd.positions);
  if (buffer_result)
  {
    zmotion->stop_continuous();
    return buffer_result;
  }

  // 4. 等待运动开始并监控
  while (running && !cancel_requested)
  {
    // 检查是否还有后续命令在缓冲中
    {
      std::lock_guard<std::mutex> lock(buffer_mutex);
      if (command_buffer.empty())
      {
        // 没有更多命令，可以停止连续模式
        // 但为了支持流式缓冲，我们等待一段时间看是否有新命令
        // 这里简单处理：等待 100ms，如果没新命令就停止
      }
      else
      {
        // 有后续命令，取出并缓冲
        MotionController::MotionCommand next_cmd = command_buffer.front();
        command_buffer.pop_front();

        // 验证新命令是否与当前连续轨迹兼容（相同轴、相同模式）
        if (next_cmd.motion_type == MotionController::CONTINUOUS_TRAJECTORY &&
            next_cmd.axes == _cmd.axes &&
            next_cmd.interpolation_mode == _cmd.interpolation_mode)
        {
          // 缓冲这个点
          auto next_result =
              zmotion->buffer_move(next_cmd.axes, next_cmd.positions);
          if (next_result)
          {
            std::cerr << "Buffer move failed in continuous trajectory: "
                      << next_result.value() << std::endl;
            zmotion->stop_continuous();
            return next_result;
          }
          continue;  // 继续循环检查下一个命令
        }
        else
        {
          // 遇到不同类型的命令，停止连续模式并处理新命令
          zmotion->stop_continuous();
          // 将新命令重新放回队列头部
          command_buffer.push_front(next_cmd);
          break;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 停止连续模式
  zmotion->stop_continuous();

  return std::nullopt;
}

void MotionController::MotionControllerPrivate::update_status()
{
  // 这个函数可以由外部定时调用，或者由 execute_goal 调用
  // 目前我们不自动更新，而是按需从 zmotion 读取
}

double MotionController::MotionControllerPrivate::calculate_progress(
    const MotionController::MotionCommand& _cmd) const
{
  if (_cmd.axes.empty()) return 0.0;

  double total_distance = 0.0;
  double remaining_distance = 0.0;

  for (size_t i = 0; i < _cmd.axes.size(); ++i)
  {
    int axis = _cmd.axes[i];
    double target = _cmd.positions[i];

    // 获取当前位置
    double current = 0.0;
    if (zmotion)
    {
      auto pos_opt = zmotion->Position(axis);
      if (pos_opt)
      {
        current = pos_opt.value();
      }
    }

    double axis_distance = std::abs(target - current);
    total_distance += axis_distance;
    remaining_distance += axis_distance;
  }

  if (total_distance < 1e-9)
  {
    return 100.0;
  }

  return (total_distance - remaining_distance) / total_distance * 100.0;
}

bool MotionController::MotionControllerPrivate::is_motion_complete(
    const MotionController::MotionCommand& _cmd) const
{
  for (size_t i = 0; i < _cmd.axes.size(); ++i)
  {
    int axis = _cmd.axes[i];
    double target = _cmd.positions[i];

    // 获取当前位置
    double current = 0.0;
    if (zmotion)
    {
      auto pos_opt = zmotion->Position(axis);
      if (pos_opt)
      {
        current = pos_opt.value();
      }
      else
      {
        return false;  // 无法读取位置，认为未完成
      }
    }

    // 检查是否到达目标位置（考虑精度）
    const double POSITION_TOLERANCE = 0.001;  // 1um  tolerance
    if (std::abs(current - target) > POSITION_TOLERANCE)
    {
      return false;
    }

    // 检查轴是否还在运动
    if (zmotion && zmotion->is_axis_moving(axis))
    {
      return false;
    }
  }

  return true;
}

}  // namespace zmc432_driver
