#include "hypa_hardware/motion_controller.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

namespace hypa_hardware
{

MotionController::MotionController()
{
  // 初始化执行线程为未运行状态
}

MotionController::~MotionController() { stop(); }

std::optional<std::string> MotionController::initialize(
    const std::string& _controller_ip)
{
  zmotion_ = std::make_unique<ZMotionWrapper>();

  // 连接控制器
  auto connect_result = zmotion_->connect(_controller_ip);
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
  if (!zmotion_ || !zmotion_->is_connected())
  {
    return "Controller not connected";
  }

  // 设置 units
  auto result = zmotion_->set_units(_axis, _units);
  if (result) return result;

  // 设置速度、加速度、减速度
  result = zmotion_->set_speed(_axis, _speed);
  if (result) return result;

  result = zmotion_->set_acceleration(_axis, _accel);
  if (result) return result;

  result = zmotion_->set_deceleration(_axis, _decel);
  if (result) return result;

  // 缓存配置
  AxisConfig config;
  config.units = _units;
  config.speed = _speed;
  config.acceleration = _accel;
  config.deceleration = _decel;
  config.configured = true;
  axis_configs_[_axis] = config;

  return std::nullopt;
}

bool MotionController::start()
{
  if (running_)
  {
    return true;  // 已经在运行
  }

  if (!zmotion_ || !zmotion_->is_connected())
  {
    return false;
  }

  running_ = true;
  executing_ = false;
  cancel_requested_ = false;

  // 启动执行线程
  execution_thread_ = std::thread(&MotionController::execution_loop, this);

  return true;
}

void MotionController::stop()
{
  if (!running_)
  {
    return;
  }

  running_ = false;
  cancel_requested_ = true;

  // 唤醒执行线程
  buffer_cv_.notify_all();

  // 等待线程结束
  if (execution_thread_.joinable())
  {
    execution_thread_.join();
  }

  // 断开连接
  if (zmotion_)
  {
    zmotion_->disconnect();
  }
}

bool MotionController::queue_motion(const MotionCommand& cmd)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  // 检查命令有效性
  if (cmd.axes.empty())
  {
    return false;
  }
  if (cmd.positions.size() != cmd.axes.size())
  {
    return false;
  }
  if (!cmd.velocities.empty() && cmd.velocities.size() != cmd.axes.size())
  {
    return false;
  }
  if (!cmd.accelerations.empty() && cmd.accelerations.size() != cmd.axes.size())
  {
    return false;
  }
  if (!cmd.decelerations.empty() && cmd.decelerations.size() != cmd.axes.size())
  {
    return false;
  }

  // 检查所有轴是否已配置
  for (int axis : cmd.axes)
  {
    if (axis_configs_.find(axis) == axis_configs_.end())
    {
      return false;
    }
  }

  // 检查连续轨迹模式下的参数
  if (cmd.motion_type == CONTINUOUS_TRAJECTORY)
  {
    // 连续轨迹需要至少一个速度参数
    if (cmd.velocities.empty())
    {
      return false;
    }
  }

  // 入队
  command_buffer_.push_back(cmd);

  // 唤醒执行线程
  buffer_cv_.notify_one();

  return true;
}

MotionController::ControllerStatus MotionController::get_current_status() const
{
  ControllerStatus status;
  status.executing = executing_;
  status.stop_requested = cancel_requested_;

  // 读取所有已配置轴的状态
  for (const auto& [axis, config] : axis_configs_)
  {
    AxisStatus axis_status;

    if (zmotion_)
    {
      axis_status.position = zmotion_->Position(axis).value_or(0.0);
      axis_status.feedback = zmotion_->Feedback(axis).value_or(0.0);
      axis_status.speed = zmotion_->Speed(axis).value_or(0.0);
      auto status_opt = zmotion_->AxisStatus(axis);
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
  cancel_requested_ = true;

  // 立即停止所有轴
  if (zmotion_)
  {
    zmotion_->stop_all();
  }
}

bool MotionController::wait_for_completion(int _timeout_ms)
{
  auto start_time = std::chrono::steady_clock::now();

  while (is_executing())
  {
    if (cancel_requested_)
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

void MotionController::execution_loop()
{
  while (running_)
  {
    MotionCommand cmd;

    // 等待命令或停止信号
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      buffer_cv_.wait(
          lock, [this]() { return !command_buffer_.empty() || !running_; });

      if (!running_)
      {
        break;
      }

      if (command_buffer_.empty())
      {
        continue;
      }

      cmd = command_buffer_.front();
      command_buffer_.pop_front();
      executing_ = true;
      cancel_requested_ = false;
    }

    // 执行命令
    std::optional<std::string> result;

    switch (cmd.motion_type)
    {
      case SINGLE_AXIS:
        result = execute_single_axis(cmd);
        break;

      case INTERPOLATED:
        result = execute_interpolated(cmd);
        break;

      case CONTINUOUS_TRAJECTORY:
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
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      executing_ = false;
    }
  }
}

std::optional<std::string> MotionController::execute_single_axis(
    const MotionCommand& cmd)
{
  if (cmd.axes.size() != 1)
  {
    return "SINGLE_AXIS requires exactly 1 axis";
  }

  int axis = cmd.axes[0];
  double target_position = cmd.positions[0];

  // 如果提供了速度，先设置
  if (!cmd.velocities.empty())
  {
    auto speed_result = zmotion_->set_speed(axis, cmd.velocities[0]);
    if (speed_result) return speed_result;
  }

  // 如果提供了加速度/减速度，先设置
  if (!cmd.accelerations.empty())
  {
    auto accel_result = zmotion_->set_acceleration(axis, cmd.accelerations[0]);
    if (accel_result) return accel_result;
  }
  if (!cmd.decelerations.empty())
  {
    auto decel_result = zmotion_->set_deceleration(axis, cmd.decelerations[0]);
    if (decel_result) return decel_result;
  }

  // 执行绝对运动
  return zmotion_->move_absolute(axis, target_position);
}

std::optional<std::string> MotionController::execute_interpolated(
    const MotionCommand& cmd)
{
  // 先设置各轴的速度和加速度（如果提供）
  for (size_t i = 0; i < cmd.axes.size(); ++i)
  {
    int axis = cmd.axes[i];

    if (!cmd.velocities.empty())
    {
      auto result = zmotion_->set_speed(axis, cmd.velocities[i]);
      if (result) return result;
    }
    if (!cmd.accelerations.empty())
    {
      auto result = zmotion_->set_acceleration(axis, cmd.accelerations[i]);
      if (result) return result;
    }
    if (!cmd.decelerations.empty())
    {
      auto result = zmotion_->set_deceleration(axis, cmd.decelerations[i]);
      if (result) return result;
    }
  }

  // 根据插补模式调用相应的函数
  switch (cmd.interpolation_mode)
  {
    case LINEAR:
      return zmotion_->move_line_absolute(cmd.axes, cmd.positions);

    case CIRCULAR:
      return zmotion_->move_circular_absolute(cmd.axes, cmd.positions,
                                              cmd.circular_params);

    case SPIRAL:
      return zmotion_->move_spiral_absolute(cmd.axes, cmd.positions,
                                            cmd.circular_params);

    case ECLIPSE:
      return zmotion_->move_eclipse_absolute(cmd.axes, cmd.positions,
                                             cmd.circular_params);

    case SPHERICAL:
      return zmotion_->move_spherical_absolute(cmd.axes, cmd.positions,
                                               cmd.circular_params);

    default:
      return "Unsupported interpolation mode: " +
             std::to_string(static_cast<int>(cmd.interpolation_mode));
  }
}

std::optional<std::string> MotionController::execute_continuous_trajectory(
    const MotionCommand& cmd)
{
  // 连续轨迹模式：先启动连续模式，然后缓冲所有点

  // 1. 设置各轴速度（如果提供）
  for (size_t i = 0; i < cmd.axes.size(); ++i)
  {
    int axis = cmd.axes[i];
    if (!cmd.velocities.empty())
    {
      auto result = zmotion_->set_speed(axis, cmd.velocities[i]);
      if (result) return result;
    }
    // 连续轨迹通常使用默认加速度
  }

  // 2. 启动连续插补模式
  auto start_result = zmotion_->start_continuous();
  if (start_result) return start_result;

  // 3. 缓冲第一个点（起始点）
  auto buffer_result = zmotion_->buffer_move(cmd.axes, cmd.positions);
  if (buffer_result)
  {
    zmotion_->stop_continuous();
    return buffer_result;
  }

  // 4. 等待运动开始并监控
  // 注意：这里需要等待用户后续继续缓冲点，或者等待一段时间后自动停止
  // 实际上，连续轨迹应该由外部持续调用 buffer_move 来添加点
  // 当命令队列空时，执行线程应该等待新的 buffer_move 命令

  // 由于我们是从 command_buffer_ 取出的这个命令，我们需要保持连续模式
  // 直到收到停止信号或新的命令

  // 这里我们等待，直到 cancel_requested_ 为 true 或 running_ 为 false
  while (running_ && !cancel_requested_)
  {
    // 检查是否还有后续命令在缓冲中
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (command_buffer_.empty())
      {
        // 没有更多命令，可以停止连续模式
        // 但为了支持流式缓冲，我们等待一段时间看是否有新命令
        // 这里简单处理：等待 100ms，如果没新命令就停止
      }
      else
      {
        // 有后续命令，取出并缓冲
        MotionCommand next_cmd = command_buffer_.front();
        command_buffer_.pop_front();

        // 验证新命令是否与当前连续轨迹兼容（相同轴、相同模式）
        if (next_cmd.motion_type == CONTINUOUS_TRAJECTORY &&
            next_cmd.axes == cmd.axes &&
            next_cmd.interpolation_mode == cmd.interpolation_mode)
        {
          // 缓冲这个点
          auto next_result =
              zmotion_->buffer_move(next_cmd.axes, next_cmd.positions);
          if (next_result)
          {
            std::cerr << "Buffer move failed in continuous trajectory: "
                      << next_result.value() << std::endl;
            zmotion_->stop_continuous();
            return next_result;
          }
          continue;  // 继续循环检查下一个命令
        }
        else
        {
          // 遇到不同类型的命令，停止连续模式并处理新命令
          zmotion_->stop_continuous();
          // 将新命令重新放回队列头部
          command_buffer_.push_front(next_cmd);
          break;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 停止连续模式
  zmotion_->stop_continuous();

  return std::nullopt;
}

void MotionController::update_status()
{
  // 这个函数可以由外部定时调用，或者由 execute_goal 调用
  // 目前我们不自动更新，而是按需从 zmotion_ 读取
}

double MotionController::calculate_progress(const MotionCommand& cmd) const
{
  if (cmd.axes.empty()) return 0.0;

  double total_distance = 0.0;
  double remaining_distance = 0.0;

  for (size_t i = 0; i < cmd.axes.size(); ++i)
  {
    int axis = cmd.axes[i];
    double target = cmd.positions[i];

    // 获取当前位置
    double current = 0.0;
    if (zmotion_)
    {
      auto pos_opt = zmotion_->Position(axis);
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

bool MotionController::is_motion_complete(const MotionCommand& cmd) const
{
  for (size_t i = 0; i < cmd.axes.size(); ++i)
  {
    int axis = cmd.axes[i];
    double target = cmd.positions[i];

    // 获取当前位置
    double current = 0.0;
    if (zmotion_)
    {
      auto pos_opt = zmotion_->Position(axis);
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
    if (zmotion_ && zmotion_->is_axis_moving(axis))
    {
      return false;
    }
  }

  return true;
}

}  // namespace hypa_hardware
