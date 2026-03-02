#include "hypa_hardware/motion_action_server.hpp"
#include <iostream>
#include <algorithm>

namespace hypa_hardware
{

// Private implementation class
class MotionActionServer::MotionActionServerPrivate
{
 public:
  MotionActionServerPrivate(const std::shared_ptr<rclcpp::Node>& _node,
                            const std::string& _action_name,
                            std::shared_ptr<MotionController> _controller)
      : node(_node),
        action_name(_action_name),
        controller(_controller),
        running(false)
  {
  }

  ~MotionActionServerPrivate() { shutdown(); }

  // 禁止拷贝
  MotionActionServerPrivate(const MotionActionServerPrivate&) = delete;
  MotionActionServerPrivate& operator=(const MotionActionServerPrivate&) =
      delete;

  std::shared_ptr<rclcpp::Node> node;
  std::string action_name;
  std::shared_ptr<MotionController> controller;

  std::shared_ptr<rclcpp_action::Server<MultiAxisMotion>> server;
  std::atomic<bool> running;

  std::thread feedback_thread;
  std::mutex goal_mutex;
  std::shared_ptr<GoalHandleMultiAxisMotion> current_goal;

  // 实现方法
  bool initialize();
  void shutdown();

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& _goal_uuid,
      std::shared_ptr<const MultiAxisMotion::Goal> _goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle);

  void execute_goal(
      const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle);

  void publish_feedback(
      const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle,
      const MotionController::ControllerStatus& _status, double _progress);

  std::optional<std::string> validate_goal(
      const MultiAxisMotion::Goal::ConstSharedPtr& _goal);

  MotionController::MotionCommand convert_goal_to_command(
      const MultiAxisMotion::Goal::ConstSharedPtr& _goal);
};

// Public interface implementations
MotionActionServer::MotionActionServer(
    const std::shared_ptr<rclcpp::Node>& _node, const std::string& _action_name,
    std::shared_ptr<MotionController> _controller)
    : pimpl_(std::make_unique<MotionActionServerPrivate>(_node, _action_name,
                                                         _controller)),
      node_(_node),
      action_name_(_action_name),
      controller_(_controller)
{
}

MotionActionServer::~MotionActionServer() { shutdown(); }

bool MotionActionServer::initialize() { return pimpl_->initialize(); }

void MotionActionServer::shutdown() { pimpl_->shutdown(); }

// MotionActionServerPrivate method implementations
bool MotionActionServer::MotionActionServerPrivate::initialize()
{
  // 创建 action 服务器
  auto goal_callback =
      [this](const rclcpp_action::GoalUUID& _goal_uuid,
             std::shared_ptr<const MultiAxisMotion::Goal> _goal)
  { return this->handle_goal(_goal_uuid, _goal); };

  auto cancel_callback =
      [this](const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle)
  { return this->handle_cancel(_goal_handle); };

  auto accepted_callback =
      [this](const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle)
  { this->handle_accepted(_goal_handle); };

  server = rclcpp_action::create_server<MultiAxisMotion>(
      node, action_name, goal_callback, cancel_callback, accepted_callback);

  if (!server)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to create action server");
    return false;
  }

  running = true;
  RCLCPP_INFO(node->get_logger(), "Action server '%s' created and ready",
              action_name.c_str());
  return true;
}

void MotionActionServer::MotionActionServerPrivate::shutdown()
{
  if (running)
  {
    running = false;
    if (feedback_thread.joinable())
    {
      feedback_thread.join();
    }
  }
}

rclcpp_action::GoalResponse
MotionActionServer::MotionActionServerPrivate::handle_goal(
    const rclcpp_action::GoalUUID& _goal_uuid,
    std::shared_ptr<const MultiAxisMotion::Goal> _goal)
{
  RCLCPP_INFO(node->get_logger(), "Received goal request");

  // 验证目标
  auto validation_result = validate_goal(_goal);
  if (validation_result)
  {
    RCLCPP_WARN(node->get_logger(), "Goal rejected: %s",
                validation_result->c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(node->get_logger(), "Goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MotionActionServer::MotionActionServerPrivate::handle_cancel(
    const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle)
{
  RCLCPP_INFO(node->get_logger(), "Received cancel request");

  // 请求控制器取消当前运动
  controller->cancel_current_motion();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionActionServer::MotionActionServerPrivate::handle_accepted(
    const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle)
{
  // 在独立线程中执行目标，避免阻塞
  std::thread([this, _goal_handle]() { this->execute_goal(_goal_handle); })
      .detach();
}

void MotionActionServer::MotionActionServerPrivate::execute_goal(
    const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle)
{
  RCLCPP_INFO(node->get_logger(), "Executing goal");

  // 转换 goal 为 MotionController 命令
  MotionController::MotionCommand cmd =
      convert_goal_to_command(_goal_handle->get_goal());

  // 提交命令到控制器
  if (!controller->queue_motion(cmd))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to queue motion command");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = false;
    result->error_code = 3;  // 参数错误
    result->error_message =
        "Failed to queue motion command (invalid parameters or controller not "
        "ready)";

    _goal_handle->abort(result);
    return;
  }

  // 启动反馈定时器（50ms 间隔）
  auto feedback_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      [this, _goal_handle]()
      {
        if (!_goal_handle->is_executing())
        {
          return;
        }

        auto status = controller->CurrentStatus();

        // 计算进度
        double progress = 0.0;
        if (!status.axis_statuses.empty())
        {
          double total_progress = 0.0;
          int axis_count = 0;
          for (const auto& [axis, axis_status] : status.axis_statuses)
          {
            if (!axis_status.moving)
            {
              total_progress += 100.0;
            }
            else
            {
              total_progress += 0.0;
            }
            axis_count++;
          }
          if (axis_count > 0)
          {
            progress = total_progress / axis_count;
          }
        }

        publish_feedback(_goal_handle, status, progress);
      });

  // 等待运动完成或取消
  bool completed = false;
  while (rclcpp::ok() && !_goal_handle->is_canceling() && !completed)
  {
    // 检查控制器是否还在执行
    if (!controller->is_executing())
    {
      completed = true;
      break;
    }

    // 检查是否已取消
    if (_goal_handle->is_canceling())
    {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 停止反馈定时器
  feedback_timer->cancel();

  // 处理结果
  if (_goal_handle->is_canceling())
  {
    RCLCPP_INFO(node->get_logger(), "Goal was cancelled");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = false;
    result->error_code = 6;  // 用户取消
    result->error_message = "Motion cancelled by user";

    // 获取最终位置
    auto final_status = controller->CurrentStatus();
    for (const auto& [axis, axis_status] : final_status.axis_statuses)
    {
      result->final_positions.push_back(axis_status.position);
    }

    _goal_handle->canceled(result);
  }
  else if (completed)
  {
    RCLCPP_INFO(node->get_logger(), "Goal completed successfully");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = true;
    result->error_code = 0;
    result->error_message = "Motion completed";

    auto final_status = controller->CurrentStatus();
    for (const auto& [axis, axis_status] : final_status.axis_statuses)
    {
      result->final_positions.push_back(axis_status.position);
    }

    _goal_handle->succeed(result);
  }
  else
  {
    // 其他情况（如错误）
    RCLCPP_ERROR(node->get_logger(), "Goal failed");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = false;
    result->error_code = 99;
    result->error_message = "Unknown error occurred";

    auto final_status = controller->CurrentStatus();
    for (const auto& [axis, axis_status] : final_status.axis_statuses)
    {
      result->final_positions.push_back(axis_status.position);
    }

    _goal_handle->abort(result);
  }
}

void MotionActionServer::MotionActionServerPrivate::publish_feedback(
    const std::shared_ptr<GoalHandleMultiAxisMotion> _goal_handle,
    const MotionController::ControllerStatus& _status, double _progress)
{
  auto feedback = std::make_shared<MultiAxisMotion::Feedback>();

  for (const auto& [axis, axis_status] : _status.axis_statuses)
  {
    feedback->current_positions.push_back(axis_status.position);
    feedback->feedback_positions.push_back(axis_status.feedback);
    feedback->current_velocities.push_back(axis_status.speed);
    feedback->axis_statuses.push_back(axis_status.status_word);
    if (axis_status.moving)
    {
      feedback->executing_axis.push_back(axis);
    }
  }

  feedback->progress = _progress;

  _goal_handle->publish_feedback(feedback);
}

std::optional<std::string>
MotionActionServer::MotionActionServerPrivate::validate_goal(
    const MultiAxisMotion::Goal::ConstSharedPtr& _goal)
{
  if (_goal->axes.empty())
  {
    return "Axes list cannot be empty";
  }

  for (int axis : _goal->axes)
  {
    if (axis < 0 || axis > 31)
    {
      return "Axis number out of range (0-31): " + std::to_string(axis);
    }
  }

  if (_goal->positions.size() != _goal->axes.size())
  {
    return "Positions size does not match axes size";
  }
  if (!_goal->velocities.empty() &&
      _goal->velocities.size() != _goal->axes.size())
  {
    return "Velocities size does not match axes size";
  }
  if (!_goal->accelerations.empty() &&
      _goal->accelerations.size() != _goal->axes.size())
  {
    return "Accelerations size does not match axes size";
  }
  if (!_goal->decelerations.empty() &&
      _goal->decelerations.size() != _goal->axes.size())
  {
    return "Decelerations size does not match axes size";
  }

  if (_goal->motion_type < 1 || _goal->motion_type > 3)
  {
    return "Invalid motion_type (must be 1, 2, or 3)";
  }

  if (_goal->motion_type == 2 || _goal->motion_type == 3)
  {
    if (_goal->interpolation_mode > 4)
    {
      return "Invalid interpolation_mode (must be 0-4)";
    }

    if (_goal->interpolation_mode == 1)
    {
      if (_goal->axes.size() < 2 || _goal->axes.size() > 3)
      {
        return "Circular interpolation requires 2 or 3 axes";
      }
      if (_goal->circular_params.size() < 3)
      {
        return "Circular mode requires at least 3 circular_params for 2D";
      }
    }
    else if (_goal->interpolation_mode == 2)
    {
      if (_goal->axes.size() != 3)
      {
        return "Spiral interpolation requires exactly 3 axes";
      }
      if (_goal->circular_params.size() < 4)
      {
        return "Spiral mode requires at least 4 circular_params: [radius, "
               "pitch, turns, end_angle]";
      }
    }
    else if (_goal->interpolation_mode == 3)
    {
      if (_goal->axes.size() != 2)
      {
        return "Eclipse interpolation requires exactly 2 axes";
      }
      if (_goal->circular_params.size() < 6)
      {
        return "Eclipse mode requires 6 circular_params: [center_x, center_y, "
               "major_axis, minor_axis, start_angle, end_angle]";
      }
    }
    else if (_goal->interpolation_mode == 4)
    {
      if (_goal->axes.size() != 3)
      {
        return "Spherical interpolation requires exactly 3 axes";
      }
      if (_goal->circular_params.size() < 8)
      {
        return "Spherical mode requires 8 circular_params: [center_x, "
               "center_y, center_z, radius, start_theta, end_theta, start_phi, "
               "end_phi]";
      }
    }
  }

  return std::nullopt;
}

MotionController::MotionCommand
MotionActionServer::MotionActionServerPrivate::convert_goal_to_command(
    const MultiAxisMotion::Goal::ConstSharedPtr& _goal)
{
  MotionController::MotionCommand cmd;

  switch (_goal->motion_type)
  {
    case 1:
      cmd.motion_type = MotionController::SINGLE_AXIS;
      break;
    case 2:
      cmd.motion_type = MotionController::INTERPOLATED;
      break;
    case 3:
      cmd.motion_type = MotionController::CONTINUOUS_TRAJECTORY;
      break;
    default:
      cmd.motion_type = MotionController::SINGLE_AXIS;
  }

  cmd.interpolation_mode = static_cast<MotionController::InterpolationMode>(
      _goal->interpolation_mode);

  cmd.axes = _goal->axes;
  cmd.positions = _goal->positions;
  if (!_goal->velocities.empty())
  {
    cmd.velocities = _goal->velocities;
  }
  if (!_goal->accelerations.empty())
  {
    cmd.accelerations = _goal->accelerations;
  }
  if (!_goal->decelerations.empty())
  {
    cmd.decelerations = _goal->decelerations;
  }
  if (!_goal->circular_params.empty())
  {
    cmd.circular_params = _goal->circular_params;
  }
  cmd.wait_time = _goal->wait_time;

  return cmd;
}

}  // namespace hypa_hardware
