#include "hypa_hardware/motion_action_server.hpp"
#include <iostream>
#include <algorithm>

namespace hypa_hardware
{

MotionActionServer::MotionActionServer(
    const std::shared_ptr<rclcpp::Node>& _node, const std::string& _action_name,
    std::shared_ptr<MotionController> _controller)
    : node_(_node), action_name_(_action_name), controller_(_controller)
{
}

MotionActionServer::~MotionActionServer() { shutdown(); }

bool MotionActionServer::initialize()
{
  // 创建 action 服务器
  auto goal_callback = [this](const rclcpp_action::GoalUUID& goal_uuid,
                              std::shared_ptr<const MultiAxisMotion::Goal> goal)
  { return this->handle_goal(goal_uuid, goal); };

  auto cancel_callback =
      [this](const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle)
  { return this->handle_cancel(goal_handle); };

  auto accepted_callback =
      [this](const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle)
  { this->handle_accepted(goal_handle); };

  server_ = rclcpp_action::create_server<MultiAxisMotion>(
      node_, action_name_, goal_callback, cancel_callback, accepted_callback);

  if (!server_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create action server");
    return false;
  }

  running_ = true;
  RCLCPP_INFO(node_->get_logger(), "Action server '%s' created and ready",
              action_name_.c_str());
  return true;
}

void MotionActionServer::shutdown()
{
  if (running_)
  {
    running_ = false;
    if (feedback_thread_.joinable())
    {
      feedback_thread_.join();
    }
  }
}

rclcpp_action::GoalResponse MotionActionServer::handle_goal(
    const rclcpp_action::GoalUUID& goal_uuid,
    std::shared_ptr<const MultiAxisMotion::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Received goal request");

  // 验证目标
  auto validation_result = validate_goal(goal);
  if (validation_result)
  {
    RCLCPP_WARN(node_->get_logger(), "Goal rejected: %s",
                validation_result->c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(node_->get_logger(), "Goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received cancel request");

  // 请求控制器取消当前运动
  controller_->cancel_current_motion();

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle)
{
  // 在独立线程中执行目标，避免阻塞
  std::thread([this, goal_handle]() { this->execute_goal(goal_handle); })
      .detach();
}

void MotionActionServer::execute_goal(
    const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Executing goal");

  // 转换 goal 为 MotionController 命令
  MotionController::MotionCommand cmd =
      convert_goal_to_command(goal_handle->get_goal());

  // 提交命令到控制器
  if (!controller_->queue_motion(cmd))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to queue motion command");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = false;
    result->error_code = 3;  // 参数错误
    result->error_message =
        "Failed to queue motion command (invalid parameters or controller not "
        "ready)";

    goal_handle->abort(result);
    return;
  }

  // 启动反馈定时器（50ms 间隔）
  auto feedback_timer = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      [this, goal_handle]()
      {
        if (!goal_handle->is_executing())
        {
          return;
        }

        auto status = controller_->get_current_status();

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

        publish_feedback(goal_handle, status, progress);
      });

  // 等待运动完成或取消
  bool completed = false;
  while (rclcpp::ok() && !goal_handle->is_canceling() && !completed)
  {
    // 检查控制器是否还在执行
    if (!controller_->is_executing())
    {
      completed = true;
      break;
    }

    // 检查是否已取消
    if (goal_handle->is_canceling())
    {
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 停止反馈定时器
  feedback_timer->cancel();

  // 处理结果
  if (goal_handle->is_canceling())
  {
    RCLCPP_INFO(node_->get_logger(), "Goal was cancelled");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = false;
    result->error_code = 6;  // 用户取消
    result->error_message = "Motion cancelled by user";

    // 获取最终位置
    auto final_status = controller_->get_current_status();
    for (const auto& [axis, axis_status] : final_status.axis_statuses)
    {
      result->final_positions.push_back(axis_status.position);
    }

    goal_handle->canceled(result);
  }
  else if (completed)
  {
    RCLCPP_INFO(node_->get_logger(), "Goal completed successfully");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = true;
    result->error_code = 0;
    result->error_message = "Motion completed";

    auto final_status = controller_->get_current_status();
    for (const auto& [axis, axis_status] : final_status.axis_statuses)
    {
      result->final_positions.push_back(axis_status.position);
    }

    goal_handle->succeed(result);
  }
  else
  {
    // 其他情况（如错误）
    RCLCPP_ERROR(node_->get_logger(), "Goal failed");

    auto result = std::make_shared<MultiAxisMotion::Result>();
    result->success = false;
    result->error_code = 99;
    result->error_message = "Unknown error occurred";

    auto final_status = controller_->get_current_status();
    for (const auto& [axis, axis_status] : final_status.axis_statuses)
    {
      result->final_positions.push_back(axis_status.position);
    }

    goal_handle->abort(result);
  }
}

void MotionActionServer::publish_feedback(
    const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle,
    const MotionController::ControllerStatus& status, double progress)
{
  auto feedback = std::make_shared<MultiAxisMotion::Feedback>();

  for (const auto& [axis, axis_status] : status.axis_statuses)
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

  feedback->progress = progress;

  goal_handle->publish_feedback(feedback);
}

std::optional<std::string> MotionActionServer::validate_goal(
    const MultiAxisMotion::Goal::ConstSharedPtr& goal)
{
  if (goal->axes.empty())
  {
    return "Axes list cannot be empty";
  }

  for (int axis : goal->axes)
  {
    if (axis < 0 || axis > 31)
    {
      return "Axis number out of range (0-31): " + std::to_string(axis);
    }
  }

  if (goal->positions.size() != goal->axes.size())
  {
    return "Positions size does not match axes size";
  }
  if (!goal->velocities.empty() && goal->velocities.size() != goal->axes.size())
  {
    return "Velocities size does not match axes size";
  }
  if (!goal->accelerations.empty() &&
      goal->accelerations.size() != goal->axes.size())
  {
    return "Accelerations size does not match axes size";
  }
  if (!goal->decelerations.empty() &&
      goal->decelerations.size() != goal->axes.size())
  {
    return "Decelerations size does not match axes size";
  }

  if (goal->motion_type < 1 || goal->motion_type > 3)
  {
    return "Invalid motion_type (must be 1, 2, or 3)";
  }

  if (goal->motion_type == 2 || goal->motion_type == 3)
  {
    if (goal->interpolation_mode > 4)
    {
      return "Invalid interpolation_mode (must be 0-4)";
    }

    if (goal->interpolation_mode == 1)
    {
      if (goal->axes.size() < 2 || goal->axes.size() > 3)
      {
        return "Circular interpolation requires 2 or 3 axes";
      }
      if (goal->circular_params.size() < 3)
      {
        return "Circular mode requires at least 3 circular_params for 2D";
      }
    }
    else if (goal->interpolation_mode == 2)
    {
      if (goal->axes.size() != 3)
      {
        return "Spiral interpolation requires exactly 3 axes";
      }
      if (goal->circular_params.size() < 4)
      {
        return "Spiral mode requires at least 4 circular_params: [radius, "
               "pitch, turns, end_angle]";
      }
    }
    else if (goal->interpolation_mode == 3)
    {
      if (goal->axes.size() != 2)
      {
        return "Eclipse interpolation requires exactly 2 axes";
      }
      if (goal->circular_params.size() < 6)
      {
        return "Eclipse mode requires 6 circular_params: [center_x, center_y, "
               "major_axis, minor_axis, start_angle, end_angle]";
      }
    }
    else if (goal->interpolation_mode == 4)
    {
      if (goal->axes.size() != 3)
      {
        return "Spherical interpolation requires exactly 3 axes";
      }
      if (goal->circular_params.size() < 8)
      {
        return "Spherical mode requires 8 circular_params: [center_x, "
               "center_y, center_z, radius, start_theta, end_theta, start_phi, "
               "end_phi]";
      }
    }
  }

  return std::nullopt;
}

MotionController::MotionCommand MotionActionServer::convert_goal_to_command(
    const MultiAxisMotion::Goal::ConstSharedPtr& goal)
{
  MotionController::MotionCommand cmd;

  switch (goal->motion_type)
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
      goal->interpolation_mode);

  cmd.axes = goal->axes;
  cmd.positions = goal->positions;
  if (!goal->velocities.empty())
  {
    cmd.velocities = goal->velocities;
  }
  if (!goal->accelerations.empty())
  {
    cmd.accelerations = goal->accelerations;
  }
  if (!goal->decelerations.empty())
  {
    cmd.decelerations = goal->decelerations;
  }
  if (!goal->circular_params.empty())
  {
    cmd.circular_params = goal->circular_params;
  }
  cmd.wait_time = goal->wait_time;

  return cmd;
}

}  // namespace hypa_hardware
