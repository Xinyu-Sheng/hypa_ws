#ifndef MOTION_ACTION_SERVER_HPP
#define MOTION_ACTION_SERVER_HPP

#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hypa_msgs/action/multi_axis_motion.hpp"
#include "hypa_hardware/motion_controller.hpp"

namespace hypa_hardware
{

using MultiAxisMotion = hypa_msgs::action::MultiAxisMotion;
using GoalHandleMultiAxisMotion =
    rclcpp_action::ServerGoalHandle<MultiAxisMotion>;

/**
 * @brief ROS2 Action 服务器
 *
 * 提供 MultiAxisMotion action 接口，接收运动目标并执行。
 * 支持单轴运动、多轴插补、连续轨迹。
 */
class MotionActionServer
{
 public:
  MotionActionServer(const std::shared_ptr<rclcpp::Node>& _node,
                     const std::string& _action_name,
                     std::shared_ptr<MotionController> _controller);

  ~MotionActionServer();

  // 禁止拷贝
  MotionActionServer(const MotionActionServer&) = delete;
  MotionActionServer& operator=(const MotionActionServer&) = delete;

  /**
   * @brief 初始化并启动 action 服务器
   * @return 成功返回 true
   */
  bool initialize();

  /**
   * @brief 停止 action 服务器
   */
  void shutdown();

 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string action_name_;
  std::shared_ptr<MotionController> controller_;

  std::shared_ptr<rclcpp_action::Server<MultiAxisMotion>> server_;
  std::atomic<bool> running_{false};

  std::thread feedback_thread_;
  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleMultiAxisMotion> current_goal_;

  // Action 回调
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& goal_uuid,
      std::shared_ptr<const MultiAxisMotion::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle);

  void handle_accepted(
      const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle);

  /**
   * @brief 执行目标（在独立线程中运行）
   */
  void execute_goal(
      const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle);

  /**
   * @brief 发布反馈
   */
  void publish_feedback(
      const std::shared_ptr<GoalHandleMultiAxisMotion> goal_handle,
      const MotionController::ControllerStatus& status, double progress);

  /**
   * @brief 验证目标参数
   * @param goal 目标
   * @return 如果有效返回空，否则返回错误信息
   */
  std::optional<std::string> validate_goal(
      const MultiAxisMotion::Goal::ConstSharedPtr& goal);

  /**
   * @brief 将 ROS action goal 转换为 MotionController 命令
   */
  MotionController::MotionCommand convert_goal_to_command(
      const MultiAxisMotion::Goal::ConstSharedPtr& goal);
};

}  // namespace hypa_hardware

#endif  // MOTION_ACTION_SERVER_HPP
