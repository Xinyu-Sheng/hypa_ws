#ifndef MOTION_ACTION_SERVER_HPP
#define MOTION_ACTION_SERVER_HPP

#include <memory>
#include <string>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hypa_msgs/action/multi_axis_motion.hpp"
#include "zmc432_driver/motion_controller.hpp"

namespace zmc432_driver
{

using MultiAxisMotion = hypa_msgs::action::MultiAxisMotion;
using GoalHandleMultiAxisMotion =
    rclcpp_action::ServerGoalHandle<MultiAxisMotion>;

/**
 * @brief ROS2 Action 服务器
 *
 * 提供 MultiAxisMotion action 接口，接收运动目标并执行。
 * 支持单轴运动、多轴插补、连续轨迹。
 * 使用 PIMPL 模式隐藏实现细节。
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

  bool initialize();
  void shutdown();

 private:
  class MotionActionServerPrivate;
  std::unique_ptr<MotionActionServerPrivate> pimpl_;

  std::shared_ptr<rclcpp::Node> node_;
  std::string action_name_;
  std::shared_ptr<MotionController> controller_;
};

}  // namespace zmc432_driver

#endif  // MOTION_ACTION_SERVER_HPP
