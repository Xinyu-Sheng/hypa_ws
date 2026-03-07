#ifndef MOTION_TOPIC_NODE_HPP
#define MOTION_TOPIC_NODE_HPP

#include <memory>
#include <mutex>
#include <string>

#include "hypa_msgs/msg/motion_command.hpp"
#include "hypa_msgs/msg/motion_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zmc432_driver/motion_controller.hpp"

namespace zmc432_driver
{

/**
 * @brief ROS 2 Topic 节点，用于运动命令和状态的发布/订阅
 *
 * 订阅 MotionCommand 主题接收运动指令，发布 MotionStatus 主题反馈运动状态。
 * 使用 PIMPL 模式隐藏实现细节。
 */
class MotionTopicNode
{
  public:
  MotionTopicNode(const std::shared_ptr<rclcpp::Node> &_node,
                  std::shared_ptr<MotionController> _controller,
                  const std::string &_command_topic = "motion_command",
                  const std::string &_status_topic = "motion_status");

  ~MotionTopicNode();

  // 禁止拷贝
  MotionTopicNode(const MotionTopicNode &) = delete;
  MotionTopicNode &operator=(const MotionTopicNode &) = delete;

  bool initialize();
  void shutdown();

  private:
  class MotionTopicNodePrivate;
  std::unique_ptr<MotionTopicNodePrivate> pimpl_;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MotionController> controller_;
  std::string command_topic_;
  std::string status_topic_;
};

}  // namespace zmc432_driver

#endif  // MOTION_TOPIC_NODE_HPP
