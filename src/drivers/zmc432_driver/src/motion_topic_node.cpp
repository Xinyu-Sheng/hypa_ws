#include "zmc432_driver/motion_topic_node.hpp"

namespace zmc432_driver
{

// Private implementation class
class MotionTopicNode::MotionTopicNodePrivate
{
 public:
  MotionTopicNodePrivate(const std::shared_ptr<rclcpp::Node>& _node,
                         std::shared_ptr<MotionController> _controller,
                         const std::string& _command_topic,
                         const std::string& _status_topic)
      : node(_node),
        controller(_controller),
        command_topic(_command_topic),
        status_topic(_status_topic),
        running(false)
  {
  }

  ~MotionTopicNodePrivate() { shutdown(); }

  // 禁止拷贝
  MotionTopicNodePrivate(const MotionTopicNodePrivate&) = delete;
  MotionTopicNodePrivate& operator=(const MotionTopicNodePrivate&) = delete;

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<MotionController> controller;
  std::string command_topic;
  std::string status_topic;
  std::atomic<bool> running;

  // ROS 2 发布/订阅
  rclcpp::Subscription<hypa_msgs::msg::MotionCommand>::SharedPtr
      command_subscription;
  rclcpp::Publisher<hypa_msgs::msg::MotionStatus>::SharedPtr status_publisher;
  rclcpp::TimerBase::SharedPtr status_timer;

  // 实现方法
  bool initialize();
  void shutdown();

  void handle_command(const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg);

  void publish_status();

  MotionController::MotionCommand convert_msg_to_command(
      const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg);

  hypa_msgs::msg::MotionStatus convert_status_to_msg(
      const MotionController::ControllerStatus& _status);
};

// Public interface implementations
MotionTopicNode::MotionTopicNode(const std::shared_ptr<rclcpp::Node>& _node,
                                 std::shared_ptr<MotionController> _controller,
                                 const std::string& _command_topic,
                                 const std::string& _status_topic)
    : pimpl_(std::make_unique<MotionTopicNodePrivate>(
          _node, _controller, _command_topic, _status_topic)),
      node_(_node),
      controller_(_controller),
      command_topic_(_command_topic),
      status_topic_(_status_topic)
{
}

MotionTopicNode::~MotionTopicNode() { shutdown(); }

bool MotionTopicNode::initialize() { return pimpl_->initialize(); }

void MotionTopicNode::shutdown() { pimpl_->shutdown(); }

// MotionTopicNodePrivate method implementations
bool MotionTopicNode::MotionTopicNodePrivate::initialize()
{
  // 创建命令订阅器
  auto command_callback =
      [this](const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg)
  { this->handle_command(_msg); };

  command_subscription =
      node->create_subscription<hypa_msgs::msg::MotionCommand>(
          command_topic, 10, command_callback);

  if (!command_subscription)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to create command subscription on topic '%s'",
                 command_topic.c_str());
    return false;
  }

  // 创建状态发布器
  status_publisher =
      node->create_publisher<hypa_msgs::msg::MotionStatus>(status_topic, 10);

  if (!status_publisher)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "Failed to create status publisher on topic '%s'",
                 status_topic.c_str());
    return false;
  }

  // 创建定时器以定期发布状态（50ms 间隔，20Hz）
  status_timer = node->create_wall_timer(std::chrono::milliseconds(50),
                                         [this]() { this->publish_status(); });

  if (!status_timer)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to create status timer");
    return false;
  }

  running = true;
  RCLCPP_INFO(node->get_logger(),
              "Motion topic node initialized (command_topic='%s', "
              "status_topic='%s')",
              command_topic.c_str(), status_topic.c_str());
  return true;
}

void MotionTopicNode::MotionTopicNodePrivate::shutdown()
{
  if (running)
  {
    running = false;
    if (status_timer)
    {
      status_timer->cancel();
    }
    command_subscription.reset();
    status_publisher.reset();
    RCLCPP_INFO(node->get_logger(), "Motion topic node shutdown");
  }
}

void MotionTopicNode::MotionTopicNodePrivate::handle_command(
    const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg)
{
  if (!running)
  {
    RCLCPP_WARN(node->get_logger(), "Received command but node is not running");
    return;
  }

  RCLCPP_INFO(node->get_logger(), "Received motion command");

  // 转换消息为命令
  MotionController::MotionCommand cmd = convert_msg_to_command(_msg);

  // 验证命令
  if (cmd.axes.empty() || cmd.positions.empty() || cmd.velocities.empty())
  {
    RCLCPP_WARN(
        node->get_logger(),
        "Invalid command: axes, positions, and velocities must not be empty");
    return;
  }

  if (cmd.axes.size() != cmd.positions.size() ||
      cmd.axes.size() != cmd.velocities.size())
  {
    RCLCPP_WARN(
        node->get_logger(),
        "Invalid command: axes, positions, and velocities array size mismatch");
    return;
  }

  // 提交命令到控制器
  if (!controller->queue_motion(cmd))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to queue motion command");
    return;
  }

  RCLCPP_INFO(node->get_logger(), "Motion command queued successfully");
}

void MotionTopicNode::MotionTopicNodePrivate::publish_status()
{
  if (!running)
  {
    return;
  }

  auto status = controller->CurrentStatus();
  auto msg = convert_status_to_msg(status);

  status_publisher->publish(msg);
}

MotionController::MotionCommand
MotionTopicNode::MotionTopicNodePrivate::convert_msg_to_command(
    const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg)
{
  MotionController::MotionCommand cmd;

  cmd.axes = _msg->axes;
  cmd.positions = _msg->positions;
  cmd.velocities = _msg->velocities;
  cmd.accelerations = _msg->accelerations;
  cmd.decelerations = _msg->decelerations;
  cmd.circular_params = _msg->circular_params;
  cmd.wait_time = _msg->wait_time;
  cmd.motion_type =
      static_cast<MotionController::MotionType>(_msg->motion_type);
  cmd.interpolation_mode = static_cast<MotionController::InterpolationMode>(
      _msg->interpolation_mode);

  return cmd;
}

hypa_msgs::msg::MotionStatus
MotionTopicNode::MotionTopicNodePrivate::convert_status_to_msg(
    const MotionController::ControllerStatus& _status)
{
  hypa_msgs::msg::MotionStatus msg;

  msg.is_executing = _status.executing;
  msg.progress = _status.progress;

  // 从 map 填充数组
  for (const auto& [axis, axis_status] : _status.axis_statuses)
  {
    msg.current_positions.push_back(axis_status.position);
    msg.feedback_positions.push_back(axis_status.feedback);
    msg.current_velocities.push_back(axis_status.speed);
    msg.axis_statuses.push_back(axis_status.status_word);
    msg.axis_enabled.push_back(axis_status.enabled);
    msg.executing_axis.push_back(axis);
  }

  return msg;
}

}  // namespace zmc432_driver
