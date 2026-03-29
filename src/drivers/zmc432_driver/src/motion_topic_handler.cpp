#include "zmc432_driver/motion_topic_handler.hpp"

namespace zmc432_driver
{

// Private implementation class
class MotionTopicHandler::MotionTopicHandlerPrivate
{
  public:
  MotionTopicHandlerPrivate(
      const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &_node_base,
      const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr
          &_node_topics,
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
          &_node_logging,
      const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr
          &_node_timers,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
          &_node_params,
      const std::shared_ptr<MotionController> &_controller,
      const std::string &_command_topic, const std::string &_status_topic,
      int _status_publish_rate_ms)
      : node_base(_node_base),
        node_topics(_node_topics),
        node_logging(_node_logging),
        node_timers(_node_timers),
        node_params(_node_params),
        controller(_controller),
        command_topic(_command_topic),
        status_topic(_status_topic),
        status_publish_rate_ms(_status_publish_rate_ms),
        running(false)
  {
  }

  ~MotionTopicHandlerPrivate()
  {
    shutdown();
  }

  // 禁止拷贝
  MotionTopicHandlerPrivate(const MotionTopicHandlerPrivate &) = delete;
  MotionTopicHandlerPrivate &operator=(const MotionTopicHandlerPrivate &) =
      delete;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_params;
  std::shared_ptr<MotionController> controller;
  std::string command_topic;
  std::string status_topic;
  int status_publish_rate_ms;
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
      const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg) const;

  hypa_msgs::msg::MotionStatus convert_status_to_msg(
      const MotionController::ControllerStatus &_status) const;
};

// Public interface implementations
MotionTopicHandler::MotionTopicHandler(
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr &_node_base,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr &_node_topics,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr
        &_node_logging,
    const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr &_node_timers,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        &_node_params,
    const std::shared_ptr<MotionController> &_controller,
    const std::string &_command_topic, const std::string &_status_topic,
    int _status_publish_rate_ms)
    : pimpl_(std::make_unique<MotionTopicHandlerPrivate>(
          _node_base, _node_topics, _node_logging, _node_timers, _node_params,
          _controller, _command_topic, _status_topic, _status_publish_rate_ms))
{
}

MotionTopicHandler::~MotionTopicHandler()
{
  shutdown();
}

bool MotionTopicHandler::initialize()
{
  return pimpl_->initialize();
}

void MotionTopicHandler::shutdown()
{
  pimpl_->shutdown();
}

// MotionTopicHandlerPrivate method implementations
bool MotionTopicHandler::MotionTopicHandlerPrivate::initialize()
{
  if (this->running)
  {
    return true;
  }

  // 创建命令订阅器
  auto command_callback =
      [this](const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg)
  { this->handle_command(_msg); };

  this->command_subscription =
      rclcpp::create_subscription<hypa_msgs::msg::MotionCommand>(
          this->node_topics, this->command_topic, 10, command_callback);

  if (!this->command_subscription)
  {
    RCLCPP_ERROR(this->node_logging->get_logger(),
                 "Failed to create command subscription on topic '%s'",
                 this->command_topic.c_str());
    return false;
  }

  // 创建状态发布器
  this->status_publisher =
      rclcpp::create_publisher<hypa_msgs::msg::MotionStatus>(
          this->node_topics, this->status_topic, 10);

  if (!this->status_publisher)
  {
    RCLCPP_ERROR(this->node_logging->get_logger(),
                 "Failed to create status publisher on topic '%s'",
                 this->status_topic.c_str());
    return false;
  }

  // 创建定时器以定期发布状态
  this->status_timer = rclcpp::create_wall_timer(
      std::chrono::milliseconds(this->status_publish_rate_ms),
      [this]() { this->publish_status(); }, nullptr, this->node_base.get(),
      this->node_timers.get());

  if (!this->status_timer)
  {
    RCLCPP_ERROR(this->node_logging->get_logger(),
                 "Failed to create status timer");
    return false;
  }

  this->running = true;
  RCLCPP_INFO(this->node_logging->get_logger(),
              "Motion topic handler initialized (command_topic='%s', "
              "status_topic='%s')",
              this->command_topic.c_str(), this->status_topic.c_str());
  return true;
}

void MotionTopicHandler::MotionTopicHandlerPrivate::shutdown()
{
  if (this->running)
  {
    this->running = false;
    if (this->status_timer)
    {
      this->status_timer->cancel();
    }
    this->status_timer.reset();
    this->command_subscription.reset();
    this->status_publisher.reset();
    RCLCPP_INFO(this->node_logging->get_logger(),
                "Motion topic handler shutdown");
  }
}

void MotionTopicHandler::MotionTopicHandlerPrivate::handle_command(
    const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg)
{
  if (!this->running)
  {
    RCLCPP_WARN(this->node_logging->get_logger(),
                "Received command but node is not running");
    return;
  }

  RCLCPP_INFO(this->node_logging->get_logger(), "Received motion command");

  // 转换消息为命令
  MotionController::MotionCommand cmd = this->convert_msg_to_command(_msg);

  // 验证命令
  if (cmd.axes.empty() || cmd.positions.empty() || cmd.velocities.empty())
  {
    RCLCPP_WARN(
        this->node_logging->get_logger(),
        "Invalid command: axes, positions, and velocities must not be empty");
    return;
  }

  if (cmd.axes.size() != cmd.positions.size() ||
      cmd.axes.size() != cmd.velocities.size())
  {
    RCLCPP_WARN(
        this->node_logging->get_logger(),
        "Invalid command: axes, positions, and velocities array size mismatch");
    return;
  }

  // 提交命令到控制器
  if (!this->controller->queue_motion(cmd))
  {
    RCLCPP_ERROR(this->node_logging->get_logger(),
                 "Failed to queue motion command");
    return;
  }

  RCLCPP_INFO(this->node_logging->get_logger(),
              "Motion command queued successfully");
}

void MotionTopicHandler::MotionTopicHandlerPrivate::publish_status()
{
  if (!this->running)
  {
    return;
  }

  auto status = this->controller->CurrentStatus();
  auto msg = this->convert_status_to_msg(status);

  this->status_publisher->publish(msg);
}

MotionController::MotionCommand
MotionTopicHandler::MotionTopicHandlerPrivate::convert_msg_to_command(
    const hypa_msgs::msg::MotionCommand::ConstSharedPtr _msg) const
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
MotionTopicHandler::MotionTopicHandlerPrivate::convert_status_to_msg(
    const MotionController::ControllerStatus &_status) const
{
  hypa_msgs::msg::MotionStatus msg;

  msg.is_executing = _status.executing;
  msg.progress = _status.progress;

  // 从 map 填充数组
  for (const auto &[axis, axis_status] : _status.axis_statuses)
  {
    msg.axis_numbers.push_back(axis);  // ✅ 修复：保存轴号
    msg.current_positions.push_back(axis_status.position);
    msg.feedback_positions.push_back(axis_status.feedback);
    msg.current_velocities.push_back(axis_status.speed);
    msg.axis_statuses.push_back(axis_status.status_word);
    msg.axis_atypes.push_back(axis_status.type);
    msg.axis_enabled.push_back(axis_status.enabled);
  }

  msg.executing_axis = _status.executing_axes;

  return msg;
}

}  // namespace zmc432_driver
