#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "zmc432_driver/ecat_init.hpp"
#include "zmc432_driver/motion_controller.hpp"
#include "zmc432_driver/motion_topic_node.hpp"

namespace zmc432_driver
{

class MotionHardwareNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
  explicit MotionHardwareNode(const std::string &node_name)
      : LifecycleNode(node_name)
  {
    // 声明必需参数（多机器人支持）
    // use_sim_time 由 LifecycleNode 自动声明，无需重复声明
    // namespace 和 robot_name 由 launch 文件通过 PushRosNamespace 处理

    // 声明应用参数
    this->declare_parameter<std::string>("controller_ip", "192.168.0.11");
    this->declare_parameter<std::string>("motion_command_topic",
                                         "motion_command");
    this->declare_parameter<std::string>("motion_status_topic",
                                         "motion_status");
    this->declare_parameter<double>("default_units", 1.0);
    this->declare_parameter<double>("default_speed", 10.0);
    this->declare_parameter<double>("default_accel", 100.0);
    this->declare_parameter<double>("default_decel", 100.0);
    this->declare_parameter<bool>("reset_position_on_configure", true);
    this->declare_parameter<int>("axis_count", 1);
    this->declare_parameter<int>("status_publish_rate_ms", 50);

    // EtherCAT 相关参数
    this->declare_parameter<bool>("perform_ecat_init", false);
    this->declare_parameter<int>("ecat_slot_id", 0);
    this->declare_parameter<int>("ecat_timeout_ms", 5000);
  }

  ~MotionHardwareNode() override = default;

  protected:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn;

    // 读取参数
    // namespace 和 robot_name 由 launch 文件通过 PushRosNamespace
    // 处理，无需在节点中读取
    controller_ip_ = this->get_parameter("controller_ip").as_string();
    command_topic_ = this->get_parameter("motion_command_topic").as_string();
    status_topic_ = this->get_parameter("motion_status_topic").as_string();
    default_units_ = this->get_parameter("default_units").as_double();
    default_speed_ = this->get_parameter("default_speed").as_double();
    default_accel_ = this->get_parameter("default_accel").as_double();
    default_decel_ = this->get_parameter("default_decel").as_double();
    reset_position_on_configure_ =
        this->get_parameter("reset_position_on_configure").as_bool();
    axis_count_ = this->get_parameter("axis_count").as_int();
    status_publish_rate_ms_ =
        this->get_parameter("status_publish_rate_ms").as_int();

    // ✅ 修复#5: 读取use_sim_time参数
    bool use_sim_time = this->get_parameter("use_sim_time").as_bool();
    if (use_sim_time)
    {
      RCLCPP_INFO(this->get_logger(),
                  "use_sim_time enabled - using simulation clock");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                  "use_sim_time disabled - using system clock");
    }

    // ✅ 修复#6: 参数范围验证
    if (axis_count_ < 0 || axis_count_ > 32)
    {
      RCLCPP_WARN(this->get_logger(),
                  "axis_count=%d out of range [0,32], clamping to valid value",
                  axis_count_);
      axis_count_ = std::max(0, std::min(axis_count_, 32));
    }

    if (default_units_ <= 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "default_units=%.2f is invalid, using default 1.0",
                  default_units_);
      default_units_ = 1.0;
    }

    if (default_speed_ <= 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "default_speed=%.2f is invalid, using default 10.0",
                  default_speed_);
      default_speed_ = 10.0;
    }

    if (default_accel_ <= 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "default_accel=%.2f is invalid, using default 100.0",
                  default_accel_);
      default_accel_ = 100.0;
    }

    if (default_decel_ <= 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "default_decel=%.2f is invalid, using default 100.0",
                  default_decel_);
      default_decel_ = 100.0;
    }

    RCLCPP_INFO(this->get_logger(), "Motion hardware node starting...");
    RCLCPP_INFO(this->get_logger(), "Controller IP: %s",
                controller_ip_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Default motion parameters: units=%.3f, speed=%.3f, "
                "accel=%.3f, decel=%.3f",
                default_units_, default_speed_, default_accel_, default_decel_);
    RCLCPP_INFO(this->get_logger(), "Axis count: %d", axis_count_);
    RCLCPP_INFO(this->get_logger(), "reset_position_on_configure: %s",
                reset_position_on_configure_ ? "true" : "false");

    controller_ = std::make_shared<MotionController>();

    // 初始化控制器（连接 ZMC432）
    auto init_result = controller_->initialize(controller_ip_);
    if (init_result)
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to initialize motion controller: %s",
                   init_result->c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "Connected to ZMC432 at %s",
                controller_ip_.c_str());

    // EtherCAT 初始化（可选）
    if (this->get_parameter("perform_ecat_init").as_bool())
    {
      auto info = EcatInitInfo::from_node(this->get_node_parameters_interface(),
                                          "ecat");
      int slot = this->get_parameter("ecat_slot_id").as_int();
      int tout = this->get_parameter("ecat_timeout_ms").as_int();
      auto err = controller_->initialize_bus(info, slot, tout);
      if (err)
      {
        RCLCPP_FATAL(this->get_logger(), "ECAT init failed: %s", err->c_str());
        return CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(this->get_logger(), "EtherCAT bus initialised");
    }

    // 轴参数配置
    for (int axis = 0; axis < axis_count_; ++axis)
    {
      std::string axis_param_prefix = "axis_" + std::to_string(axis) + ".";

      // 设置默认值
      double units = default_units_;
      double speed = default_speed_;
      double accel = default_accel_;
      double decel = default_decel_;

      // 声明并获取单轴特定参数，如果未在 YAML 中定义，则使用全局默认值
      this->declare_parameter<double>(axis_param_prefix + "units",
                                      default_units_);
      this->declare_parameter<double>(axis_param_prefix + "speed",
                                      default_speed_);
      this->declare_parameter<double>(axis_param_prefix + "accel",
                                      default_accel_);
      this->declare_parameter<double>(axis_param_prefix + "decel",
                                      default_decel_);

      this->get_parameter(axis_param_prefix + "units", units);
      this->get_parameter(axis_param_prefix + "speed", speed);
      this->get_parameter(axis_param_prefix + "accel", accel);
      this->get_parameter(axis_param_prefix + "decel", decel);

      auto config_result = controller_->configure_axis(
          axis, units, speed, accel, decel, reset_position_on_configure_);
      if (config_result)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to configure axis %d: %s", axis,
                    config_result->c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(),
                    "Axis %d configured: units=%.3f, speed=%.3f, accel=%.3f, "
                    "decel=%.3f",
                    axis, units, speed, accel, decel);
      }
    }

    // ✅ topic 命名空间由 launch 层通过 PushRosNamespace
    // 处理，无需在节点中手动添加前缀

    topic_node_ = std::make_unique<MotionTopicNode>(
        this->get_node_base_interface(), this->get_node_topics_interface(),
        this->get_node_logging_interface(), this->get_node_timers_interface(),
        this->get_node_parameters_interface(), controller_, command_topic_,
        status_topic_, status_publish_rate_ms_);

    if (!topic_node_->initialize())
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to initialize motion topic node");
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Motion topic node ready (command_topic='%s', status_topic='%s')",
        command_topic_.c_str(), status_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Listening for motion commands...");

    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!controller_)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Cannot activate: controller not initialized");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::FAILURE;
    }

    if (!controller_->start())
    {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to start motion controller execution thread");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(this->get_logger(), "Motion controller started");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (topic_node_)
    {
      topic_node_->shutdown();
    }
    if (controller_)
    {
      controller_->stop();
    }
    RCLCPP_INFO(this->get_logger(), "Motion hardware node deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    topic_node_.reset();
    controller_.reset();
    RCLCPP_INFO(this->get_logger(), "Motion hardware node cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    if (topic_node_)
    {
      topic_node_->shutdown();
    }
    if (controller_)
    {
      controller_->stop();
    }
    RCLCPP_INFO(this->get_logger(), "Motion hardware node shutting down");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  private:
  std::string controller_ip_;
  std::string command_topic_;
  std::string status_topic_;
  double default_units_ = 1.0;
  double default_speed_ = 10.0;
  double default_accel_ = 100.0;
  double default_decel_ = 100.0;
  bool reset_position_on_configure_ = true;
  int axis_count_ = 1;
  int status_publish_rate_ms_ = 50;

  std::shared_ptr<MotionController> controller_;
  std::unique_ptr<MotionTopicNode> topic_node_;
};

}  // namespace zmc432_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<zmc432_driver::MotionHardwareNode>(
      "motion_hardware_node");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
