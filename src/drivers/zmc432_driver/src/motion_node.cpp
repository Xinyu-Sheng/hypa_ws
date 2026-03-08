#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "zmc432_driver/ecat_init.hpp"
#include "zmc432_driver/motion_controller.hpp"
#include "zmc432_driver/motion_topic_node.hpp"

int main(int argc, char **argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建生命周期节点
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("motion_hardware_node");

  // 声明必需参数（多机器人支持）
  node->declare_parameter<std::string>("namespace", "");
  // use_sim_time 由 LifecycleNode 自动声明，无需重复声明
  node->declare_parameter<std::string>("robot_name", "hypa");

  // 声明应用参数
  node->declare_parameter<std::string>("controller_ip", "192.168.0.11");
  node->declare_parameter<double>("default_units", 1.0);
  node->declare_parameter<double>("default_speed", 10.0);
  node->declare_parameter<double>("default_accel", 100.0);
  node->declare_parameter<double>("default_decel", 100.0);
  // 最多配置的轴数量，默认 4
  node->declare_parameter<int>("axis_count", 4);

  // 获取必需参数
  auto namespace_val = node->get_parameter("namespace").as_string();
  auto robot_name = node->get_parameter("robot_name").as_string();
  // use_sim_time 由 ROS 2 框架自动处理，获取后不需要显式使用

  // 获取应用参数
  std::string controller_ip = node->get_parameter("controller_ip").as_string();
  double default_units = node->get_parameter("default_units").as_double();
  double default_speed = node->get_parameter("default_speed").as_double();
  double default_accel = node->get_parameter("default_accel").as_double();
  double default_decel = node->get_parameter("default_decel").as_double();
  int axis_count = node->get_parameter("axis_count").as_int();
  if (axis_count < 0)
  {
    RCLCPP_WARN(node->get_logger(), "axis_count negative (%d), treating as 0", axis_count);
    axis_count = 0;
  }

  RCLCPP_INFO(node->get_logger(), "Motion hardware node starting...");
  RCLCPP_INFO(node->get_logger(), "Namespace: %s, Robot: %s",
              namespace_val.c_str(), robot_name.c_str());
  RCLCPP_INFO(node->get_logger(), "Controller IP: %s", controller_ip.c_str());
  RCLCPP_INFO(node->get_logger(),
              "Default motion parameters: units=%.3f, speed=%.3f, accel=%.3f, "
              "decel=%.3f",
              default_units, default_speed, default_accel, default_decel);
  RCLCPP_INFO(node->get_logger(), "Axis count: %d", axis_count);

  // 创建运动控制器
  auto controller = std::make_shared<zmc432_driver::MotionController>();

  // 初始化控制器（连接 ZMC432）
  auto init_result = controller->initialize(controller_ip);
  if (init_result)
  {
    RCLCPP_FATAL(node->get_logger(),
                 "Failed to initialize motion controller: %s",
                 init_result->c_str());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Connected to ZMC432 at %s",
              controller_ip.c_str());

  // helper pointer used for parameter utilities
  auto node_base = std::dynamic_pointer_cast<rclcpp::Node>(node);

  // EtherCAT 初始化参数
  node->declare_parameter<bool>("perform_ecat_init", false);
  node->declare_parameter<int>("ecat_slot_id", 0);
  node->declare_parameter<int>("ecat_timeout_ms", 5000);
  // some common fields, advanced users can still call API manually
  node->declare_parameter<int>("ecat.drive_axis_start", 0);
  node->declare_parameter<int>("ecat.drive_axis_num", -1);

  if (node->get_parameter("perform_ecat_init").as_bool())
  {
    auto info = zmc432_driver::EcatInitInfo::from_node(node_base.get(), "ecat");
    int slot = node->get_parameter("ecat_slot_id").as_int();
    int tout = node->get_parameter("ecat_timeout_ms").as_int();
    auto err = controller->initialize_bus(info, slot, tout);
    if (err)
    {
      RCLCPP_FATAL(node->get_logger(), "ECAT init failed: %s", err->c_str());
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "EtherCAT bus initialised");
  }

  // 配置默认轴参数（这里可以配置所有可能使用的轴）
  // 实际配置数量由 axis_count 参数决定
  for (int axis = 0; axis < axis_count; ++axis)
  {
    auto config_result = controller->configure_axis(
        axis, default_units, default_speed, default_accel, default_decel);
    if (config_result)
    {
      RCLCPP_WARN(node->get_logger(), "Failed to configure axis %d: %s", axis,
                  config_result->c_str());
    }
    else
    {
      RCLCPP_INFO(
          node->get_logger(),
          "Axis %d configured: units=%.3f, speed=%.3f, accel=%.3f, decel=%.3f",
          axis, default_units, default_speed, default_accel, default_decel);
    }
  }

  // 启动控制器执行线程
  if (!controller->start())
  {
    RCLCPP_FATAL(node->get_logger(),
                 "Failed to start motion controller execution thread");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Motion controller started");

  // 创建 Topic Node（获取 Node 接口）  // 使用之前定义的 node_base
  if (!node_base)
  {
    RCLCPP_FATAL(node->get_logger(),
                 "Failed to get Node interface from LifecycleNode");
    return 1;
  }

  // 声明主题参数
  node->declare_parameter<std::string>("motion_command_topic",
                                       "motion_command");
  node->declare_parameter<std::string>("motion_status_topic", "motion_status");

  // 获取主题名称
  std::string command_topic =
      node->get_parameter("motion_command_topic").as_string();
  std::string status_topic =
      node->get_parameter("motion_status_topic").as_string();

  // 使用机器人名称前缀主题（如果指定了命名空间）
  if (!namespace_val.empty())
  {
    command_topic =
        "/" + namespace_val + "/" + robot_name + "/" + command_topic;
    status_topic = "/" + namespace_val + "/" + robot_name + "/" + status_topic;
  }
  else if (!robot_name.empty() && robot_name != "hypa")
  {
    command_topic = "/" + robot_name + "/" + command_topic;
    status_topic = "/" + robot_name + "/" + status_topic;
  }

  zmc432_driver::MotionTopicNode topic_node(node_base, controller,
                                            command_topic, status_topic);
  if (!topic_node.initialize())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize motion topic node");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Motion topic node ready (command_topic='%s', status_topic='%s')",
              command_topic.c_str(), status_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Listening for motion commands...");

  // 运行 ROS2 spin（对于 LifecycleNode 使用 get_node_base_interface）
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  // 清理
  RCLCPP_INFO(node->get_logger(), "Shutting down motion hardware node");
  topic_node.shutdown();
  controller->stop();

  rclcpp::shutdown();
  return 0;
}
