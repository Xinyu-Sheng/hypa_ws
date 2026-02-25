#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "hypa_hardware/motion_controller.hpp"
#include "hypa_hardware/motion_action_server.hpp"

int main(int argc, char** argv)
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建生命周期节点
  auto node =
      std::make_shared<rclcpp_lifecycle::LifecycleNode>("motion_hardware_node");

  // 声明必需参数（多机器人支持和仿真时间）
  node->declare_parameter<std::string>("namespace", "");
  node->declare_parameter<bool>("use_sim_time", false);
  node->declare_parameter<std::string>("robot_name", "hypa");

  // 声明应用参数
  node->declare_parameter<std::string>("controller_ip", "192.168.0.11");
  node->declare_parameter<double>("default_units", 1.0);
  node->declare_parameter<double>("default_speed", 10.0);
  node->declare_parameter<double>("default_accel", 100.0);
  node->declare_parameter<double>("default_decel", 100.0);

  // 获取必需参数
  auto namespace_val = node->get_parameter("namespace").as_string();
  auto robot_name = node->get_parameter("robot_name").as_string();
  bool use_sim_time = node->get_parameter("use_sim_time").as_bool();

  // 获取应用参数
  std::string controller_ip = node->get_parameter("controller_ip").as_string();
  double default_units = node->get_parameter("default_units").as_double();
  double default_speed = node->get_parameter("default_speed").as_double();
  double default_accel = node->get_parameter("default_accel").as_double();
  double default_decel = node->get_parameter("default_decel").as_double();

  RCLCPP_INFO(node->get_logger(), "Motion hardware node starting...");
  RCLCPP_INFO(node->get_logger(), "Namespace: %s, Robot: %s, Use Sim Time: %s",
              namespace_val.c_str(), robot_name.c_str(),
              use_sim_time ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "Controller IP: %s", controller_ip.c_str());
  RCLCPP_INFO(node->get_logger(),
              "Default motion parameters: units=%.3f, speed=%.3f, accel=%.3f, "
              "decel=%.3f",
              default_units, default_speed, default_accel, default_decel);

  // 创建运动控制器
  auto controller = std::make_shared<hypa_hardware::MotionController>();

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

  // 配置默认轴参数（这里可以配置所有可能使用的轴）
  // 示例：配置轴 0-3
  for (int axis = 0; axis < 4; ++axis)
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

  // 创建 Action Server（获取 Node 接口）
  auto node_base = std::dynamic_pointer_cast<rclcpp::Node>(node);
  if (!node_base)
  {
    RCLCPP_FATAL(node->get_logger(),
                 "Failed to get Node interface from LifecycleNode");
    return 1;
  }
  hypa_hardware::MotionActionServer action_server(
      node_base, "motion/multi_axis_move", controller);
  if (!action_server.initialize())
  {
    RCLCPP_FATAL(node->get_logger(), "Failed to initialize action server");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Motion action server ready on /motion/multi_axis_move");
  RCLCPP_INFO(node->get_logger(), "Waiting for action goals...");

  // 运行 ROS2 spin（对于 LifecycleNode 使用 get_node_base_interface）
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  // 清理
  RCLCPP_INFO(node->get_logger(), "Shutting down motion hardware node");
  action_server.shutdown();
  controller->stop();

  rclcpp::shutdown();
  return 0;
}
