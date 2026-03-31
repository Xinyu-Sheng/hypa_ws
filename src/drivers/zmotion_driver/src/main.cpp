#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "zmotion_driver/zmotion_driver_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<zmotion_driver::ZMotionDriverNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
