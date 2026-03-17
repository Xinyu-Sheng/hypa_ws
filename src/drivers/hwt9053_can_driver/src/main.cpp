#include <memory>

#include "hwt9053_can_driver/hwt9053_can_driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int _argc, char *_argv[])
{
  rclcpp::init(_argc, _argv);

  rclcpp::NodeOptions options;
  auto node =
      std::make_shared<hwt9053_can_driver::HWT9053CANDriverNode>(options);

  // 创建执行器并添加节点
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // 运行节点（生命周期管理由 launch 文件或外部管理器处理）
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
