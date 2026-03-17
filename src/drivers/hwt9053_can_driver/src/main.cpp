// Copyright 2026 Xinyu Sheng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hwt9053_can_driver/hwt9053_can_driver_node.hpp"

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
