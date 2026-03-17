#ifndef HWT9053_CAN_DRIVER__HWT9053_CAN_DRIVER_NODE_HPP_
#define HWT9053_CAN_DRIVER__HWT9053_CAN_DRIVER_NODE_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace hwt9053_can_driver
{

class HWT9053CANDriverNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
  explicit HWT9053CANDriverNode(
      const rclcpp::NodeOptions &_options = rclcpp::NodeOptions());
  ~HWT9053CANDriverNode();

  // 生命周期回调
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &_state) override;

  private:
  // PIMPL：私有实现
  class Impl;
  std::unique_ptr<Impl> pimpl_;

  // 回调函数（由实现类调用）
  friend class Impl;
};

}  // namespace hwt9053_can_driver

#endif  // HWT9053_CAN_DRIVER__HWT9053_CAN_DRIVER_NODE_HPP_