#ifndef ZMOTION_DRIVER__ZMOTION_DRIVER_NODE_HPP_
#define ZMOTION_DRIVER__ZMOTION_DRIVER_NODE_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace zmotion_driver
{

class ZMotionDriverNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
  explicit ZMotionDriverNode(
      const rclcpp::NodeOptions &_options = rclcpp::NodeOptions());
  ~ZMotionDriverNode() override;

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
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace zmotion_driver

#endif  // ZMOTION_DRIVER__ZMOTION_DRIVER_NODE_HPP_
