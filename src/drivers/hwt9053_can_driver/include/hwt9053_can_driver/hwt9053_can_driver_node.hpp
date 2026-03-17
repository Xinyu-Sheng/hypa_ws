#ifndef HWT9053_CAN_DRIVER__HWT9053_CAN_DRIVER_NODE_HPP_
#define HWT9053_CAN_DRIVER__HWT9053_CAN_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "hwt9053_can_driver/hwt9053_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

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
  // 回调函数
  void CanFrameCallback(const can_msgs::msg::Frame::SharedPtr _msg);

  // PIMPL：私有数据实现
  class Impl;
  std::unique_ptr<Impl> pimpl_;

  // 公有成员（直接访问）
  std::unique_ptr<HWT9053Parser> parser_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr
      imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  // 参数
  std::string robot_name_;
  std::string can_interface_;
  std::string imu_frame_id_;
  std::string imu_topic_name_;
  std::string can_bus_topic_;
  bool log_debug_;
  double accel_covariance_;
  double gyro_covariance_;
};

}  // namespace hwt9053_can_driver

#endif  // HWT9053_CAN_DRIVER__HWT9053_CAN_DRIVER_NODE_HPP_
