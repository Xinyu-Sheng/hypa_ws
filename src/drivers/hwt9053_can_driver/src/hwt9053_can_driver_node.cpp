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

#include <string>

namespace hwt9053_can_driver
{

// PIMPL 实现
class HWT9053CANDriverNode::Impl
{
  public:
};

HWT9053CANDriverNode::HWT9053CANDriverNode(const rclcpp::NodeOptions &_options)
    : rclcpp_lifecycle::LifecycleNode("hwt9053_can_driver", _options),
      pimpl_(std::make_unique<Impl>()),
      parser_(std::make_unique<HWT9053Parser>()),
      robot_name_("robot"),
      can_interface_("can0"),
      imu_frame_id_("imu_link"),
      log_debug_(false)
{
  // 声明参数
  this->declare_parameter("robot_name", rclcpp::ParameterValue("robot"));
  this->declare_parameter("can_interface", rclcpp::ParameterValue("can0"));
  this->declare_parameter("imu_frame_id", rclcpp::ParameterValue("imu_link"));
  this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu/data"));
  this->declare_parameter("can_bus_topic",
                          rclcpp::ParameterValue("from_can_bus"));
  this->declare_parameter("log_debug", rclcpp::ParameterValue(false));
}

HWT9053CANDriverNode::~HWT9053CANDriverNode() = default;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_configure(const rclcpp_lifecycle::State &_state)
{
  RCLCPP_INFO(this->get_logger(), "正在配置 HWT9053 CAN 驱动节点");

  // 获取参数
  this->robot_name_ = this->get_parameter("robot_name").as_string();
  this->can_interface_ = this->get_parameter("can_interface").as_string();
  this->imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
  this->log_debug_ = this->get_parameter("log_debug").as_bool();

  std::string imu_topic_name =
      this->get_parameter("imu_topic_name").as_string();
  std::string can_bus_topic = this->get_parameter("can_bus_topic").as_string();

  RCLCPP_INFO(this->get_logger(), "参数设置:");
  RCLCPP_INFO(this->get_logger(), "  robot_name: %s",
              this->robot_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "  can_interface: %s",
              this->can_interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "  imu_frame_id: %s",
              this->imu_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  imu_topic_name: %s",
              imu_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "  can_bus_topic: %s", can_bus_topic.c_str());

  // 创建 CAN 帧订阅（相对话题，自动添加 namespace 和 robot_name）
  auto can_sub_options = rclcpp::SubscriptionOptions();
  can_sub_options.callback_group = nullptr;

  this->can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      can_bus_topic, rclcpp::QoS(10),
      std::bind(&HWT9053CANDriverNode::CanFrameCallback, this,
                std::placeholders::_1),
      can_sub_options);

  // 创建 IMU 发布器（需要在 activate 时创建）
  auto imu_pub_options = rclcpp::PublisherOptions();
  imu_pub_options.qos_overriding_options = rclcpp::QosOverridingOptions();

  this->imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_name, rclcpp::QoS(10), imu_pub_options);

  RCLCPP_INFO(this->get_logger(), "HWT9053 CAN 驱动节点配置完成");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_activate(const rclcpp_lifecycle::State &_state)
{
  RCLCPP_INFO(this->get_logger(), "正在激活 HWT9053 CAN 驱动节点");

  // 激活发布器
  if (this->imu_pub_)
  {
    this->imu_pub_->on_activate();
  }

  RCLCPP_INFO(this->get_logger(),
              "HWT9053 CAN 驱动节点已激活，开始接收 CAN 数据");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_deactivate(const rclcpp_lifecycle::State &_state)
{
  RCLCPP_INFO(this->get_logger(), "正在停用 HWT9053 CAN 驱动节点");

  // 停用发布器
  if (this->imu_pub_)
  {
    this->imu_pub_->on_deactivate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_cleanup(const rclcpp_lifecycle::State &_state)
{
  RCLCPP_INFO(this->get_logger(), "正在清理 HWT9053 CAN 驱动节点");

  this->can_sub_.reset();
  this->imu_pub_.reset();
  this->parser_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_shutdown(const rclcpp_lifecycle::State &_state)
{
  RCLCPP_INFO(this->get_logger(), "正在关闭 HWT9053 CAN 驱动节点");

  this->can_sub_.reset();
  this->imu_pub_.reset();
  this->parser_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void HWT9053CANDriverNode::CanFrameCallback(
    const can_msgs::msg::Frame::SharedPtr _msg)
{
  if (!this->imu_pub_ || !this->imu_pub_->is_activated())
  {
    return;
  }

  // 验证 CAN 帧数据长度
  if (_msg->dlc != 8)
  {
    if (this->log_debug_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "收到非标准长度的 CAN 帧: DLC=%u, ID=0x%x", _msg->dlc,
                  _msg->id);
    }
    return;
  }

  // 验证 CAN ID 是否为 HWT9053 支持的 ID
  uint32_t can_id = _msg->id;
  if (can_id != HWT9053Parser::CAN_ID_TIME &&
      can_id != HWT9053Parser::CAN_ID_ACCEL &&
      can_id != HWT9053Parser::CAN_ID_GYRO &&
      can_id != HWT9053Parser::CAN_ID_ANGLE &&
      can_id != HWT9053Parser::CAN_ID_MAGN)
  {
    if (this->log_debug_)
    {
      RCLCPP_DEBUG(this->get_logger(), "忽略不支持的 CAN ID: 0x%x", can_id);
    }
    return;
  }

  if (this->log_debug_)
  {
    RCLCPP_DEBUG(this->get_logger(), "接收 CAN 帧 ID=0x%x, DLC=%u", _msg->id,
                 _msg->dlc);
  }

  // 转换 CAN 数据为数组
  std::array<uint8_t, 8> data;
  for (size_t i = 0; i < 8 && i < _msg->data.size(); ++i)
  {
    data[i] = _msg->data[i];
  }

  // 解析 CAN 帧
  this->parser_->ParseCANFrame(can_id, data, _msg->dlc);

  // 转换为 IMU 消息并发布
  auto imu_msg = this->parser_->ToIMUMessage();
  imu_msg.header.stamp = _msg->header.stamp;
  imu_msg.header.frame_id = this->imu_frame_id_;

  this->imu_pub_->publish(imu_msg);

  if (this->log_debug_)
  {
    RCLCPP_DEBUG(this->get_logger(),
                 "发布 IMU 消息: accel=[%.3f, %.3f, %.3f] m/s^2, gyro=[%.3f, "
                 "%.3f, %.3f] rad/s",
                 imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                 imu_msg.linear_acceleration.z, imu_msg.angular_velocity.x,
                 imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
  }
}

}  // namespace hwt9053_can_driver

// 主节点入口
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hwt9053_can_driver::HWT9053CANDriverNode)
