#include "hwt9053_can_driver/hwt9053_can_driver_node.hpp"

#include <array>
#include <memory>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "hwt9053_can_driver/hwt9053_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace hwt9053_can_driver
{

// PIMPL 实现类
class HWT9053CANDriverNode::Impl
{
  public:
  explicit Impl();
  ~Impl() = default;

  // 生命周期方法实现
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(HWT9053CANDriverNode *_node,
               const rclcpp_lifecycle::State &_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(HWT9053CANDriverNode *_node,
              const rclcpp_lifecycle::State &_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(HWT9053CANDriverNode *_node,
                const rclcpp_lifecycle::State &_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(HWT9053CANDriverNode *_node,
             const rclcpp_lifecycle::State &_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(HWT9053CANDriverNode *_node,
              const rclcpp_lifecycle::State &_state);

  // CAN 帧回调处理
  void CanFrameCallback(HWT9053CANDriverNode *_node,
                        const can_msgs::msg::Frame::SharedPtr _msg);

  private:
  // 私有成员变量
  std::unique_ptr<HWT9053Parser> parser_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr
      imu_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
      sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

  // 参数
  std::string robot_name_;
  // CAN 设备接口名（例如 "can0"）。
  // 注意：本节点仅将该参数作为配置/文档提示保存并打印，
  // 本节点本身不会打开或管理 socketCAN。底层 CAN 接口
  // 的实际打开/转发应由外部节点（例如 `ros2_socketcan`）或系统
  // 配置负责，数据通过 `can_bus_topic` 订阅接收。
  std::string can_interface_;
  std::string imu_frame_id_;
  std::string imu_topic_name_;
  std::string can_bus_topic_;
  bool log_debug_;
  double accel_covariance_;
  double gyro_covariance_;
};

// Impl 构造函数实现
HWT9053CANDriverNode::Impl::Impl()
    : parser_(std::make_unique<HWT9053Parser>()),
      robot_name_("robot"),
      can_interface_("can0"),
      imu_frame_id_("imu_link"),
      log_debug_(false),
      accel_covariance_(1.18e-9),
      gyro_covariance_(2.39e-7)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::Impl::on_configure(HWT9053CANDriverNode *_node,
                                         const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(_node->get_logger(), "正在配置 HWT9053 CAN 驱动节点");

  if (!this->parser_)
  {
    this->parser_ = std::make_unique<HWT9053Parser>();
  }
  else
  {
    this->parser_->Reset();
  }

  // 获取参数
  this->robot_name_ = _node->get_parameter("robot_name").as_string();
  this->can_interface_ = _node->get_parameter("can_interface").as_string();
  this->imu_frame_id_ = _node->get_parameter("imu_frame_id").as_string();
  this->log_debug_ = _node->get_parameter("log_debug").as_bool();
  this->accel_covariance_ =
      _node->get_parameter("accel_covariance").as_double();
  this->gyro_covariance_ = _node->get_parameter("gyro_covariance").as_double();

  // 数据过期阈值
  uint32_t data_timeout_ms =
      static_cast<uint32_t>(_node->get_parameter("data_timeout_ms").as_int());

  // 设置协方差到解析器
  this->parser_->SetAccelCovariance(this->accel_covariance_);
  this->parser_->SetGyroCovariance(this->gyro_covariance_);
  // 设置数据过期阈值到解析器
  this->parser_->SetDataTimeoutMs(data_timeout_ms);

  this->imu_topic_name_ = _node->get_parameter("imu_topic_name").as_string();
  this->can_bus_topic_ = _node->get_parameter("can_bus_topic").as_string();

  RCLCPP_INFO(_node->get_logger(), "参数设置:");
  RCLCPP_INFO(_node->get_logger(), "  robot_name: %s",
              this->robot_name_.c_str());
  // 仅记录所选 CAN 接口名；本节点不负责打开该接口
  RCLCPP_INFO(_node->get_logger(),
              "  can_interface: %s (仅记录，不用于打开设备)",
              this->can_interface_.c_str());
  RCLCPP_INFO(_node->get_logger(), "  imu_frame_id: %s",
              this->imu_frame_id_.c_str());
  RCLCPP_INFO(_node->get_logger(), "  imu_topic_name: %s",
              this->imu_topic_name_.c_str());
  RCLCPP_INFO(_node->get_logger(), "  can_bus_topic: %s",
              this->can_bus_topic_.c_str());
  RCLCPP_INFO(_node->get_logger(), "  accel_covariance: %.6e",
              this->accel_covariance_);
  RCLCPP_INFO(_node->get_logger(), "  gyro_covariance: %.6e",
              this->gyro_covariance_);
  RCLCPP_INFO(_node->get_logger(), "  data_timeout_ms: %u", data_timeout_ms);

  RCLCPP_INFO(_node->get_logger(), "HWT9053 CAN 驱动节点配置完成");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::Impl::on_activate(HWT9053CANDriverNode *_node,
                                        const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(_node->get_logger(), "正在激活 HWT9053 CAN 驱动节点");

  // 订阅 CAN 帧（只在激活态接收数据）
  if (!this->can_sub_)
  {
    auto can_sub_options = rclcpp::SubscriptionOptions();
    can_sub_options.callback_group = nullptr;

    this->can_sub_ = _node->create_subscription<can_msgs::msg::Frame>(
        this->can_bus_topic_, rclcpp::QoS(10),
        [this, _node](const can_msgs::msg::Frame::SharedPtr _msg)
        { this->CanFrameCallback(_node, _msg); }, can_sub_options);
  }

  // 在激活阶段创建 Publisher（生命周期规范）
  if (!this->imu_pub_)
  {
    auto imu_pub_options = rclcpp::PublisherOptions();
    imu_pub_options.qos_overriding_options = rclcpp::QosOverridingOptions();

    this->imu_pub_ = _node->create_publisher<sensor_msgs::msg::Imu>(
        this->imu_topic_name_, rclcpp::QoS(10), imu_pub_options);
  }

  if (!this->mag_pub_)
  {
    auto mag_pub_options = rclcpp::PublisherOptions();
    mag_pub_options.qos_overriding_options = rclcpp::QosOverridingOptions();

    this->mag_pub_ = _node->create_publisher<sensor_msgs::msg::MagneticField>(
        this->imu_topic_name_ + "_mag", rclcpp::QoS(10), mag_pub_options);
  }

  // 激活发布器
  if (this->imu_pub_)
  {
    this->imu_pub_->on_activate();
  }

  if (this->mag_pub_)
  {
    this->mag_pub_->on_activate();
  }

  RCLCPP_INFO(_node->get_logger(),
              "HWT9053 CAN 驱动节点已激活，开始接收 CAN 数据");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::Impl::on_deactivate(HWT9053CANDriverNode *_node,
                                          const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(_node->get_logger(), "正在停用 HWT9053 CAN 驱动节点");

  // 停用发布器
  if (this->imu_pub_)
  {
    this->imu_pub_->on_deactivate();
  }

  if (this->mag_pub_)
  {
    this->mag_pub_->on_deactivate();
  }

  // 停止接收 CAN 帧
  this->can_sub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::Impl::on_cleanup(HWT9053CANDriverNode *_node,
                                       const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(_node->get_logger(), "正在清理 HWT9053 CAN 驱动节点");

  this->can_sub_.reset();
  this->imu_pub_.reset();
  this->mag_pub_.reset();
  if (this->parser_)
  {
    this->parser_->Reset();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::Impl::on_shutdown(HWT9053CANDriverNode *_node,
                                        const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(_node->get_logger(), "正在关闭 HWT9053 CAN 驱动节点");

  this->can_sub_.reset();
  this->imu_pub_.reset();
  this->mag_pub_.reset();
  if (this->parser_)
  {
    this->parser_->Reset();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void HWT9053CANDriverNode::Impl::CanFrameCallback(
    HWT9053CANDriverNode *_node, const can_msgs::msg::Frame::SharedPtr _msg)
{
  if (!this->parser_ || !this->imu_pub_ || !this->imu_pub_->is_activated())
  {
    return;
  }

  // 验证 CAN 帧数据长度
  if (_msg->dlc != 8)
  {
    if (this->log_debug_)
    {
      RCLCPP_WARN(_node->get_logger(),
                  "收到非标准长度的 CAN 帧: DLC=%u, ID=0x%x", _msg->dlc,
                  _msg->id);
    }
    return;
  }
  // 先把 CAN 数据拷贝到固定数组
  std::array<uint8_t, 8> can_data{};
  for (size_t i = 0; i < 8 && i < _msg->data.size(); ++i)
  {
    can_data[i] = _msg->data[i];
  }

  // 强制采用 WIT 封装：payload[0] 必须为 0x55，payload[1] 为 TYPE
  if (can_data[0] != 0x55)
  {
    if (this->log_debug_)
    {
      RCLCPP_DEBUG(_node->get_logger(), "忽略非 WIT 封装的 CAN 帧: CAN ID=0x%x",
                   _msg->id);
    }
    return;
  }

  uint8_t payload_type = can_data[1];
  if (payload_type != HWT9053Parser::WIT_TYPE_TIME &&
      payload_type != HWT9053Parser::WIT_TYPE_ACCEL &&
      payload_type != HWT9053Parser::WIT_TYPE_GYRO &&
      payload_type != HWT9053Parser::WIT_TYPE_ANGLE &&
      payload_type != HWT9053Parser::WIT_TYPE_MAGN)
  {
    if (this->log_debug_)
    {
      RCLCPP_DEBUG(_node->get_logger(), "忽略不支持的 WIT TYPE: 0x%x",
                   payload_type);
    }
    return;
  }

  if (this->log_debug_)
  {
    RCLCPP_DEBUG(_node->get_logger(), "接收 WIT CAN 帧 TYPE=0x%x, ID=0x%x",
                 payload_type, _msg->id);
  }

  // 解析 CAN 帧
  bool parse_success = this->parser_->ParseCANFrame(can_data, _msg->dlc);

  if (!parse_success)
  {
    RCLCPP_WARN(_node->get_logger(),
                "CAN 帧解析失败: ID=0x%x, DLC=%u (期望 DLC=8)", _msg->id,
                _msg->dlc);
    return;
  }

  auto parsed_data = this->parser_->GetData();
  if (!parsed_data.accel_valid || !parsed_data.gyro_valid)
  {
    return;
  }

  // 转换为 IMU 消息并发布
  auto imu_msg = this->parser_->ToIMUMessage();
  imu_msg.header.stamp = _msg->header.stamp;
  imu_msg.header.frame_id = this->imu_frame_id_;

  this->imu_pub_->publish(imu_msg);

  // 获取磁场数据（线程安全）
  auto mag_data = this->parser_->GetMagneticFieldData();

  // 发布磁场消息（如果数据有效）
  if (mag_data.mag_valid && this->mag_pub_ && this->mag_pub_->is_activated())
  {
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = _msg->header.stamp;
    mag_msg.header.frame_id = this->imu_frame_id_;
    mag_msg.magnetic_field.x = mag_data.mag_x;
    mag_msg.magnetic_field.y = mag_data.mag_y;
    mag_msg.magnetic_field.z = mag_data.mag_z;

    // 磁场协方差矩阵 (T^2)
    constexpr double MAG_STDDEV_T = 13.0e-9;
    constexpr double MAG_COVARIANCE = MAG_STDDEV_T * MAG_STDDEV_T;
    for (int i = 0; i < 9; ++i)
    {
      if (i % 4 == 0)
      {
        mag_msg.magnetic_field_covariance[i] = MAG_COVARIANCE;
      }
      else
      {
        mag_msg.magnetic_field_covariance[i] = 0.0;
      }
    }

    this->mag_pub_->publish(mag_msg);
  }

  if (this->log_debug_)
  {
    RCLCPP_DEBUG(_node->get_logger(),
                 "发布 IMU 消息: accel=[%.3f, %.3f, %.3f] m/s^2, gyro=[%.3f, "
                 "%.3f, %.3f] rad/s",
                 imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
                 imu_msg.linear_acceleration.z, imu_msg.angular_velocity.x,
                 imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
  }
}

// 公共接口实现
HWT9053CANDriverNode::HWT9053CANDriverNode(const rclcpp::NodeOptions &_options)
    : rclcpp_lifecycle::LifecycleNode("hwt9053_can_driver", _options),
      pimpl_(std::make_unique<Impl>())
{
  // 声明参数
  this->declare_parameter("robot_name", rclcpp::ParameterValue("robot"));
  this->declare_parameter("can_interface", rclcpp::ParameterValue("can0"));
  this->declare_parameter("imu_frame_id", rclcpp::ParameterValue("imu_link"));
  this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu/data"));
  this->declare_parameter("can_bus_topic",
                          rclcpp::ParameterValue("from_can_bus"));
  this->declare_parameter("log_debug", rclcpp::ParameterValue(false));
  // 参数语义：`*_covariance` 为方差 (variance)，单位分别为 (m/s^2)^2 和
  // (rad/s)^2
  this->declare_parameter("accel_covariance", rclcpp::ParameterValue(1.18e-9));
  this->declare_parameter("gyro_covariance", rclcpp::ParameterValue(2.39e-7));
  // 数据过期阈值（毫秒）
  this->declare_parameter("data_timeout_ms", rclcpp::ParameterValue(500));
}

HWT9053CANDriverNode::~HWT9053CANDriverNode() = default;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_configure(const rclcpp_lifecycle::State &_state)
{
  return this->pimpl_->on_configure(this, _state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_activate(const rclcpp_lifecycle::State &_state)
{
  return this->pimpl_->on_activate(this, _state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_deactivate(const rclcpp_lifecycle::State &_state)
{
  return this->pimpl_->on_deactivate(this, _state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_cleanup(const rclcpp_lifecycle::State &_state)
{
  return this->pimpl_->on_cleanup(this, _state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HWT9053CANDriverNode::on_shutdown(const rclcpp_lifecycle::State &_state)
{
  return this->pimpl_->on_shutdown(this, _state);
}

}  // namespace hwt9053_can_driver

// 主节点入口
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hwt9053_can_driver::HWT9053CANDriverNode)