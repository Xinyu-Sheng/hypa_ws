// HypaDataManager 实现
#include "hypa_state_display/HypaDataManager.hh"

#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <QDebug>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "hypa_state_display/HypaStateDisplayPlugin.hh"

namespace hypa_display
{
HypaDataManager::HypaDataManager(HypaStateDisplayPlugin *_plugin,
                                 QObject *_parent)
    : QObject(_parent), plugin_(_plugin)
{
  qDebug() << "[HypaDataManager] 构造函数";
}

HypaDataManager::~HypaDataManager()
{
  qDebug() << "[HypaDataManager] 析构函数";
  if (this->update_timer_)
  {
    this->update_timer_->stop();
  }
  if (this->node_)
  {
    try
    {
      rclcpp::shutdown();
    }
    catch (const std::exception &e)
    {
      qDebug() << "[HypaDataManager] 关闭 ROS 时出错:" << e.what();
    }
  }
}

bool HypaDataManager::initialize(const std::string &_imu_topic,
                                 const std::string &_joint_state_topic,
                                 const std::string &_log_topic)
{
  qDebug() << "[HypaDataManager] 初始化 ROS 2 节点";

  try
  {
    // 初始化 ROS 2
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // 创建生命周期节点
    this->node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "hypa_state_display_plugin", rclcpp::NodeOptions());

    // 订阅关节状态
    this->joint_state_sub_ =
        this->node_->create_subscription<sensor_msgs::msg::JointState>(
            _joint_state_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            { this->OnJointStateMessage(msg); });

    qDebug() << "[HypaDataManager] 已订阅关节状态话题:"
             << QString::fromStdString(_joint_state_topic);

    // 订阅 IMU 数据
    this->imu_sub_ = this->node_->create_subscription<sensor_msgs::msg::Imu>(
        _imu_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { this->OnIMUMessage(msg); });

    qDebug() << "[HypaDataManager] 已订阅 IMU 话题:"
             << QString::fromStdString(_imu_topic);

    // 订阅日志消息
    rclcpp::QoS log_qos(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    log_qos.keep_last(100);
    this->log_sub_ = this->node_->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", log_qos,
        [this](const rcl_interfaces::msg::Log::SharedPtr msg)
        { this->OnLogMessage(msg); });

    qDebug() << "[HypaDataManager] 已订阅日志话题: /rosout";

    // 启动生命周期节点
    this->node_->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    this->node_->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    // 创建定时器用于主线程数据更新（100ms 一次）
    this->update_timer_ = std::make_unique<QTimer>(this);
    connect(this->update_timer_.get(), &QTimer::timeout, this,
            &HypaDataManager::OnUpdateTimer);
    this->update_timer_->start(100);

    return true;
  }
  catch (const std::exception &e)
  {
    qDebug() << "[HypaDataManager] 初始化失败:" << e.what();
    return false;
  }
}

std::vector<std::string> HypaDataManager::getJointNames() const
{
  return this->joint_names_;
}

bool HypaDataManager::getJointData(const std::string &_name, double &_position,
                                   double &_acceleration) const
{
  auto pos_it = this->joint_positions_.find(_name);
  auto accel_it = this->joint_accelerations_.find(_name);

  if (pos_it != this->joint_positions_.end() &&
      accel_it != this->joint_accelerations_.end())
  {
    _position = pos_it->second;
    _acceleration = accel_it->second;
    return true;
  }

  return false;
}

IMUData HypaDataManager::getIMUData() const
{
  return this->imu_data_;
}

std::vector<LogMessage> HypaDataManager::getRecentLogs(size_t _count) const
{
  std::vector<LogMessage> result;
  size_t start = this->log_messages_.size() > _count
                     ? this->log_messages_.size() - _count
                     : 0;

  for (size_t i = start; i < this->log_messages_.size(); ++i)
  {
    result.push_back(this->log_messages_[i]);
  }

  return result;
}

void HypaDataManager::subscribeToIMUTopic(const std::string &_topic)
{
  if (this->imu_sub_)
  {
    this->imu_sub_.reset();
  }

  this->imu_sub_ = this->node_->create_subscription<sensor_msgs::msg::Imu>(
      _topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg)
      { this->OnIMUMessage(msg); });

  qDebug() << "[HypaDataManager] IMU 订阅已切换到:"
           << QString::fromStdString(_topic);
}

void HypaDataManager::subscribeToJointStateTopic(const std::string &_topic)
{
  if (this->joint_state_sub_)
  {
    this->joint_state_sub_.reset();
  }

  this->joint_state_sub_ =
      this->node_->create_subscription<sensor_msgs::msg::JointState>(
          _topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::JointState::SharedPtr msg)
          { this->OnJointStateMessage(msg); });

  qDebug() << "[HypaDataManager] 关节状态订阅已切换到:"
           << QString::fromStdString(_topic);
}

void HypaDataManager::OnJointStateMessage(
    const sensor_msgs::msg::JointState::SharedPtr _msg)
{
  if (!_msg)
    return;

  // 更新关节名称列表（仅第一次）
  if (this->joint_names_.empty())
  {
    this->joint_names_ = _msg->name;
  }

  // 更新关节数据
  for (size_t i = 0; i < _msg->name.size(); ++i)
  {
    const auto &name = _msg->name[i];

    if (i < _msg->position.size())
    {
      this->joint_positions_[name] = _msg->position[i];
    }

    // 注意：JointState 消息中没有加速度，这里使用占位符 0.0
    // 如果模型发送自定义消息，需要更新此处
    this->joint_accelerations_[name] = 0.0;
  }

  this->joint_data_updated_ = true;
}

void HypaDataManager::OnIMUMessage(const sensor_msgs::msg::Imu::SharedPtr _msg)
{
  if (!_msg)
    return;

  // 从四元数转换为欧拉角
  tf2::Quaternion quat(_msg->orientation.x, _msg->orientation.y,
                       _msg->orientation.z, _msg->orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  // 更新 IMU 数据
  this->imu_data_.roll = roll * 180.0 / M_PI;  // 转换为度数
  this->imu_data_.pitch = pitch * 180.0 / M_PI;
  this->imu_data_.yaw = yaw * 180.0 / M_PI;

  this->imu_data_.linear_accel_x = _msg->linear_acceleration.x;
  this->imu_data_.linear_accel_y = _msg->linear_acceleration.y;
  this->imu_data_.linear_accel_z = _msg->linear_acceleration.z;

  this->imu_data_.angular_vel_x = _msg->angular_velocity.x;
  this->imu_data_.angular_vel_y = _msg->angular_velocity.y;
  this->imu_data_.angular_vel_z = _msg->angular_velocity.z;

  this->imu_data_updated_ = true;
}

void HypaDataManager::OnLogMessage(
    const rcl_interfaces::msg::Log::SharedPtr _msg)
{
  if (!_msg)
    return;

  LogMessage log;
  switch (_msg->level)
  {
    case rcl_interfaces::msg::Log::DEBUG:
      log.level = "DEBUG";
      break;
    case rcl_interfaces::msg::Log::INFO:
      log.level = "INFO";
      break;
    case rcl_interfaces::msg::Log::WARN:
      log.level = "WARN";
      break;
    case rcl_interfaces::msg::Log::ERROR:
      log.level = "ERROR";
      break;
    case rcl_interfaces::msg::Log::FATAL:
      log.level = "FATAL";
      break;
    default:
      log.level = "UNKNOWN";
  }

  log.message = _msg->msg;
  log.node = _msg->name;
  log.timestamp_sec = _msg->stamp.sec;

  // 添加到循环缓冲区
  if (this->log_messages_.size() >= this->MAX_LOG_MESSAGES)
  {
    this->log_messages_.pop_front();
  }
  this->log_messages_.push_back(log);

  this->log_data_updated_ = true;

  // 发送信号
  QString log_str = QString::fromStdString(log.level) + " [" +
                    QString::fromStdString(log.node) + "] " +
                    QString::fromStdString(log.message);
  emit logMessageReceived(log_str);
}

void HypaDataManager::OnUpdateTimer()
{
  // 在主线程中处理 ROS 数据
  if (this->node_)
  {
    rclcpp::spin_some(this->node_->get_node_base_interface());
  }

  // 检查是否有数据更新，如有则发送信号
  if (this->joint_data_updated_)
  {
    this->joint_data_updated_ = false;
    emit jointDataUpdated();
  }

  if (this->imu_data_updated_)
  {
    this->imu_data_updated_ = false;
    emit imuDataUpdated();
  }
}

}  // namespace hypa_display
