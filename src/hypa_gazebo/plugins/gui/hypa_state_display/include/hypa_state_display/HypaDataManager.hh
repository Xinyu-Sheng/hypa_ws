// 该文件定义了数据管理类，负责 ROS 2 订阅和数据缓存
#ifndef HYPA_STATE_DISPLAY_DATA_MANAGER_HH_
#define HYPA_STATE_DISPLAY_DATA_MANAGER_HH_

#include <QObject>
#include <QString>
#include <QTimer>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hypa_display
{
// 前向声明
class HypaStateDisplayPlugin;

/// IMU 数据结构
struct IMUData
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double linear_accel_x = 0.0;
  double linear_accel_y = 0.0;
  double linear_accel_z = 0.0;
  double angular_vel_x = 0.0;
  double angular_vel_y = 0.0;
  double angular_vel_z = 0.0;
};

/// 日志消息结构
struct LogMessage
{
  std::string level;  // DEBUG, INFO, WARN, ERROR, FATAL
  std::string message;
  std::string node;
  uint32_t timestamp_sec = 0;
};

/// 数据管理和 ROS 2 订阅类
class HypaDataManager : public QObject
{
  Q_OBJECT

  public:
  explicit HypaDataManager(HypaStateDisplayPlugin *_plugin,
                           QObject *_parent = nullptr);
  ~HypaDataManager() override;

  /// 初始化 ROS 2 节点和订阅
  bool initialize(const std::string &_imu_topic,
                  const std::string &_joint_state_topic,
                  const std::string &_log_topic);

  /// 获取当前关节名称列表
  std::vector<std::string> getJointNames() const;

  /// 获取指定关节的位置和加速度
  bool getJointData(const std::string &_name, double &_position,
                    double &_acceleration) const;

  /// 获取最新 IMU 数据
  IMUData getIMUData() const;

  /// 获取最近的日志消息（最多指定数量）
  std::vector<LogMessage> getRecentLogs(size_t _count = 100) const;

  /// 更改订阅话题
  void subscribeToIMUTopic(const std::string &_topic);
  void subscribeToJointStateTopic(const std::string &_topic);

  signals:
  void jointDataUpdated();
  void imuDataUpdated();
  void logMessageReceived(const QString &_message);

  private:
  /// 关节状态消息回调
  void OnJointStateMessage(const sensor_msgs::msg::JointState::SharedPtr _msg);

  /// IMU 消息回调
  void OnIMUMessage(const sensor_msgs::msg::Imu::SharedPtr _msg);

  /// 日志消息回调
  void OnLogMessage(const rcl_interfaces::msg::Log::SharedPtr _msg);

  /// 定时器回调，用于从 ROS 线程抽取数据
  void OnUpdateTimer();

  HypaStateDisplayPlugin *plugin_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  // 关节数据缓存
  std::map<std::string, double> joint_positions_;
  std::map<std::string, double> joint_accelerations_;
  std::vector<std::string> joint_names_;

  // IMU 数据缓存
  IMUData imu_data_;

  // 日志缓存（循环缓冲区，最多 1000 条）
  std::deque<LogMessage> log_messages_;
  static constexpr size_t MAX_LOG_MESSAGES = 1000;

  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;

  // 定时器（用于主线程安全的数据更新）
  std::unique_ptr<QTimer> update_timer_;

  // 线程安全的标志
  bool joint_data_updated_ = false;
  bool imu_data_updated_ = false;
  bool log_data_updated_ = false;
};
}  // namespace hypa_display

#endif  // HYPA_STATE_DISPLAY_DATA_MANAGER_HH_
