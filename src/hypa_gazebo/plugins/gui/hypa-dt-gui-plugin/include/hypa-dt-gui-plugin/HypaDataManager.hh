#ifndef HYPA_DT_GUI_PLUGIN_HYPA_DATA_MANAGER_HH_
#define HYPA_DT_GUI_PLUGIN_HYPA_DATA_MANAGER_HH_

#include <QObject>
#include <QStringList>
#include <QVariantList>
#include <QVector>
#include <array>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hypa_dt_gui_plugin
{
struct ImuSample
{
  double stamp_sec = 0.0;
  double orientation_w = 1.0;
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;
  double angular_velocity_x = 0.0;
  double angular_velocity_y = 0.0;
  double angular_velocity_z = 0.0;
  double linear_acceleration_x = 0.0;
  double linear_acceleration_y = 0.0;
  double linear_acceleration_z = 0.0;
  std::array<double, 9> orientation_covariance{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 9> linear_acceleration_covariance{};
};

struct JointSample
{
  double stamp_sec = 0.0;
  double position = 0.0;
  double velocity = 0.0;
  double effort = 0.0;
};

class HypaDataManager : public QObject
{
  Q_OBJECT

  public:
  explicit HypaDataManager(QObject *_parent = nullptr);
  ~HypaDataManager() override;

  bool initialize();

  Q_INVOKABLE QStringList getImuSourceNames() const;
  Q_INVOKABLE QStringList getJointNames() const;
  Q_INVOKABLE QString formatSourceSummary(const QString &_kind,
                                          const QString &_source) const;
  Q_INVOKABLE QString formatSeriesSummary(const QString &_kind,
                                          const QString &_source,
                                          const QVariantList &_metrics) const;
  Q_INVOKABLE QVariantList getSeriesPoints(const QString &_kind,
                                           const QString &_source,
                                           const QString &_metric) const;
  bool getSeriesDataDelta(const QString &_kind, const QString &_source,
                          const QString &_metric, uint64_t _lastSequence,
                          QVector<double> &_xValues, QVector<double> &_yValues,
                          uint64_t &_latestSequence,
                          bool &_resetRequired) const;
  Q_INVOKABLE QVariantList getImuCovariance(
      const QString &_source, const QString &_covarianceName) const;

  Q_SIGNALS:
  void imuDataUpdated();
  void jointDataUpdated();

  private:
  struct ImuCache
  {
    std::deque<ImuSample> samples;
    uint64_t total_samples = 0;
  };

  struct JointCache
  {
    std::deque<JointSample> samples;
    uint64_t total_samples = 0;
  };

  void handleImuMessage(const sensor_msgs::msg::Imu::ConstSharedPtr &_msg);
  void handleJointStateMessage(
      const sensor_msgs::msg::JointState::ConstSharedPtr &_msg);

  static double messageStampToSec(const builtin_interfaces::msg::Time &_stamp);
  static QString formatValue(double _value);
  static QString metricLabel(const QString &_metric);

  bool getLatestImuValue(const QString &_source, const QString &_metric,
                         double &_value) const;
  bool getLatestJointValue(const QString &_source, const QString &_metric,
                           double &_value) const;
  QVariantList samplesToPoints(const std::deque<ImuSample> &_samples,
                               const QString &_metric) const;
  QVariantList samplesToPoints(const std::deque<JointSample> &_samples,
                               const QString &_metric) const;

  ImuSample createImuSample(
      const sensor_msgs::msg::Imu::ConstSharedPtr &_msg) const;
  JointSample createJointSample(
      const sensor_msgs::msg::JointState::ConstSharedPtr &_msg,
      size_t _index) const;

  mutable std::mutex mutex_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  bool ros_initialized_here_ = false;
  bool running_ = false;

  static constexpr size_t MAX_SAMPLES = 10000;
  static constexpr double HISTORY_WINDOW_SEC = 30.0;
  static constexpr const char *IMU_TOPIC = "/hypa/imu/data";
  static constexpr const char *JOINT_TOPIC = "/hypa/joint_states";

  std::unordered_map<std::string, ImuCache> imu_caches_;
  std::unordered_map<std::string, JointCache> joint_caches_;
  std::vector<std::string> imu_source_order_;
  std::vector<std::string> joint_name_order_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscription_;

  // Lightweight runtime metrics (probes)
  mutable std::atomic<uint64_t> imu_msg_count_{0};
  mutable std::atomic<uint64_t> imu_msg_total_ns_{0};
  mutable std::atomic<uint64_t> imu_msg_lock_ns_{0};

  mutable std::atomic<uint64_t> joint_msg_count_{0};
  mutable std::atomic<uint64_t> joint_msg_total_ns_{0};
  mutable std::atomic<uint64_t> joint_msg_lock_ns_{0};

  mutable std::atomic<uint64_t> get_series_count_{0};
  mutable std::atomic<uint64_t> get_series_total_ns_{0};
  mutable std::atomic<uint64_t> get_series_lock_ns_{0};
};
}  // namespace hypa_dt_gui_plugin

#endif  // HYPA_DT_GUI_PLUGIN_HYPA_DATA_MANAGER_HH_
