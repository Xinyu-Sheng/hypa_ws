#include "hypa-dt-gui-plugin/HypaDataManager.hh"

#include <QDebug>
#include <algorithm>
#include <cmath>

// lifecycle transitions are not used after switching to plain rclcpp::Node

namespace hypa_dt_gui_plugin
{
namespace
{
constexpr const char *kFallbackImuSource = "imu";
}

HypaDataManager::HypaDataManager(QObject *_parent) : QObject(_parent)
{
}

HypaDataManager::~HypaDataManager()
{
  this->running_ = false;

  if (this->executor_)
    this->executor_->cancel();

  if (this->executor_thread_.joinable())
    this->executor_thread_.join();

  this->imu_subscription_.reset();
  this->joint_state_subscription_.reset();
  this->executor_.reset();
  this->node_.reset();

  if (this->ros_initialized_here_ && rclcpp::ok())
  {
    try
    {
      rclcpp::shutdown();
    }
    catch (const std::exception &)
    {
    }
  }
}

bool HypaDataManager::initialize()
{
  if (this->node_)
    return true;

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
    this->ros_initialized_here_ = true;
  }

  try
  {
    // create a regular rclcpp::Node instead of a lifecycle node
    this->node_ = std::make_shared<rclcpp::Node>("hypa_dt_gui_plugin",
                                                 rclcpp::NodeOptions());

    this->imu_subscription_ =
        this->node_->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr _msg)
            { this->handleImuMessage(_msg); });

    this->joint_state_subscription_ =
        this->node_->create_subscription<sensor_msgs::msg::JointState>(
            JOINT_TOPIC, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::ConstSharedPtr _msg)
            { this->handleJointStateMessage(_msg); });

    this->executor_ =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->executor_->add_node(this->node_->get_node_base_interface());

    this->running_ = true;
    this->executor_thread_ = std::thread(
        [this]()
        {
          if (this->executor_)
            this->executor_->spin();
        });

    qDebug() << "[HypaDataManager] 已订阅" << IMU_TOPIC << "和" << JOINT_TOPIC;
    return true;
  }
  catch (const std::exception &exception)
  {
    qCritical() << "[HypaDataManager] 初始化失败:" << exception.what();
    return false;
  }
}

QStringList HypaDataManager::getImuSourceNames() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  QStringList result;
  for (const auto &name : this->imu_source_order_)
  {
    result.push_back(QString::fromStdString(name));
  }
  return result;
}

QStringList HypaDataManager::getJointNames() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  QStringList result;
  for (const auto &name : this->joint_name_order_)
  {
    result.push_back(QString::fromStdString(name));
  }
  return result;
}

QString HypaDataManager::formatSourceSummary(const QString &_kind,
                                             const QString &_source) const
{
  if (_kind == "imu")
  {
    QVariantList metrics;
    metrics << QStringLiteral("orientation_w")
            << QStringLiteral("orientation_x")
            << QStringLiteral("orientation_y")
            << QStringLiteral("orientation_z")
            << QStringLiteral("angular_velocity_x")
            << QStringLiteral("angular_velocity_y")
            << QStringLiteral("angular_velocity_z")
            << QStringLiteral("linear_acceleration_x")
            << QStringLiteral("linear_acceleration_y")
            << QStringLiteral("linear_acceleration_z");
    return this->formatSeriesSummary(_kind, _source, metrics);
  }

  if (_kind == "joint")
  {
    QVariantList metrics;
    metrics << QStringLiteral("position") << QStringLiteral("velocity")
            << QStringLiteral("effort");
    return this->formatSeriesSummary(_kind, _source, metrics);
  }

  return QStringLiteral("无数据");
}

QString HypaDataManager::formatSeriesSummary(const QString &_kind,
                                             const QString &_source,
                                             const QVariantList &_metrics) const
{
  QStringList parts;

  for (const auto &metric_variant : _metrics)
  {
    const QString metric = metric_variant.toString();
    double value = 0.0;
    bool found = false;

    if (_kind == "imu")
    {
      found = this->getLatestImuValue(_source, metric, value);
    }
    else if (_kind == "joint")
    {
      found = this->getLatestJointValue(_source, metric, value);
    }

    if (found)
    {
      parts.push_back(QString("%1=%2").arg(this->metricLabel(metric),
                                           this->formatValue(value)));
    }
  }

  if (parts.isEmpty())
    return QStringLiteral("无数据");

  return parts.join(QStringLiteral("  "));
}

QVariantList HypaDataManager::getSeriesPoints(const QString &_kind,
                                              const QString &_source,
                                              const QString &_metric) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (_kind == "imu")
  {
    const auto cache_it = this->imu_caches_.find(_source.toStdString());
    if (cache_it == this->imu_caches_.end())
      return QVariantList();

    return this->samplesToPoints(cache_it->second.samples, _metric);
  }

  if (_kind == "joint")
  {
    const auto cache_it = this->joint_caches_.find(_source.toStdString());
    if (cache_it == this->joint_caches_.end())
      return QVariantList();

    return this->samplesToPoints(cache_it->second.samples, _metric);
  }

  return QVariantList();
}

QVariantList HypaDataManager::getImuCovariance(
    const QString &_source, const QString &_covarianceName) const
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  const auto cache_it = this->imu_caches_.find(_source.toStdString());
  if (cache_it == this->imu_caches_.end() || cache_it->second.samples.empty())
    return QVariantList();

  const auto &latest = cache_it->second.samples.back();
  const std::array<double, 9> *covariance = nullptr;

  if (_covarianceName == "orientation_covariance")
    covariance = &latest.orientation_covariance;
  else if (_covarianceName == "angular_velocity_covariance")
    covariance = &latest.angular_velocity_covariance;
  else if (_covarianceName == "linear_acceleration_covariance")
    covariance = &latest.linear_acceleration_covariance;

  if (!covariance)
    return QVariantList();

  QVariantList result;
  result.reserve(9);
  for (double value : *covariance)
  {
    result.push_back(value);
  }
  return result;
}

void HypaDataManager::handleImuMessage(
    const sensor_msgs::msg::Imu::ConstSharedPtr &_msg)
{
  if (!_msg)
    return;

  const ImuSample sample = this->createImuSample(_msg);
  const std::string source = _msg->header.frame_id.empty()
                                 ? std::string(kFallbackImuSource)
                                 : _msg->header.frame_id;

  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    auto &cache = this->imu_caches_[source];
    if (std::find(this->imu_source_order_.begin(),
                  this->imu_source_order_.end(),
                  source) == this->imu_source_order_.end())
    {
      this->imu_source_order_.push_back(source);
    }

    cache.samples.push_back(sample);
    if (cache.samples.size() > MAX_SAMPLES)
      cache.samples.pop_front();
  }

  emit imuDataUpdated();
}

void HypaDataManager::handleJointStateMessage(
    const sensor_msgs::msg::JointState::ConstSharedPtr &_msg)
{
  if (!_msg)
    return;

  {
    std::lock_guard<std::mutex> lock(this->mutex_);

    for (size_t i = 0; i < _msg->name.size(); ++i)
    {
      const std::string name = _msg->name[i];
      auto &cache = this->joint_caches_[name];
      if (std::find(this->joint_name_order_.begin(),
                    this->joint_name_order_.end(),
                    name) == this->joint_name_order_.end())
      {
        this->joint_name_order_.push_back(name);
      }

      cache.samples.push_back(this->createJointSample(_msg, i));
      if (cache.samples.size() > MAX_SAMPLES)
        cache.samples.pop_front();
    }
  }

  emit jointDataUpdated();
}

double HypaDataManager::messageStampToSec(
    const builtin_interfaces::msg::Time &_stamp)
{
  return static_cast<double>(_stamp.sec) +
         static_cast<double>(_stamp.nanosec) * 1e-9;
}

QString HypaDataManager::formatValue(double _value)
{
  return QString::number(_value, 'f', 3);
}

QString HypaDataManager::metricLabel(const QString &_metric)
{
  if (_metric.endsWith("_x"))
    return QStringLiteral("x");
  if (_metric.endsWith("_y"))
    return QStringLiteral("y");
  if (_metric.endsWith("_z"))
    return QStringLiteral("z");
  if (_metric.endsWith("_w"))
    return QStringLiteral("w");
  return _metric;
}

bool HypaDataManager::getLatestImuValue(const QString &_source,
                                        const QString &_metric,
                                        double &_value) const
{
  const auto cache_it = this->imu_caches_.find(_source.toStdString());
  if (cache_it == this->imu_caches_.end() || cache_it->second.samples.empty())
    return false;

  const auto &sample = cache_it->second.samples.back();

  if (_metric == "orientation_w")
    _value = sample.orientation_w;
  else if (_metric == "orientation_x")
    _value = sample.orientation_x;
  else if (_metric == "orientation_y")
    _value = sample.orientation_y;
  else if (_metric == "orientation_z")
    _value = sample.orientation_z;
  else if (_metric == "angular_velocity_x")
    _value = sample.angular_velocity_x;
  else if (_metric == "angular_velocity_y")
    _value = sample.angular_velocity_y;
  else if (_metric == "angular_velocity_z")
    _value = sample.angular_velocity_z;
  else if (_metric == "linear_acceleration_x")
    _value = sample.linear_acceleration_x;
  else if (_metric == "linear_acceleration_y")
    _value = sample.linear_acceleration_y;
  else if (_metric == "linear_acceleration_z")
    _value = sample.linear_acceleration_z;
  else
    return false;

  return true;
}

bool HypaDataManager::getLatestJointValue(const QString &_source,
                                          const QString &_metric,
                                          double &_value) const
{
  const auto cache_it = this->joint_caches_.find(_source.toStdString());
  if (cache_it == this->joint_caches_.end() || cache_it->second.samples.empty())
    return false;

  const auto &sample = cache_it->second.samples.back();

  if (_metric == "position")
    _value = sample.position;
  else if (_metric == "velocity")
    _value = sample.velocity;
  else if (_metric == "effort")
    _value = sample.effort;
  else
    return false;

  return true;
}

QVariantList HypaDataManager::samplesToPoints(
    const std::deque<ImuSample> &_samples, const QString &_metric) const
{
  QVariantList points;
  points.reserve(static_cast<int>(_samples.size()));

  for (const auto &sample : _samples)
  {
    double value = 0.0;
    if (_metric == "orientation_w")
      value = sample.orientation_w;
    else if (_metric == "orientation_x")
      value = sample.orientation_x;
    else if (_metric == "orientation_y")
      value = sample.orientation_y;
    else if (_metric == "orientation_z")
      value = sample.orientation_z;
    else if (_metric == "angular_velocity_x")
      value = sample.angular_velocity_x;
    else if (_metric == "angular_velocity_y")
      value = sample.angular_velocity_y;
    else if (_metric == "angular_velocity_z")
      value = sample.angular_velocity_z;
    else if (_metric == "linear_acceleration_x")
      value = sample.linear_acceleration_x;
    else if (_metric == "linear_acceleration_y")
      value = sample.linear_acceleration_y;
    else if (_metric == "linear_acceleration_z")
      value = sample.linear_acceleration_z;
    else
      continue;

    QVariantMap point;
    point["x"] = sample.stamp_sec;
    point["y"] = value;
    points.push_back(point);
  }

  return points;
}

QVariantList HypaDataManager::samplesToPoints(
    const std::deque<JointSample> &_samples, const QString &_metric) const
{
  QVariantList points;
  points.reserve(static_cast<int>(_samples.size()));

  for (const auto &sample : _samples)
  {
    double value = 0.0;
    if (_metric == "position")
      value = sample.position;
    else if (_metric == "velocity")
      value = sample.velocity;
    else if (_metric == "effort")
      value = sample.effort;
    else
      continue;

    QVariantMap point;
    point["x"] = sample.stamp_sec;
    point["y"] = value;
    points.push_back(point);
  }

  return points;
}

ImuSample HypaDataManager::createImuSample(
    const sensor_msgs::msg::Imu::ConstSharedPtr &_msg) const
{
  ImuSample sample;
  sample.stamp_sec = messageStampToSec(_msg->header.stamp);
  if (sample.stamp_sec <= 0.0)
  {
    if (this->node_)
      sample.stamp_sec = this->node_->now().seconds();
  }

  sample.orientation_x = _msg->orientation.x;
  sample.orientation_y = _msg->orientation.y;
  sample.orientation_z = _msg->orientation.z;
  sample.orientation_w = _msg->orientation.w;

  sample.angular_velocity_x = _msg->angular_velocity.x;
  sample.angular_velocity_y = _msg->angular_velocity.y;
  sample.angular_velocity_z = _msg->angular_velocity.z;

  sample.linear_acceleration_x = _msg->linear_acceleration.x;
  sample.linear_acceleration_y = _msg->linear_acceleration.y;
  sample.linear_acceleration_z = _msg->linear_acceleration.z;

  for (size_t i = 0; i < 9; ++i)
  {
    sample.orientation_covariance[i] = _msg->orientation_covariance[i];
    sample.angular_velocity_covariance[i] =
        _msg->angular_velocity_covariance[i];
    sample.linear_acceleration_covariance[i] =
        _msg->linear_acceleration_covariance[i];
  }

  return sample;
}

JointSample HypaDataManager::createJointSample(
    const sensor_msgs::msg::JointState::ConstSharedPtr &_msg,
    size_t _index) const
{
  JointSample sample;
  sample.stamp_sec = messageStampToSec(_msg->header.stamp);
  if (sample.stamp_sec <= 0.0)
  {
    if (this->node_)
      sample.stamp_sec = this->node_->now().seconds();
  }

  if (_index < _msg->position.size())
    sample.position = _msg->position[_index];
  if (_index < _msg->velocity.size())
    sample.velocity = _msg->velocity[_index];
  if (_index < _msg->effort.size())
    sample.effort = _msg->effort[_index];

  return sample;
}

}  // namespace hypa_dt_gui_plugin
