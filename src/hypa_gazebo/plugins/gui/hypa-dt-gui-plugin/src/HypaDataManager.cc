#include "hypa-dt-gui-plugin/HypaDataManager.hh"

#include <QDebug>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>

// lifecycle transitions are not used after switching to plain rclcpp::Node

namespace hypa_dt_gui_plugin
{
namespace
{
constexpr const char *kFallbackImuSource = "imu";
constexpr size_t kMetricsLogInterval = 250;

static bool is_debug_enabled()
{
  static const bool enabled = []()
  {
    bool ok = false;
    const int value = qEnvironmentVariableIntValue("HYPA_DT_GUI_DEBUG", &ok);
    return ok && value != 0;
  }();
  return enabled;
}

#define HYPA_DT_DEBUG_LOG() \
  if (!is_debug_enabled())  \
  {                         \
  }                         \
  else                      \
    qDebug()

// Append a single-line metrics record to the metrics log file.
// Open the file once and reuse the handle to reduce open/close overhead.
static void append_metrics_log(const std::string &_line)
{
  static std::mutex metrics_file_mutex;
  std::lock_guard<std::mutex> lock(metrics_file_mutex);
  static std::ofstream ofs("/tmp/hypa_dt_metrics.log", std::ios::app);
  if (!ofs)
    return;
  ofs << _line << std::endl;
}

static std::string make_timestamp()
{
  using namespace std::chrono;
  const auto now = system_clock::now();
  const std::time_t now_c = system_clock::to_time_t(now);
  char buf[64] = {};
  if (std::strftime(buf, sizeof(buf), "%F %T", std::localtime(&now_c)))
    return std::string(buf);
  return std::string();
}
}  // namespace

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
  this->imu_subscription_1_.reset();
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
            { this->handleImuMessage(_msg, IMU_TOPIC); });

    this->imu_subscription_1_ =
        this->node_->create_subscription<sensor_msgs::msg::Imu>(
            IMU_TOPIC_1, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::ConstSharedPtr _msg)
            { this->handleImuMessage(_msg, IMU_TOPIC_1); });

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

    qDebug() << "[HypaDataManager] subscribed to" << IMU_TOPIC << ","
             << IMU_TOPIC_1 << "and" << JOINT_TOPIC;
    return true;
  }
  catch (const std::exception &exception)
  {
    qCritical() << "[HypaDataManager] initialization failed:"
                << exception.what();
    return false;
  }
}

QStringList HypaDataManager::getImuSourceNames() const
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  QStringList result;
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] getImuSourceNames called";
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
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] getJointNames called";
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

  return QStringLiteral("No data");
}

QString HypaDataManager::formatSeriesSummary(const QString &_kind,
                                             const QString &_source,
                                             const QVariantList &_metrics) const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] formatSeriesSummary called:"
                      << _kind << _source;
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
      parts.push_back(this->formatValue(value));
    }
  }

  if (parts.isEmpty())
    return QStringLiteral("No data");

  return parts.join(QStringLiteral("  "));
}

QVariantList HypaDataManager::getSeriesPoints(const QString &_kind,
                                              const QString &_source,
                                              const QString &_metric) const
{
  using namespace std::chrono;
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] getSeriesPoints called:" << _kind
                      << _source << _metric;

  const auto t_total_start = steady_clock::now();
  steady_clock::time_point t_lock_start;
  steady_clock::time_point t_lock_end;

  QVariantList points;

  // Snapshot data while holding the lock, convert to QVariantList after.
  std::deque<ImuSample> imu_samples_copy;
  std::deque<JointSample> joint_samples_copy;

  {
    t_lock_start = steady_clock::now();
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (_kind == "imu")
    {
      const auto cache_it = this->imu_caches_.find(_source.toStdString());
      if (cache_it == this->imu_caches_.end())
      {
        t_lock_end = steady_clock::now();
        const auto t_total_end = steady_clock::now();
        const uint64_t total_ns =
            duration_cast<nanoseconds>(t_total_end - t_total_start).count();
        const uint64_t lock_ns =
            duration_cast<nanoseconds>(t_lock_end - t_lock_start).count();
        const uint64_t c =
            this->get_series_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        this->get_series_total_ns_.fetch_add(total_ns,
                                             std::memory_order_relaxed);
        this->get_series_lock_ns_.fetch_add(lock_ns, std::memory_order_relaxed);
        if ((c % kMetricsLogInterval) == 0)
        {
          HYPA_DT_DEBUG_LOG()
              << "[HypaDataManager] getSeriesPoints avg total ms:"
              << (this->get_series_total_ns_.load() / static_cast<double>(c)) /
                     1e6
              << "avg lock ms:"
              << (this->get_series_lock_ns_.load() / static_cast<double>(c)) /
                     1e6;
          std::ostringstream oss;
          oss << make_timestamp() << " type=get_series"
              << " avg_total_ms="
              << (this->get_series_total_ns_.load() / static_cast<double>(c)) /
                     1e6
              << " avg_lock_ms="
              << (this->get_series_lock_ns_.load() / static_cast<double>(c)) /
                     1e6
              << " count=" << c;
          append_metrics_log(oss.str());
        }
        return QVariantList();
      }

      imu_samples_copy = cache_it->second.samples;
    }
    else if (_kind == "joint")
    {
      const auto cache_it = this->joint_caches_.find(_source.toStdString());
      if (cache_it == this->joint_caches_.end())
      {
        t_lock_end = steady_clock::now();
        const auto t_total_end = steady_clock::now();
        const uint64_t total_ns =
            duration_cast<nanoseconds>(t_total_end - t_total_start).count();
        const uint64_t lock_ns =
            duration_cast<nanoseconds>(t_lock_end - t_lock_start).count();
        const uint64_t c =
            this->get_series_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        this->get_series_total_ns_.fetch_add(total_ns,
                                             std::memory_order_relaxed);
        this->get_series_lock_ns_.fetch_add(lock_ns, std::memory_order_relaxed);
        if ((c % kMetricsLogInterval) == 0)
        {
          HYPA_DT_DEBUG_LOG()
              << "[HypaDataManager] getSeriesPoints avg total ms:"
              << (this->get_series_total_ns_.load() / static_cast<double>(c)) /
                     1e6
              << "avg lock ms:"
              << (this->get_series_lock_ns_.load() / static_cast<double>(c)) /
                     1e6;
        }
        return QVariantList();
      }

      joint_samples_copy = cache_it->second.samples;
    }
    else
    {
      t_lock_end = steady_clock::now();
      const auto t_total_end = steady_clock::now();
      const uint64_t total_ns =
          duration_cast<nanoseconds>(t_total_end - t_total_start).count();
      const uint64_t lock_ns =
          duration_cast<nanoseconds>(t_lock_end - t_lock_start).count();
      const uint64_t c =
          this->get_series_count_.fetch_add(1, std::memory_order_relaxed) + 1;
      this->get_series_total_ns_.fetch_add(total_ns, std::memory_order_relaxed);
      this->get_series_lock_ns_.fetch_add(lock_ns, std::memory_order_relaxed);
      if ((c % kMetricsLogInterval) == 0)
      {
        HYPA_DT_DEBUG_LOG()
            << "[HypaDataManager] getSeriesPoints avg total ms:"
            << (this->get_series_total_ns_.load() / static_cast<double>(c)) /
                   1e6
            << "avg lock ms:"
            << (this->get_series_lock_ns_.load() / static_cast<double>(c)) /
                   1e6;
      }
      return QVariantList();
    }

    t_lock_end = steady_clock::now();
  }

  // Convert snapshot to QVariantList outside lock
  if (_kind == "imu")
    points = this->samplesToPoints(imu_samples_copy, _metric);
  else if (_kind == "joint")
    points = this->samplesToPoints(joint_samples_copy, _metric);

  const auto t_total_end = steady_clock::now();
  const uint64_t total_ns =
      duration_cast<nanoseconds>(t_total_end - t_total_start).count();
  const uint64_t lock_ns =
      duration_cast<nanoseconds>(t_lock_end - t_lock_start).count();
  const uint64_t c =
      this->get_series_count_.fetch_add(1, std::memory_order_relaxed) + 1;
  this->get_series_total_ns_.fetch_add(total_ns, std::memory_order_relaxed);
  this->get_series_lock_ns_.fetch_add(lock_ns, std::memory_order_relaxed);
  if ((c % kMetricsLogInterval) == 0)
  {
    HYPA_DT_DEBUG_LOG()
        << "[HypaDataManager] getSeriesPoints avg total ms:"
        << (this->get_series_total_ns_.load() / static_cast<double>(c)) / 1e6
        << "avg lock ms:"
        << (this->get_series_lock_ns_.load() / static_cast<double>(c)) / 1e6;
  }

  // Print a short summary of returned points for debugging
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] getSeriesPoints result size:"
                      << points.size();
  const int to_show = std::min(5, points.size());
  for (int i = 0; i < to_show; ++i)
  {
    const QVariant &v = points.at(i);
    if (v.canConvert<QVariantMap>())
    {
      const QVariantMap m = v.toMap();
      HYPA_DT_DEBUG_LOG() << "[HypaDataManager] point" << i << ":" << m;
    }
    else
    {
      HYPA_DT_DEBUG_LOG() << "[HypaDataManager] point" << i << "type"
                          << v.typeName() << v;
    }
  }

  return points;
}

bool HypaDataManager::getSeriesDataDelta(
    const QString &_kind, const QString &_source, const QString &_metric,
    uint64_t _lastSequence, QVector<double> &_xValues,
    QVector<double> &_yValues, uint64_t &_latestSequence,
    bool &_resetRequired) const
{
  _xValues.clear();
  _yValues.clear();
  _latestSequence = 0;
  _resetRequired = false;

  std::lock_guard<std::mutex> lock(this->mutex_);

  if (_kind == "imu")
  {
    const auto cache_it = this->imu_caches_.find(_source.toStdString());
    if (cache_it == this->imu_caches_.end())
      return false;

    const auto &samples = cache_it->second.samples;
    const uint64_t total = cache_it->second.total_samples;
    const uint64_t available = static_cast<uint64_t>(samples.size());
    const uint64_t earliest = (total > available) ? (total - available) : 0;

    _latestSequence = total;
    if (available == 0)
      return true;

    uint64_t begin_sequence = _lastSequence;
    if (begin_sequence < earliest)
    {
      begin_sequence = earliest;
      _resetRequired = true;
    }

    if (begin_sequence > total)
      begin_sequence = total;

    const uint64_t begin_index_u64 = begin_sequence - earliest;
    const size_t begin_index = static_cast<size_t>(begin_index_u64);
    if (begin_index >= samples.size())
      return true;

    const size_t append_count = samples.size() - begin_index;
    _xValues.reserve(static_cast<int>(append_count));
    _yValues.reserve(static_cast<int>(append_count));

    for (size_t i = begin_index; i < samples.size(); ++i)
    {
      const auto &sample = samples[i];
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
        return false;

      _xValues.push_back(sample.stamp_sec);
      _yValues.push_back(value);
    }

    return true;
  }

  if (_kind == "joint")
  {
    const auto cache_it = this->joint_caches_.find(_source.toStdString());
    if (cache_it == this->joint_caches_.end())
      return false;

    const auto &samples = cache_it->second.samples;
    const uint64_t total = cache_it->second.total_samples;
    const uint64_t available = static_cast<uint64_t>(samples.size());
    const uint64_t earliest = (total > available) ? (total - available) : 0;

    _latestSequence = total;
    if (available == 0)
      return true;

    uint64_t begin_sequence = _lastSequence;
    if (begin_sequence < earliest)
    {
      begin_sequence = earliest;
      _resetRequired = true;
    }

    if (begin_sequence > total)
      begin_sequence = total;

    const uint64_t begin_index_u64 = begin_sequence - earliest;
    const size_t begin_index = static_cast<size_t>(begin_index_u64);
    if (begin_index >= samples.size())
      return true;

    const size_t append_count = samples.size() - begin_index;
    _xValues.reserve(static_cast<int>(append_count));
    _yValues.reserve(static_cast<int>(append_count));

    for (size_t i = begin_index; i < samples.size(); ++i)
    {
      const auto &sample = samples[i];
      double value = 0.0;
      if (_metric == "position")
        value = sample.position;
      else if (_metric == "velocity")
        value = sample.velocity;
      else if (_metric == "effort")
        value = sample.effort;
      else
        return false;

      _xValues.push_back(sample.stamp_sec);
      _yValues.push_back(value);
    }

    return true;
  }

  return false;
}

QVariantList HypaDataManager::getImuCovariance(
    const QString &_source, const QString &_covarianceName) const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] getImuCovariance called:" << _source
                      << _covarianceName;
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
    const sensor_msgs::msg::Imu::ConstSharedPtr &_msg, const char *_topicSource)
{
  if (!_msg)
  {
    HYPA_DT_DEBUG_LOG() << "[HypaDataManager] handleImuMessage: null message";
    return;
  }

  using namespace std::chrono;
  HYPA_DT_DEBUG_LOG() << "[HypaDataManager] handleImuMessage received:"
                      << QString::fromStdString(_msg->header.frame_id)
                      << "stamp:" << _msg->header.stamp.sec
                      << _msg->header.stamp.nanosec;

  const auto t_total_start = steady_clock::now();
  const ImuSample sample = this->createImuSample(_msg);
  std::string source;
  if (!_msg->header.frame_id.empty())
  {
    source = _msg->header.frame_id;
  }
  else if (_topicSource != nullptr && _topicSource[0] != '\0')
  {
    source = _topicSource;
  }
  else
  {
    source = std::string(kFallbackImuSource);
  }

  steady_clock::time_point t_lock_start;
  steady_clock::time_point t_lock_end;
  {
    t_lock_start = steady_clock::now();
    std::lock_guard<std::mutex> lock(this->mutex_);
    auto &cache = this->imu_caches_[source];
    if (std::find(this->imu_source_order_.begin(),
                  this->imu_source_order_.end(),
                  source) == this->imu_source_order_.end())
    {
      this->imu_source_order_.push_back(source);
    }

    cache.samples.push_back(sample);
    cache.total_samples += 1;
    const double window_start_sec = sample.stamp_sec - HISTORY_WINDOW_SEC;
    while (cache.samples.size() > 1 &&
           cache.samples.front().stamp_sec < window_start_sec)
    {
      cache.samples.pop_front();
    }
    if (cache.samples.size() > MAX_SAMPLES)
      cache.samples.pop_front();
    t_lock_end = steady_clock::now();
  }

  emit imuDataUpdated();

  const auto t_total_end = steady_clock::now();
  const uint64_t total_ns =
      duration_cast<nanoseconds>(t_total_end - t_total_start).count();
  const uint64_t lock_ns =
      duration_cast<nanoseconds>(t_lock_end - t_lock_start).count();
  const uint64_t c =
      this->imu_msg_count_.fetch_add(1, std::memory_order_relaxed) + 1;
  this->imu_msg_total_ns_.fetch_add(total_ns, std::memory_order_relaxed);
  this->imu_msg_lock_ns_.fetch_add(lock_ns, std::memory_order_relaxed);
  if ((c % kMetricsLogInterval) == 0)
  {
    HYPA_DT_DEBUG_LOG()
        << "[HypaDataManager] imu msg avg total ms:"
        << (this->imu_msg_total_ns_.load() / static_cast<double>(c)) / 1e6
        << "avg lock ms:"
        << (this->imu_msg_lock_ns_.load() / static_cast<double>(c)) / 1e6;
    std::ostringstream oss;
    oss << make_timestamp() << " type=imu_msg"
        << " avg_total_ms="
        << (this->imu_msg_total_ns_.load() / static_cast<double>(c)) / 1e6
        << " avg_lock_ms="
        << (this->imu_msg_lock_ns_.load() / static_cast<double>(c)) / 1e6
        << " count=" << c;
    append_metrics_log(oss.str());
  }
}

void HypaDataManager::handleJointStateMessage(
    const sensor_msgs::msg::JointState::ConstSharedPtr &_msg)
{
  if (!_msg)
  {
    HYPA_DT_DEBUG_LOG()
        << "[HypaDataManager] handleJointStateMessage: null message";
    return;
  }

  using namespace std::chrono;
  HYPA_DT_DEBUG_LOG()
      << "[HypaDataManager] handleJointStateMessage received: names="
      << static_cast<int>(_msg->name.size());

  const auto t_total_start = steady_clock::now();
  steady_clock::time_point t_lock_start;
  steady_clock::time_point t_lock_end;
  {
    t_lock_start = steady_clock::now();
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

      const JointSample sample = this->createJointSample(_msg, i);
      cache.samples.push_back(sample);
      cache.total_samples += 1;
      const double window_start_sec = sample.stamp_sec - HISTORY_WINDOW_SEC;
      while (cache.samples.size() > 1 &&
             cache.samples.front().stamp_sec < window_start_sec)
      {
        cache.samples.pop_front();
      }
      if (cache.samples.size() > MAX_SAMPLES)
        cache.samples.pop_front();
    }
    t_lock_end = steady_clock::now();
  }

  emit jointDataUpdated();

  const auto t_total_end = steady_clock::now();
  const uint64_t total_ns =
      duration_cast<nanoseconds>(t_total_end - t_total_start).count();
  const uint64_t lock_ns =
      duration_cast<nanoseconds>(t_lock_end - t_lock_start).count();
  const uint64_t c =
      this->joint_msg_count_.fetch_add(1, std::memory_order_relaxed) + 1;
  this->joint_msg_total_ns_.fetch_add(total_ns, std::memory_order_relaxed);
  this->joint_msg_lock_ns_.fetch_add(lock_ns, std::memory_order_relaxed);
  if ((c % kMetricsLogInterval) == 0)
  {
    HYPA_DT_DEBUG_LOG()
        << "[HypaDataManager] joint msg avg total ms:"
        << (this->joint_msg_total_ns_.load() / static_cast<double>(c)) / 1e6
        << "avg lock ms:"
        << (this->joint_msg_lock_ns_.load() / static_cast<double>(c)) / 1e6;
    std::ostringstream oss;
    oss << make_timestamp() << " type=joint_msg"
        << " avg_total_ms="
        << (this->joint_msg_total_ns_.load() / static_cast<double>(c)) / 1e6
        << " avg_lock_ms="
        << (this->joint_msg_lock_ns_.load() / static_cast<double>(c)) / 1e6
        << " count=" << c;
    append_metrics_log(oss.str());
  }
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
  std::lock_guard<std::mutex> lock(this->mutex_);
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
  std::lock_guard<std::mutex> lock(this->mutex_);
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

#undef HYPA_DT_DEBUG_LOG
