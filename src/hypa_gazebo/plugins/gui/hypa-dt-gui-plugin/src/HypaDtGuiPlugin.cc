#include "hypa-dt-gui-plugin/HypaDtGuiPlugin.hh"

#include <QtQml/qqml.h>

#include <QDebug>
#include <QQmlContext>
#include <QVariant>
#include <algorithm>

#include <gz/plugin/Register.hh>

#include "hypa-dt-gui-plugin/HypaQCustomPlotItem.hh"

namespace hypa_dt_gui_plugin
{
namespace
{
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
}  // namespace

HypaDtGuiPlugin::HypaDtGuiPlugin()
{
  qmlRegisterSingletonInstance("HypaDtGuiPluginBackend", 1, 0, "Backend", this);
  qmlRegisterType<HypaQCustomPlotItem>("HypaDtGuiPluginBackend", 1, 0,
                                       "HypaQCustomPlotItem");
  qDebug() << "[hypa-dt-gui-plugin] plugin initialized";
}

HypaDtGuiPlugin::~HypaDtGuiPlugin() = default;

void HypaDtGuiPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  Q_UNUSED(_pluginElem);

  if (auto *context = this->Context())
  {
    context->setContextProperty("_HypaDtGuiPlugin", this);
    context->setContextProperty("HypaDtGuiPlugin", this);
  }

  this->imu_selection_model_ = std::make_unique<SelectionModel>(this);
  this->joint_selection_model_ = std::make_unique<SelectionModel>(this);
  this->mag_selection_model_ = std::make_unique<SelectionModel>(this);
  this->data_manager_ = std::make_unique<HypaDataManager>(this);

  connect(this->data_manager_.get(), &HypaDataManager::imuDataUpdated, this,
          &HypaDtGuiPlugin::onImuDataUpdated, Qt::QueuedConnection);
  connect(this->data_manager_.get(), &HypaDataManager::jointDataUpdated, this,
          &HypaDtGuiPlugin::onJointDataUpdated, Qt::QueuedConnection);
  connect(this->data_manager_.get(), &HypaDataManager::magDataUpdated, this,
          &HypaDtGuiPlugin::onMagDataUpdated, Qt::QueuedConnection);

  if (!this->data_manager_->initialize())
  {
    qCritical() << "[hypa-dt-gui-plugin] ROS 2 backend initialization failed";
    return;
  }

  this->onImuDataUpdated();
  this->onJointDataUpdated();
  this->onMagDataUpdated();
}

std::string HypaDtGuiPlugin::Title() const
{
  return "hypa-dt-gui-plugin";
}

QObject *HypaDtGuiPlugin::imuSelectionModel() const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] imuSelectionModel accessed";
  return this->imu_selection_model_.get();
}

QObject *HypaDtGuiPlugin::jointSelectionModel() const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] jointSelectionModel accessed";
  return this->joint_selection_model_.get();
}

QObject *HypaDtGuiPlugin::magSelectionModel() const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] magSelectionModel accessed";
  return this->mag_selection_model_.get();
}

void HypaDtGuiPlugin::onImuDataUpdated()
{
  if (!this->data_manager_ || !this->imu_selection_model_)
    return;

  const QStringList imu_names = this->data_manager_->getImuSourceNames();
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] onImuDataUpdated: imu_names size="
                      << imu_names.size() << imu_names;
  this->imu_selection_model_->syncNames(imu_names, true);

  for (const auto &name : imu_names)
  {
    this->imu_selection_model_->updateLatestText(
        name, this->data_manager_->formatSourceSummary("imu", name));
  }

  emit imuDataUpdated();
}

void HypaDtGuiPlugin::onJointDataUpdated()
{
  if (!this->data_manager_ || !this->joint_selection_model_)
    return;

  const QStringList joint_names = this->data_manager_->getJointNames();
  HYPA_DT_DEBUG_LOG()
      << "[HypaDtGuiPlugin] onJointDataUpdated: joint_names size="
      << joint_names.size() << joint_names;
  this->joint_selection_model_->syncNames(joint_names, true);

  for (const auto &name : joint_names)
  {
    this->joint_selection_model_->updateLatestText(
        name, this->data_manager_->formatSourceSummary("joint", name));
  }

  emit jointDataUpdated();
}

void HypaDtGuiPlugin::onMagDataUpdated()
{
  if (!this->data_manager_ || !this->mag_selection_model_)
    return;

  const QStringList mag_names = this->data_manager_->getMagSourceNames();
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] onMagDataUpdated: mag_names size="
                      << mag_names.size() << mag_names;
  this->mag_selection_model_->syncNames(mag_names, true);

  for (const auto &name : mag_names)
  {
    this->mag_selection_model_->updateLatestText(
        name, this->data_manager_->formatSourceSummary("mag", name));
  }

  emit magDataUpdated();
}

QStringList HypaDtGuiPlugin::getImuSourceNames() const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] getImuSourceNames() called";
  if (this->data_manager_)
    return this->data_manager_->getImuSourceNames();
  return QStringList();
}

QStringList HypaDtGuiPlugin::getJointNames() const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] getJointNames() called";
  if (this->data_manager_)
    return this->data_manager_->getJointNames();
  return QStringList();
}

QStringList HypaDtGuiPlugin::getMagSourceNames() const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] getMagSourceNames() called";
  if (this->data_manager_)
    return this->data_manager_->getMagSourceNames();
  return QStringList();
}

QString HypaDtGuiPlugin::formatSourceSummary(const QString &_kind,
                                             const QString &_source) const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] formatSourceSummary() called:"
                      << _kind << _source;
  if (this->data_manager_)
    return this->data_manager_->formatSourceSummary(_kind, _source);
  return QString();
}

QString HypaDtGuiPlugin::formatSeriesSummary(const QString &_kind,
                                             const QString &_source,
                                             const QVariantList &_metrics) const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] formatSeriesSummary() called:"
                      << _kind << _source << "metrics len:" << _metrics.size();
  if (this->data_manager_)
    return this->data_manager_->formatSeriesSummary(_kind, _source, _metrics);
  return QString();
}

QVariantList HypaDtGuiPlugin::getSeriesPoints(const QString &_kind,
                                              const QString &_source,
                                              const QString &_metric) const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] getSeriesPoints() called:" << _kind
                      << _source << _metric;

  if (!this->data_manager_)
    return QVariantList();

  QVariantList points =
      this->data_manager_->getSeriesPoints(_kind, _source, _metric);
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] getSeriesPoints result size:"
                      << points.size();
  const int to_show = std::min(5, points.size());
  for (int i = 0; i < to_show; ++i)
  {
    const QVariant &v = points.at(i);
    if (v.canConvert<QVariantMap>())
    {
      const QVariantMap m = v.toMap();
      HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] point" << i << ":" << m;
    }
    else
    {
      HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] point" << i << "type"
                          << v.typeName() << v;
    }
  }

  return points;
}

bool HypaDtGuiPlugin::getSeriesDataDelta(
    const QString &_kind, const QString &_source, const QString &_metric,
    uint64_t _lastSequence, QVector<double> &_xValues,
    QVector<double> &_yValues, uint64_t &_latestSequence,
    bool &_resetRequired) const
{
  if (!this->data_manager_)
    return false;

  return this->data_manager_->getSeriesDataDelta(
      _kind, _source, _metric, _lastSequence, _xValues, _yValues,
      _latestSequence, _resetRequired);
}

QVariantList HypaDtGuiPlugin::getImuCovariance(
    const QString &_source, const QString &_covarianceName) const
{
  HYPA_DT_DEBUG_LOG() << "[HypaDtGuiPlugin] getImuCovariance() called:"
                      << _source << _covarianceName;
  if (this->data_manager_)
    return this->data_manager_->getImuCovariance(_source, _covarianceName);
  return QVariantList();
}

}  // namespace hypa_dt_gui_plugin

GZ_ADD_PLUGIN(hypa_dt_gui_plugin::HypaDtGuiPlugin, gz::gui::Plugin)

#undef HYPA_DT_DEBUG_LOG
