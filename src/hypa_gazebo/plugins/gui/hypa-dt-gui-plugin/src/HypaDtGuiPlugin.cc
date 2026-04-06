#include "hypa-dt-gui-plugin/HypaDtGuiPlugin.hh"

#include <QtQml/qqml.h>

#include <QDebug>
#include <QQmlContext>
#include <QVariant>
#include <algorithm>

#include <gz/plugin/Register.hh>

namespace hypa_dt_gui_plugin
{
HypaDtGuiPlugin::HypaDtGuiPlugin()
{
  qmlRegisterSingletonInstance("HypaDtGuiPluginBackend", 1, 0, "Backend", this);
  qDebug() << "[hypa-dt-gui-plugin] 插件初始化";
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
  this->data_manager_ = std::make_unique<HypaDataManager>(this);

  connect(this->data_manager_.get(), &HypaDataManager::imuDataUpdated, this,
          &HypaDtGuiPlugin::onImuDataUpdated, Qt::QueuedConnection);
  connect(this->data_manager_.get(), &HypaDataManager::jointDataUpdated, this,
          &HypaDtGuiPlugin::onJointDataUpdated, Qt::QueuedConnection);

  if (!this->data_manager_->initialize())
  {
    qCritical() << "[hypa-dt-gui-plugin] ROS 2 后端初始化失败";
    return;
  }

  this->onImuDataUpdated();
  this->onJointDataUpdated();
}

std::string HypaDtGuiPlugin::Title() const
{
  return "hypa-dt-gui-plugin";
}

QObject *HypaDtGuiPlugin::imuSelectionModel() const
{
  qDebug() << "[HypaDtGuiPlugin] imuSelectionModel accessed";
  return this->imu_selection_model_.get();
}

QObject *HypaDtGuiPlugin::jointSelectionModel() const
{
  qDebug() << "[HypaDtGuiPlugin] jointSelectionModel accessed";
  return this->joint_selection_model_.get();
}

void HypaDtGuiPlugin::onImuDataUpdated()
{
  if (!this->data_manager_ || !this->imu_selection_model_)
    return;

  const QStringList imu_names = this->data_manager_->getImuSourceNames();
  qDebug() << "[HypaDtGuiPlugin] onImuDataUpdated: imu_names size="
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
  qDebug() << "[HypaDtGuiPlugin] onJointDataUpdated: joint_names size="
           << joint_names.size() << joint_names;
  this->joint_selection_model_->syncNames(joint_names, true);

  for (const auto &name : joint_names)
  {
    this->joint_selection_model_->updateLatestText(
        name, this->data_manager_->formatSourceSummary("joint", name));
  }

  emit jointDataUpdated();
}

QStringList HypaDtGuiPlugin::getImuSourceNames() const
{
  qDebug() << "[HypaDtGuiPlugin] getImuSourceNames() called";
  if (this->data_manager_)
    return this->data_manager_->getImuSourceNames();
  return QStringList();
}

QStringList HypaDtGuiPlugin::getJointNames() const
{
  qDebug() << "[HypaDtGuiPlugin] getJointNames() called";
  if (this->data_manager_)
    return this->data_manager_->getJointNames();
  return QStringList();
}

QString HypaDtGuiPlugin::formatSourceSummary(const QString &_kind,
                                             const QString &_source) const
{
  qDebug() << "[HypaDtGuiPlugin] formatSourceSummary() called:" << _kind
           << _source;
  if (this->data_manager_)
    return this->data_manager_->formatSourceSummary(_kind, _source);
  return QString();
}

QString HypaDtGuiPlugin::formatSeriesSummary(const QString &_kind,
                                             const QString &_source,
                                             const QVariantList &_metrics) const
{
  qDebug() << "[HypaDtGuiPlugin] formatSeriesSummary() called:" << _kind
           << _source << "metrics len:" << _metrics.size();
  if (this->data_manager_)
    return this->data_manager_->formatSeriesSummary(_kind, _source, _metrics);
  return QString();
}

QVariantList HypaDtGuiPlugin::getSeriesPoints(const QString &_kind,
                                              const QString &_source,
                                              const QString &_metric) const
{
  qDebug() << "[HypaDtGuiPlugin] getSeriesPoints() called:" << _kind << _source
           << _metric;

  if (!this->data_manager_)
    return QVariantList();

  QVariantList points =
      this->data_manager_->getSeriesPoints(_kind, _source, _metric);
  qDebug() << "[HypaDtGuiPlugin] getSeriesPoints result size:" << points.size();
  const int to_show = std::min(5, points.size());
  for (int i = 0; i < to_show; ++i)
  {
    const QVariant &v = points.at(i);
    if (v.canConvert<QVariantMap>())
    {
      const QVariantMap m = v.toMap();
      qDebug() << "[HypaDtGuiPlugin] point" << i << ":" << m;
    }
    else
    {
      qDebug() << "[HypaDtGuiPlugin] point" << i << "type" << v.typeName() << v;
    }
  }

  return points;
}

QVariantList HypaDtGuiPlugin::getImuCovariance(
    const QString &_source, const QString &_covarianceName) const
{
  qDebug() << "[HypaDtGuiPlugin] getImuCovariance() called:" << _source
           << _covarianceName;
  if (this->data_manager_)
    return this->data_manager_->getImuCovariance(_source, _covarianceName);
  return QVariantList();
}

}  // namespace hypa_dt_gui_plugin

GZ_ADD_PLUGIN(hypa_dt_gui_plugin::HypaDtGuiPlugin, gz::gui::Plugin)
