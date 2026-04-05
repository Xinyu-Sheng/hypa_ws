#include "hypa-dt-gui-plugin/HypaDtGuiPlugin.hh"

#include <QtQml/qqml.h>

#include <QDebug>
#include <QQmlContext>

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
          &HypaDtGuiPlugin::onImuDataUpdated);
  connect(this->data_manager_.get(), &HypaDataManager::jointDataUpdated, this,
          &HypaDtGuiPlugin::onJointDataUpdated);

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
  return this->imu_selection_model_.get();
}

QObject *HypaDtGuiPlugin::jointSelectionModel() const
{
  return this->joint_selection_model_.get();
}

void HypaDtGuiPlugin::onImuDataUpdated()
{
  if (!this->data_manager_ || !this->imu_selection_model_)
    return;

  const QStringList imu_names = this->data_manager_->getImuSourceNames();
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
  this->joint_selection_model_->syncNames(joint_names, true);

  for (const auto &name : joint_names)
  {
    this->joint_selection_model_->updateLatestText(
        name, this->data_manager_->formatSourceSummary("joint", name));
  }

  emit jointDataUpdated();
}

}  // namespace hypa_dt_gui_plugin

GZ_ADD_PLUGIN(hypa_dt_gui_plugin::HypaDtGuiPlugin, gz::gui::Plugin)
