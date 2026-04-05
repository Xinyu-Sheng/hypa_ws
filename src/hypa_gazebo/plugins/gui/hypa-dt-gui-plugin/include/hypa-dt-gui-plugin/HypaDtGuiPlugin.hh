#ifndef HYPA_DT_GUI_PLUGIN_HYPA_DT_GUI_PLUGIN_HH_
#define HYPA_DT_GUI_PLUGIN_HYPA_DT_GUI_PLUGIN_HH_

#include <QObject>
#include <QString>
#include <memory>
#include <string>

#include <gz/gui/Plugin.hh>

#include "hypa-dt-gui-plugin/HypaDataManager.hh"
#include "hypa-dt-gui-plugin/HypaSelectionModel.hh"

namespace hypa_dt_gui_plugin
{
class HypaDtGuiPlugin : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QObject *imuSelectionModel READ imuSelectionModel CONSTANT)
  Q_PROPERTY(QObject *jointSelectionModel READ jointSelectionModel CONSTANT)

  public:
  HypaDtGuiPlugin();
  ~HypaDtGuiPlugin() override;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;
  std::string Title() const override;

  QObject *imuSelectionModel() const;
  QObject *jointSelectionModel() const;

  Q_SIGNALS:
  void imuDataUpdated();
  void jointDataUpdated();

  private Q_SLOTS:
  void onImuDataUpdated();
  void onJointDataUpdated();

  private:
  std::unique_ptr<HypaDataManager> data_manager_;
  std::unique_ptr<SelectionModel> imu_selection_model_;
  std::unique_ptr<SelectionModel> joint_selection_model_;
};
}  // namespace hypa_dt_gui_plugin

#endif  // HYPA_DT_GUI_PLUGIN_HYPA_DT_GUI_PLUGIN_HH_
