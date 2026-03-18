// 该文件是 hypa_gazebo GUI 插件的主头文件
#ifndef HYPA_STATE_DISPLAY_PLUGIN_HH_
#define HYPA_STATE_DISPLAY_PLUGIN_HH_

#include <QAbstractListModel>
#include <QObject>
#include <QString>
#include <QStringListModel>
#include <QVariant>
#include <memory>
#include <string>
#include <vector>

#include <gz/gui/Plugin.hh>

namespace hypa_display
{
class HypaDataManager;

/// 关节数据模型（用于 QML ListView 绑定）
class JointListModel : public QAbstractListModel
{
  Q_OBJECT

  public:
  enum JointRoles
  {
    NameRole = Qt::UserRole + 1,
    PositionRole,
    AccelerationRole,
    SelectedRole
  };

  JointListModel(QObject *_parent = nullptr);
  int rowCount(const QModelIndex &_parent = QModelIndex()) const override;
  QVariant data(const QModelIndex &_index,
                int _role = Qt::DisplayRole) const override;
  bool setData(const QModelIndex &_index, const QVariant &_value,
               int _role = Qt::EditRole) override;
  Qt::ItemFlags flags(const QModelIndex &_index) const override;
  QHash<int, QByteArray> roleNames() const override;

  void updateJoint(const std::string &_name, double _position,
                   double _acceleration);
  void setJoints(const std::vector<std::string> &_names);
  void toggleJointSelection(const std::string &_name);
  void selectAllJoints();
  void deselectAllJoints();

  std::vector<std::string> getSelectedJoints() const;

  private:
  struct JointItem
  {
    std::string name;
    double position = 0.0;
    double acceleration = 0.0;
    bool selected = true;
  };

  std::vector<JointItem> joint_items_;
};

/// 主 Gazebo GUI 插件类
class HypaStateDisplayPlugin : public gz::gui::Plugin
{
  Q_OBJECT

  public:
  HypaStateDisplayPlugin();
  ~HypaStateDisplayPlugin() override;

  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  std::string Title() const override;

  // QML 可调用的插槽（slot）方法
  Q_INVOKABLE QObject *getJointListModel() const;
  Q_INVOKABLE void setIMUTopic(const QString &_topic);
  Q_INVOKABLE void setJointStateTopic(const QString &_topic);
  Q_INVOKABLE void toggleJointSelection(const QString &_jointName);
  Q_INVOKABLE void selectAllJoints();
  Q_INVOKABLE void deselectAllJoints();
  Q_INVOKABLE QString getIMUTopic() const;
  Q_INVOKABLE QString getJointStateTopic() const;

  signals:
  // 上游信号（从 DataManager 转发到 QML）
  void jointDataUpdated();
  void imuDataUpdated();
  void logMessageReceived(const QString &_message);

  private:
  /// 处理关节数据更新
  void OnJointDataUpdated();

  /// 处理 IMU 数据更新
  void OnIMUDataUpdated();

  /// 处理日志消息
  void OnLogMessageReceived(const QString &_message);

  std::unique_ptr<HypaDataManager> data_manager_;
  std::unique_ptr<JointListModel> joint_model_;

  std::string imu_topic_ = "/hypa/imu";
  std::string joint_state_topic_ = "/hypa/joint_states";
};
}  // namespace hypa_display

#endif  // HYPA_STATE_DISPLAY_PLUGIN_HH_
