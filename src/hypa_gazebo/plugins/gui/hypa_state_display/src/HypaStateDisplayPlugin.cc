// HypaStateDisplayPlugin 实现
#include "hypa_state_display/HypaStateDisplayPlugin.hh"

#include <QDebug>

#include <gz/plugin/Register.hh>

#include "hypa_state_display/HypaDataManager.hh"

namespace hypa_display
{
JointListModel::JointListModel(QObject *_parent) : QAbstractListModel(_parent)
{
}

int JointListModel::rowCount(const QModelIndex &_parent) const
{
  if (_parent.isValid())
    return 0;
  return this->joint_items_.size();
}

QVariant JointListModel::data(const QModelIndex &_index, int _role) const
{
  if (!_index.isValid() ||
      _index.row() >= static_cast<int>(this->joint_items_.size()))
    return QVariant();

  const auto &item = this->joint_items_[_index.row()];

  switch (_role)
  {
    case NameRole:
      return QString::fromStdString(item.name);
    case PositionRole:
      return item.position;
    case AccelerationRole:
      return item.acceleration;
    case SelectedRole:
      return item.selected;
    default:
      return QVariant();
  }
}

bool JointListModel::setData(const QModelIndex &_index, const QVariant &_value,
                             int _role)
{
  if (!_index.isValid() ||
      _index.row() >= static_cast<int>(this->joint_items_.size()))
    return false;

  auto &item = this->joint_items_[_index.row()];

  switch (_role)
  {
    case SelectedRole:
      item.selected = _value.toBool();
      emit dataChanged(_index, _index, {_role});
      return true;
    default:
      return false;
  }
}

Qt::ItemFlags JointListModel::flags(const QModelIndex &_index) const
{
  if (!_index.isValid())
    return Qt::NoItemFlags;
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable;
}

QHash<int, QByteArray> JointListModel::roleNames() const
{
  QHash<int, QByteArray> roles;
  roles[NameRole] = "name";
  roles[PositionRole] = "position";
  roles[AccelerationRole] = "acceleration";
  roles[SelectedRole] = "selected";
  return roles;
}

void JointListModel::updateJoint(const std::string &_name, double _position,
                                 double _acceleration)
{
  for (size_t i = 0; i < this->joint_items_.size(); ++i)
  {
    if (this->joint_items_[i].name == _name)
    {
      this->joint_items_[i].position = _position;
      this->joint_items_[i].acceleration = _acceleration;
      auto idx = index(i);
      emit dataChanged(idx, idx, {PositionRole, AccelerationRole});
      return;
    }
  }
}

void JointListModel::setJoints(const std::vector<std::string> &_names)
{
  beginResetModel();
  this->joint_items_.clear();
  for (const auto &name : _names)
  {
    this->joint_items_.push_back({name, 0.0, 0.0, true});
  }
  endResetModel();
}

void JointListModel::toggleJointSelection(const std::string &_name)
{
  for (size_t i = 0; i < this->joint_items_.size(); ++i)
  {
    if (this->joint_items_[i].name == _name)
    {
      this->joint_items_[i].selected = !this->joint_items_[i].selected;
      auto idx = index(i);
      emit dataChanged(idx, idx, {SelectedRole});
      return;
    }
  }
}

void JointListModel::selectAllJoints()
{
  for (auto &item : this->joint_items_)
  {
    item.selected = true;
  }
  emit dataChanged(index(0), index(this->joint_items_.size() - 1),
                   {SelectedRole});
}

void JointListModel::deselectAllJoints()
{
  for (auto &item : this->joint_items_)
  {
    item.selected = false;
  }
  emit dataChanged(index(0), index(this->joint_items_.size() - 1),
                   {SelectedRole});
}

std::vector<std::string> JointListModel::getSelectedJoints() const
{
  std::vector<std::string> selected;
  for (const auto &item : this->joint_items_)
  {
    if (item.selected)
      selected.push_back(item.name);
  }
  return selected;
}

// ==================== HypaStateDisplayPlugin ====================

HypaStateDisplayPlugin::HypaStateDisplayPlugin()
{
  qDebug() << "[HypaStateDisplayPlugin] 插件初始化";
}

HypaStateDisplayPlugin::~HypaStateDisplayPlugin() = default;

void HypaStateDisplayPlugin::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  qDebug() << "[HypaStateDisplayPlugin] 加载配置";

  // 从插件配置中读取话题名称（可选，有默认值）
  std::string imu_topic = "/hypa/imu";
  std::string joint_state_topic = "/hypa/joint_states";
  std::string log_topic = "/hypa/log_stream";

  if (_pluginElem)
  {
    auto elem = _pluginElem->FirstChildElement("imu_topic");
    if (elem && elem->GetText())
      imu_topic = elem->GetText();
    elem = _pluginElem->FirstChildElement("joint_state_topic");
    if (elem && elem->GetText())
      joint_state_topic = elem->GetText();
    elem = _pluginElem->FirstChildElement("log_topic");
    if (elem && elem->GetText())
      log_topic = elem->GetText();
  }

  this->imu_topic_ = imu_topic;
  this->joint_state_topic_ = joint_state_topic;

  // 初始化数据管理器
  this->data_manager_ = std::make_unique<HypaDataManager>(this, this);
  this->data_manager_->initialize(imu_topic, joint_state_topic, log_topic);

  // 初始化关节列表模型
  this->joint_model_ = std::make_unique<JointListModel>(this);
  const auto &joint_names = this->data_manager_->getJointNames();
  this->joint_model_->setJoints(joint_names);

  // 连接信号
  connect(this->data_manager_.get(), &HypaDataManager::jointDataUpdated, this,
          &HypaStateDisplayPlugin::OnJointDataUpdated);
  connect(this->data_manager_.get(), &HypaDataManager::imuDataUpdated, this,
          &HypaStateDisplayPlugin::OnIMUDataUpdated);
  connect(this->data_manager_.get(), &HypaDataManager::logMessageReceived, this,
          &HypaStateDisplayPlugin::OnLogMessageReceived);

  qDebug() << "[HypaStateDisplayPlugin] 插件配置完成";
}

std::string HypaStateDisplayPlugin::Title() const
{
  return "HYPA State Display";
}

QObject *HypaStateDisplayPlugin::getJointListModel() const
{
  return this->joint_model_.get();
}

void HypaStateDisplayPlugin::setIMUTopic(const QString &_topic)
{
  this->imu_topic_ = _topic.toStdString();
  this->data_manager_->subscribeToIMUTopic(this->imu_topic_);
  qDebug() << "[HypaStateDisplayPlugin] IMU 话题已更新为:" << _topic;
}

void HypaStateDisplayPlugin::setJointStateTopic(const QString &_topic)
{
  this->joint_state_topic_ = _topic.toStdString();
  this->data_manager_->subscribeToJointStateTopic(this->joint_state_topic_);
  qDebug() << "[HypaStateDisplayPlugin] 关节状态话题已更新为:" << _topic;
}

void HypaStateDisplayPlugin::toggleJointSelection(const QString &_jointName)
{
  this->joint_model_->toggleJointSelection(_jointName.toStdString());
}

void HypaStateDisplayPlugin::selectAllJoints()
{
  this->joint_model_->selectAllJoints();
}

void HypaStateDisplayPlugin::deselectAllJoints()
{
  this->joint_model_->deselectAllJoints();
}

QString HypaStateDisplayPlugin::getIMUTopic() const
{
  return QString::fromStdString(this->imu_topic_);
}

QString HypaStateDisplayPlugin::getJointStateTopic() const
{
  return QString::fromStdString(this->joint_state_topic_);
}

void HypaStateDisplayPlugin::OnJointDataUpdated()
{
  // 从数据管理器更新关节数据到模型
  if (this->data_manager_)
  {
    const auto &names = this->data_manager_->getJointNames();
    for (const auto &name : names)
    {
      double pos = 0.0, accel = 0.0;
      if (this->data_manager_->getJointData(name, pos, accel))
      {
        this->joint_model_->updateJoint(name, pos, accel);
      }
    }
  }
  emit jointDataUpdated();
}

void HypaStateDisplayPlugin::OnIMUDataUpdated()
{
  emit imuDataUpdated();
}

void HypaStateDisplayPlugin::OnLogMessageReceived(const QString &_message)
{
  emit logMessageReceived(_message);
}

}  // namespace hypa_display

// 注册 Gazebo 插件
GZ_ADD_PLUGIN(hypa_display::HypaStateDisplayPlugin, gz::gui::Plugin)
