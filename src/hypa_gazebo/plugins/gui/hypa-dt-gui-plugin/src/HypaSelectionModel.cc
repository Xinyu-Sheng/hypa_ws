#include "hypa-dt-gui-plugin/HypaSelectionModel.hh"

#include <QDebug>

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

SelectionModel::SelectionModel(QObject *_parent) : QAbstractListModel(_parent)
{
}

int SelectionModel::rowCount(const QModelIndex &_parent) const
{
  if (_parent.isValid())
    return 0;

  return static_cast<int>(this->entries_.size());
}

QVariant SelectionModel::data(const QModelIndex &_index, int _role) const
{
  if (!_index.isValid() || _index.row() < 0 ||
      _index.row() >= static_cast<int>(this->entries_.size()))
  {
    return QVariant();
  }

  const auto &entry = this->entries_[static_cast<size_t>(_index.row())];

  switch (_role)
  {
    case NameRole:
      return entry.name;
    case SelectedRole:
      return entry.selected;
    case LatestTextRole:
      return entry.latestText;
    default:
      return QVariant();
  }
}

bool SelectionModel::setData(const QModelIndex &_index, const QVariant &_value,
                             int _role)
{
  if (!_index.isValid() || _index.row() < 0 ||
      _index.row() >= static_cast<int>(this->entries_.size()))
  {
    return false;
  }

  auto &entry = this->entries_[static_cast<size_t>(_index.row())];

  if (_role == SelectedRole)
  {
    const bool selected = _value.toBool();
    if (entry.selected == selected)
      return true;

    entry.selected = selected;
    emit dataChanged(_index, _index, {SelectedRole});
    emit entriesChanged();
    return true;
  }

  return false;
}

Qt::ItemFlags SelectionModel::flags(const QModelIndex &_index) const
{
  if (!_index.isValid())
    return Qt::NoItemFlags;

  return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable;
}

QHash<int, QByteArray> SelectionModel::roleNames() const
{
  QHash<int, QByteArray> roles;
  roles[NameRole] = "name";
  roles[SelectedRole] = "selected";
  roles[LatestTextRole] = "latestText";
  return roles;
}

int SelectionModel::count() const
{
  return static_cast<int>(this->entries_.size());
}

QVariantMap SelectionModel::get(const int _index) const
{
  QVariantMap result;

  if (_index < 0 || _index >= static_cast<int>(this->entries_.size()))
    return result;

  const auto &entry = this->entries_[static_cast<size_t>(_index)];
  result["name"] = entry.name;
  result["selected"] = entry.selected;
  result["latestText"] = entry.latestText;
  return result;
}

QStringList SelectionModel::names() const
{
  QStringList result;
  for (const auto &entry : this->entries_)
  {
    result.push_back(entry.name);
  }
  return result;
}

QStringList SelectionModel::selectedNames() const
{
  QStringList result;
  for (const auto &entry : this->entries_)
  {
    if (entry.selected)
      result.push_back(entry.name);
  }
  return result;
}

bool SelectionModel::isSelected(const QString &_name) const
{
  const int index = this->indexOf(_name);
  if (index < 0)
    return false;

  return this->entries_[static_cast<size_t>(index)].selected;
}

void SelectionModel::setSelected(const QString &_name, bool _selected)
{
  const int index = this->indexOf(_name);
  if (index < 0)
    return;

  auto &entry = this->entries_[static_cast<size_t>(index)];
  if (entry.selected == _selected)
    return;

  entry.selected = _selected;
  const QModelIndex model_index = this->index(index, 0);
  emit dataChanged(model_index, model_index, {SelectedRole});
  HYPA_DT_DEBUG_LOG() << "[SelectionModel] setSelected:" << _name << _selected;
  emit entriesChanged();
}

void SelectionModel::syncNames(const QStringList &_names, bool _defaultSelected)
{
  // Fast-path: exact same size and same names in same order -> nothing to do
  if (_names.size() == static_cast<int>(this->entries_.size()))
  {
    bool same = true;
    for (int i = 0; i < _names.size(); ++i)
    {
      if (this->entries_[static_cast<size_t>(i)].name != _names[i])
      {
        same = false;
        break;
      }
    }

    if (same)
      return;
  }

  // Optimize simple prefix add (new items appended at end)
  if (_names.size() > static_cast<int>(this->entries_.size()))
  {
    bool prefix = true;
    for (size_t i = 0; i < this->entries_.size(); ++i)
    {
      if (this->entries_[i].name != _names[static_cast<int>(i)])
      {
        prefix = false;
        break;
      }
    }

    if (prefix)
    {
      const int oldN = static_cast<int>(this->entries_.size());
      const int newN = _names.size();
      beginInsertRows(QModelIndex(), oldN, newN - 1);
      for (int i = oldN; i < newN; ++i)
      {
        Entry entry;
        entry.name = _names[i];
        entry.selected = _defaultSelected;
        this->entries_.push_back(entry);
      }
      endInsertRows();
      HYPA_DT_DEBUG_LOG()
          << "[SelectionModel] syncNames: appended entries count="
          << this->entries_.size();
      emit entriesChanged();
      return;
    }
  }

  // Optimize simple prefix remove (trailing items removed)
  if (_names.size() < static_cast<int>(this->entries_.size()))
  {
    bool prefix = true;
    for (int i = 0; i < _names.size(); ++i)
    {
      if (this->entries_[static_cast<size_t>(i)].name != _names[i])
      {
        prefix = false;
        break;
      }
    }

    if (prefix)
    {
      const int oldN = static_cast<int>(this->entries_.size());
      const int newN = _names.size();
      beginRemoveRows(QModelIndex(), newN, oldN - 1);
      this->entries_.erase(this->entries_.begin() + newN, this->entries_.end());
      endRemoveRows();
      HYPA_DT_DEBUG_LOG()
          << "[SelectionModel] syncNames: removed trailing entries, count="
          << this->entries_.size();
      emit entriesChanged();
      return;
    }
  }

  // Fallback: rebuild model (names changed significantly)
  std::vector<Entry> next_entries;
  next_entries.reserve(static_cast<size_t>(_names.size()));

  for (const auto &name : _names)
  {
    const int index = this->indexOf(name);
    if (index >= 0)
    {
      next_entries.push_back(this->entries_[static_cast<size_t>(index)]);
    }
    else
    {
      Entry entry;
      entry.name = name;
      entry.selected = _defaultSelected;
      next_entries.push_back(entry);
    }
  }

  beginResetModel();
  this->entries_ = std::move(next_entries);
  endResetModel();
  HYPA_DT_DEBUG_LOG() << "[SelectionModel] syncNames: entries count="
                      << this->entries_.size();
  emit entriesChanged();
}

void SelectionModel::updateLatestText(const QString &_name,
                                      const QString &_latestText)
{
  const int index = this->indexOf(_name);
  if (index < 0)
    return;

  auto &entry = this->entries_[static_cast<size_t>(index)];
  if (entry.latestText == _latestText)
    return;

  entry.latestText = _latestText;
  const QModelIndex model_index = this->index(index, 0);
  emit dataChanged(model_index, model_index, {LatestTextRole});
  HYPA_DT_DEBUG_LOG() << "[SelectionModel] updateLatestText:" << _name
                      << _latestText.left(120);
}

int SelectionModel::indexOf(const QString &_name) const
{
  for (size_t i = 0; i < this->entries_.size(); ++i)
  {
    if (this->entries_[i].name == _name)
      return static_cast<int>(i);
  }

  return -1;
}

}  // namespace hypa_dt_gui_plugin

#undef HYPA_DT_DEBUG_LOG
