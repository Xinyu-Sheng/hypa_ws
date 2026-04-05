#include "hypa-dt-gui-plugin/HypaSelectionModel.hh"

#include <algorithm>

namespace hypa_dt_gui_plugin
{
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
  emit entriesChanged();
}

void SelectionModel::syncNames(const QStringList &_names, bool _defaultSelected)
{
  bool changed = false;

  if (_names.size() == static_cast<int>(this->entries_.size()))
  {
    changed = false;
    for (int i = 0; i < _names.size(); ++i)
    {
      if (this->entries_[static_cast<size_t>(i)].name != _names[i])
      {
        changed = true;
        break;
      }
    }

    if (!changed)
      return;
  }
  else
  {
    changed = true;
  }

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

  if (changed)
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
