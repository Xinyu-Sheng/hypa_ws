#ifndef HYPA_DT_GUI_PLUGIN_HYPA_SELECTION_MODEL_HH_
#define HYPA_DT_GUI_PLUGIN_HYPA_SELECTION_MODEL_HH_

#include <QAbstractListModel>
#include <QStringList>
#include <QVariantMap>
#include <string>
#include <vector>

namespace hypa_dt_gui_plugin
{
class SelectionModel : public QAbstractListModel
{
  Q_OBJECT

  public:
  enum SelectionRole
  {
    NameRole = Qt::UserRole + 1,
    SelectedRole,
    LatestTextRole
  };

  explicit SelectionModel(QObject *_parent = nullptr);

  int rowCount(const QModelIndex &_parent = QModelIndex()) const override;
  QVariant data(const QModelIndex &_index,
                int _role = Qt::DisplayRole) const override;
  bool setData(const QModelIndex &_index, const QVariant &_value,
               int _role = Qt::EditRole) override;
  Qt::ItemFlags flags(const QModelIndex &_index) const override;
  QHash<int, QByteArray> roleNames() const override;

  Q_INVOKABLE int count() const;
  Q_INVOKABLE QVariantMap get(const int _index) const;
  Q_INVOKABLE QStringList names() const;
  Q_INVOKABLE QStringList selectedNames() const;
  Q_INVOKABLE bool isSelected(const QString &_name) const;
  Q_INVOKABLE void setSelected(const QString &_name, bool _selected);

  void syncNames(const QStringList &_names, bool _defaultSelected = true);
  void updateLatestText(const QString &_name, const QString &_latestText);

  Q_SIGNALS:
  void entriesChanged();

  private:
  struct Entry
  {
    QString name;
    bool selected = true;
    QString latestText;
  };

  int indexOf(const QString &_name) const;

  std::vector<Entry> entries_;
};
}  // namespace hypa_dt_gui_plugin

#endif  // HYPA_DT_GUI_PLUGIN_HYPA_SELECTION_MODEL_HH_
