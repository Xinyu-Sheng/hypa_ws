#ifndef HYPA_DT_GUI_PLUGIN_HYPA_QCUSTOM_PLOT_ITEM_HH_
#define HYPA_DT_GUI_PLUGIN_HYPA_QCUSTOM_PLOT_ITEM_HH_

#include <QHash>
#include <QImage>
#include <QQuickPaintedItem>
#include <QSet>
#include <QSize>
#include <QVariantList>
#include <memory>

class QCPGraph;
class QCustomPlot;

namespace hypa_dt_gui_plugin
{
class HypaQCustomPlotItem : public QQuickPaintedItem
{
  Q_OBJECT
  Q_PROPERTY(QString chartKind READ chartKind WRITE setChartKind NOTIFY
                 chartKindChanged)
  Q_PROPERTY(QVariantList seriesModel READ seriesModel WRITE setSeriesModel
                 NOTIFY seriesModelChanged)
  Q_PROPERTY(QObject *dataProvider READ dataProvider WRITE setDataProvider
                 NOTIFY dataProviderChanged)
  Q_PROPERTY(bool hasData READ hasData NOTIFY hasDataChanged)
  Q_PROPERTY(bool autoYRange READ autoYRange WRITE setAutoYRange NOTIFY
                 autoYRangeChanged)
  Q_PROPERTY(double yScale READ yScale WRITE setYScale NOTIFY yScaleChanged)
  Q_PROPERTY(double yMin READ yMin WRITE setYMin NOTIFY yRangeChanged)
  Q_PROPERTY(double yMax READ yMax WRITE setYMax NOTIFY yRangeChanged)

  public:
  explicit HypaQCustomPlotItem(QQuickItem *_parent = nullptr);
  ~HypaQCustomPlotItem() override;

  QString chartKind() const;
  void setChartKind(const QString &_chartKind);

  bool autoYRange() const;
  void setAutoYRange(bool _v);

  double yScale() const;
  void setYScale(double _v);

  double yMin() const;
  void setYMin(double _v);

  double yMax() const;
  void setYMax(double _v);

  QVariantList seriesModel() const;
  void setSeriesModel(const QVariantList &_seriesModel);

  QObject *dataProvider() const;
  void setDataProvider(QObject *_dataProvider);

  bool hasData() const;

  Q_INVOKABLE void refreshPlot();

  void paint(QPainter *_painter) override;

  Q_SIGNALS:
  void chartKindChanged();
  void seriesModelChanged();
  void dataProviderChanged();
  void hasDataChanged();
  void autoYRangeChanged();
  void yScaleChanged();
  void yRangeChanged();

  private Q_SLOTS:
  void onBackendUpdated();

  private:
  struct SeriesState
  {
    QCPGraph *graph = nullptr;
    uint64_t sequence = 0;
    bool range_initialized = false;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
  };

  void ensurePlot();
  QCPGraph *ensureGraph(const QString &_seriesKey, const QString &_metricName);
  void removeInactiveGraphs(const QSet<QString> &_activeKeys);
  void updateAxesFromRanges();
  void updateSeriesRange(SeriesState &_state, const QVector<double> &_x,
                         const QVector<double> &_y, bool _reset);
  void setHasData(bool _hasData);

  QString chart_kind_;
  QVariantList series_model_;
  QObject *data_provider_ = nullptr;
  bool has_data_ = false;
  bool auto_y_range_ = true;
  double y_scale_ = 1.0;
  double y_min_override_;
  double y_max_override_;

  std::unique_ptr<QCustomPlot> plot_;
  QHash<QString, SeriesState> series_states_;
  QImage cached_image_;
  QSize cached_image_size_;
  double cached_dpr_ = 0.0;
  bool plot_dirty_ = true;
};
}  // namespace hypa_dt_gui_plugin

#endif  // HYPA_DT_GUI_PLUGIN_HYPA_QCUSTOM_PLOT_ITEM_HH_
