#include "hypa-dt-gui-plugin/HypaQCustomPlotItem.hh"

#include <qcustomplot.h>

#include <QColor>
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QVariantMap>
#include <algorithm>

#include "hypa-dt-gui-plugin/HypaDtGuiPlugin.hh"

namespace hypa_dt_gui_plugin
{
namespace
{
constexpr double kHistoryWindowSec = 30.0;
}

HypaQCustomPlotItem::HypaQCustomPlotItem(QQuickItem *_parent)
    : QQuickPaintedItem(_parent)
{
  this->setAntialiasing(false);
  this->setRenderTarget(QQuickPaintedItem::FramebufferObject);
  this->ensurePlot();
}

HypaQCustomPlotItem::~HypaQCustomPlotItem() = default;

QString HypaQCustomPlotItem::chartKind() const
{
  return this->chart_kind_;
}

void HypaQCustomPlotItem::setChartKind(const QString &_chartKind)
{
  if (this->chart_kind_ == _chartKind)
    return;

  this->chart_kind_ = _chartKind;
  this->series_states_.clear();
  if (this->plot_)
  {
    this->plot_->clearGraphs();
  }
  emit chartKindChanged();
}

QVariantList HypaQCustomPlotItem::seriesModel() const
{
  return this->series_model_;
}

void HypaQCustomPlotItem::setSeriesModel(const QVariantList &_seriesModel)
{
  this->series_model_ = _seriesModel;
  emit seriesModelChanged();
}

QObject *HypaQCustomPlotItem::dataProvider() const
{
  return this->data_provider_;
}

void HypaQCustomPlotItem::setDataProvider(QObject *_dataProvider)
{
  if (this->data_provider_ == _dataProvider)
    return;

  if (this->data_provider_)
  {
    disconnect(this->data_provider_, SIGNAL(imuDataUpdated()), this,
               SLOT(onBackendUpdated()));
    disconnect(this->data_provider_, SIGNAL(jointDataUpdated()), this,
               SLOT(onBackendUpdated()));
  }

  this->data_provider_ = _dataProvider;

  if (this->data_provider_)
  {
    connect(this->data_provider_, SIGNAL(imuDataUpdated()), this,
            SLOT(onBackendUpdated()), Qt::QueuedConnection);
    connect(this->data_provider_, SIGNAL(jointDataUpdated()), this,
            SLOT(onBackendUpdated()), Qt::QueuedConnection);
  }

  emit dataProviderChanged();
}

bool HypaQCustomPlotItem::hasData() const
{
  return this->has_data_;
}

void HypaQCustomPlotItem::refreshPlot()
{
  this->ensurePlot();

  auto *backend = qobject_cast<HypaDtGuiPlugin *>(this->data_provider_);
  if (!backend || !this->plot_)
  {
    this->setHasData(false);
    return;
  }

  QSet<QString> active_keys;
  bool any_data = false;

  for (const QVariant &series_entry : this->series_model_)
  {
    const QVariantMap map = series_entry.toMap();
    const bool selected = map.value("sourceSelected").toBool();
    if (!selected)
      continue;

    const QString source_name = map.value("sourceName").toString();
    const QString metric_name = map.value("metricName").toString();
    if (source_name.isEmpty() || metric_name.isEmpty())
      continue;

    const QString key = source_name + "|" + metric_name;
    active_keys.insert(key);

    QCPGraph *graph = this->ensureGraph(key, metric_name);
    if (!graph)
      continue;

    SeriesState &state = this->series_states_[key];
    state.graph = graph;

    QVector<double> x_values;
    QVector<double> y_values;
    uint64_t latest_sequence = state.sequence;
    bool reset_required = false;

    const bool ok = backend->getSeriesDataDelta(
        this->chart_kind_, source_name, metric_name, state.sequence, x_values,
        y_values, latest_sequence, reset_required);
    if (!ok)
      continue;

    if (reset_required)
    {
      state.graph->data()->clear();
      state.range_initialized = false;
    }

    if (!x_values.isEmpty())
    {
      state.graph->addData(x_values, y_values);
      this->updateSeriesRange(state, x_values, y_values, reset_required);
    }

    if (state.graph->dataCount() > 0)
    {
      auto end_it = state.graph->data()->constEnd();
      --end_it;
      const double latest_x = end_it->key;
      state.graph->data()->removeBefore(latest_x - kHistoryWindowSec);
    }

    state.sequence = latest_sequence;
    if (state.graph->dataCount() > 0)
      any_data = true;
  }

  this->removeInactiveGraphs(active_keys);
  this->updateAxesFromRanges();
  this->plot_->replot(QCustomPlot::rpQueuedReplot);
  this->setHasData(any_data);
  this->update();
}

void HypaQCustomPlotItem::paint(QPainter *_painter)
{
  if (!this->plot_ || !_painter)
    return;

  const double dpr = this->window() ? this->window()->devicePixelRatio() : 1.0;
  const QSize image_size(static_cast<int>(this->width() * dpr),
                         static_cast<int>(this->height() * dpr));
  if (image_size.width() <= 0 || image_size.height() <= 0)
    return;

  QImage image(image_size, QImage::Format_ARGB32_Premultiplied);
  image.setDevicePixelRatio(dpr);
  image.fill(Qt::transparent);

  this->plot_->setViewport(QRect(0, 0, static_cast<int>(this->width()),
                                 static_cast<int>(this->height())));
  QCPPainter qcp_painter(&image);
  this->plot_->toPainter(&qcp_painter, this->width(), this->height());
  _painter->drawImage(QPoint(0, 0), image);
}

void HypaQCustomPlotItem::onBackendUpdated()
{
  this->refreshPlot();
}

void HypaQCustomPlotItem::ensurePlot()
{
  if (this->plot_)
    return;

  this->plot_ = std::make_unique<QCustomPlot>();
  this->plot_->setNoAntialiasingOnDrag(true);
  this->plot_->setInteractions(QCP::Interactions());
  this->plot_->legend->setVisible(false);
  this->plot_->setBackground(QColor("#ffffff"));

  this->plot_->xAxis->setVisible(true);
  this->plot_->yAxis->setVisible(true);
  this->plot_->xAxis->setTickLabelColor(QColor("#6b7280"));
  this->plot_->yAxis->setTickLabelColor(QColor("#6b7280"));
  this->plot_->xAxis->grid()->setPen(QPen(QColor("#e6edf3")));
  this->plot_->yAxis->grid()->setPen(QPen(QColor("#e6edf3")));
  this->plot_->xAxis->setRange(0.0, 1.0);
  this->plot_->yAxis->setRange(-1.0, 1.0);
}

QCPGraph *HypaQCustomPlotItem::ensureGraph(const QString &_seriesKey,
                                           const QString &_metricName)
{
  this->ensurePlot();
  if (!this->plot_)
    return nullptr;

  auto state_it = this->series_states_.find(_seriesKey);
  if (state_it != this->series_states_.end() && state_it->graph)
    return state_it->graph;

  QCPGraph *graph =
      this->plot_->addGraph(this->plot_->xAxis, this->plot_->yAxis);
  if (!graph)
    return nullptr;

  const uint32_t hash = qHash(_seriesKey);
  const int hue = static_cast<int>(hash % 360U);
  QColor color;
  color.setHsv(hue, 180, 210);

  graph->setName(_metricName);
  graph->setPen(QPen(color, 1.4));
  graph->setLineStyle(QCPGraph::lsLine);
  graph->setScatterStyle(QCPScatterStyle::ssNone);
  graph->setAdaptiveSampling(true);

  SeriesState state;
  state.graph = graph;
  this->series_states_.insert(_seriesKey, state);
  return graph;
}

void HypaQCustomPlotItem::removeInactiveGraphs(const QSet<QString> &_activeKeys)
{
  if (!this->plot_)
    return;

  auto it = this->series_states_.begin();
  while (it != this->series_states_.end())
  {
    if (_activeKeys.contains(it.key()))
    {
      ++it;
      continue;
    }

    if (it->graph)
      this->plot_->removeGraph(it->graph);
    it = this->series_states_.erase(it);
  }
}

void HypaQCustomPlotItem::updateAxesFromRanges()
{
  bool has_x = false;
  double latest_x = 0.0;

  for (auto it = this->series_states_.begin(); it != this->series_states_.end();
       ++it)
  {
    if (!it->graph || it->graph->dataCount() == 0)
      continue;

    auto end_it = it->graph->data()->constEnd();
    --end_it;
    if (!has_x)
    {
      latest_x = end_it->key;
      has_x = true;
    }
    else
    {
      latest_x = std::max(latest_x, end_it->key);
    }
  }

  if (!has_x)
  {
    this->plot_->xAxis->setRange(0.0, 1.0);
    this->plot_->yAxis->setRange(-1.0, 1.0);
    return;
  }

  const double min_x = latest_x - kHistoryWindowSec;
  const double max_x = latest_x;

  bool has_y = false;
  double min_y = -1.0;
  double max_y = 1.0;

  for (auto it = this->series_states_.begin(); it != this->series_states_.end();
       ++it)
  {
    if (!it->graph || it->graph->dataCount() == 0)
      continue;

    bool found_range = false;
    const QCPRange y_range = it->graph->data()->valueRange(
        found_range, QCP::sdBoth, QCPRange(min_x, max_x));
    if (!found_range)
      continue;

    if (!has_y)
    {
      min_y = y_range.lower;
      max_y = y_range.upper;
      has_y = true;
    }
    else
    {
      min_y = std::min(min_y, y_range.lower);
      max_y = std::max(max_y, y_range.upper);
    }
  }

  if (!has_y)
  {
    min_y = -1.0;
    max_y = 1.0;
  }

  if (max_x <= min_x)
  {
    this->plot_->xAxis->setRange(latest_x - 1.0, latest_x);
  }
  else
  {
    this->plot_->xAxis->setRange(min_x, max_x);
  }

  const double y_span = max_y - min_y;
  if (y_span < 1e-6)
  {
    min_y -= 1.0;
    max_y += 1.0;
  }
  else
  {
    const double padding = y_span * 0.12;
    min_y -= padding;
    max_y += padding;
  }

  this->plot_->yAxis->setRange(min_y, max_y);
}

void HypaQCustomPlotItem::updateSeriesRange(SeriesState &_state,
                                            const QVector<double> &_x,
                                            const QVector<double> &_y,
                                            bool _reset)
{
  if (_x.isEmpty() || _y.isEmpty())
    return;

  if (_reset || !_state.range_initialized)
  {
    _state.min_x = _x.front();
    _state.max_x = _x.front();
    _state.min_y = _y.front();
    _state.max_y = _y.front();
    _state.range_initialized = true;
  }

  for (int i = 0; i < _x.size(); ++i)
  {
    _state.min_x = std::min(_state.min_x, _x[i]);
    _state.max_x = std::max(_state.max_x, _x[i]);
    _state.min_y = std::min(_state.min_y, _y[i]);
    _state.max_y = std::max(_state.max_y, _y[i]);
  }
}

void HypaQCustomPlotItem::setHasData(bool _hasData)
{
  if (this->has_data_ == _hasData)
    return;

  this->has_data_ = _hasData;
  emit hasDataChanged();
}

}  // namespace hypa_dt_gui_plugin
