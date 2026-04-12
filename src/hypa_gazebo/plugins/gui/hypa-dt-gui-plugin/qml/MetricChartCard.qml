import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import HypaDtGuiPluginBackend 1.0

Rectangle {
  id: root

  property color cardColor: "#ffffff"
  property color cardBorderColor: "#d5e0ec"
  property color panelColor: "#f8fbff"
  property color panelBorderColor: "#d9e4f1"
  property color hintColor: "#54667b"
  property color cardGradientTop: "#ffffff"
  property color cardGradientBottom: "#f7fbff"
  property color panelGradientTop: "#fcfeff"
  property color panelGradientBottom: "#f2f8ff"
  property color summaryGradientTop: "#f6faff"
  property color summaryGradientBottom: "#ecf4ff"
  property color summaryRowGradientTop: "#ffffff"
  property color summaryRowGradientBottom: "#f6faff"

  property string titleText: ""
  property string chartKind: "imu"
  property var selectionModel
  property var dataProvider
  property var metrics: []
  property int dataRevision: 0
  property real yScale: 2.0
  property bool autoYRange: true
  property real yMin: NaN
  property real yMax: NaN
  property bool hasVisibleSeries: chartItem ? chartItem.hasData : false
  property var seriesModel: []
  property bool debugEnabled: false
  // Refresh throttle settings (ms)
  property int refreshIntervalMs: 300
  // 每个图表在同一刷新周期内错峰，避免同一时刻批量重绘造成卡顿
  property int refreshSpreadMs: Math.max(1, Math.floor(root.refreshIntervalMs / 2))
  property int refreshPhaseMs: root._phaseForCard()
  property bool refreshPending: false

  radius: 12
  color: root.cardColor
  gradient: Gradient {
    orientation: Gradient.Vertical
    GradientStop { position: 0.0; color: root.cardGradientTop }
    GradientStop { position: 1.0; color: root.cardGradientBottom }
  }
  border.color: root.cardBorderColor
  border.width: 1
  clip: true

  Layout.fillWidth: true
  Layout.preferredHeight: contentLayout.implicitHeight + 16
  Layout.minimumWidth: 0
  implicitHeight: contentLayout.implicitHeight + 16

  function _hashString(text) {
    var h = 0
    for (var i = 0; i < text.length; ++i)
      h = ((h * 31) + text.charCodeAt(i)) & 0x7fffffff
    return h
  }

  function _phaseForCard() {
    var key = root.titleText + "|" + root.chartKind
    return root._hashString(key) % root.refreshSpreadMs
  }

  function _delayToNextPhaseMs() {
    var interval = Math.max(1, root.refreshIntervalMs)
    var phase = root.refreshPhaseMs % interval
    var nowMod = Date.now() % interval
    var delay = phase - nowMod
    if (delay < 0)
      delay += interval
    return delay
  }

  function rebuildSeriesModel() {
    var items = []
    if (root.selectionModel) {
      var total = root.selectionModel.count()
      for (var i = 0; i < total; ++i) {
        var item = root.selectionModel.get(i)
        for (var j = 0; j < root.metrics.length; ++j) {
          items.push({
            sourceName: item.name,
            metricName: root.metrics[j],
            sourceSelected: item.selected
          })
        }
      }
    }

    root.seriesModel = items
    if (root.debugEnabled)
      console.log("MetricChartCard.rebuildSeriesModel: built seriesModel size=", items.length)
    root.requestRefresh()
  }

  function requestRefresh() {
    // Unified refresh gate: all chart updates must pass through this timer.
    root.refreshPending = true
    if (!refreshTimer.running) {
      refreshTimer.interval = root._delayToNextPhaseMs()
      refreshTimer.start()
    }
  }

  function refreshSeries() {
    if (root.debugEnabled)
      console.log("MetricChartCard.refreshSeries: start", "chartKind=", root.chartKind)
    if (!root.dataProvider || !chartItem)
      return
    chartItem.refreshPlot()
    root.hasVisibleSeries = chartItem.hasData
  }

  Connections {
    target: root.selectionModel ? root.selectionModel : null
    ignoreUnknownSignals: true
    function onEntriesChanged() {
      root.rebuildSeriesModel()
    }
  }

  Connections {
    target: root.dataProvider ? root.dataProvider : null
    ignoreUnknownSignals: true
    function onImuDataUpdated() {
      if (root.chartKind === "imu") {
        root.requestRefresh()
      }
    }

    function onJointDataUpdated() {
      if (root.chartKind === "joint") {
        root.requestRefresh()
      }
    }

    function onMagDataUpdated() {
      if (root.chartKind === "mag") {
        root.requestRefresh()
      }
    }
  }

  Timer {
    id: refreshTimer
    interval: root.refreshIntervalMs
    repeat: false
    running: false
    onTriggered: {
      if (root.refreshPending) {
        // only refresh data; series model is rebuilt by selection-change events
        root.refreshPending = false
        root.dataRevision += 1
        root.refreshSeries()
      }
    }
  }

  Component.onCompleted: {
    root.rebuildSeriesModel()
    _logSizes()
  }

  onSelectionModelChanged: {
    root.rebuildSeriesModel()
  }

  onDataProviderChanged: {
    root.requestRefresh()
  }

  function _logSizes() {
    if (root.debugEnabled) {
      console.log("MetricChartCard:", titleText, "w=", root.width, "h=", root.height,
                  "implicitW=", root.implicitWidth, "implicitH=", root.implicitHeight)
    }
  }

  onWidthChanged: _logSizes()
  onHeightChanged: _logSizes()

  ColumnLayout {
    id: contentLayout
    anchors.fill: parent
    anchors.margins: 4
    spacing: 1

    RowLayout {
      Layout.fillWidth: true
      spacing: 6

      Text {
        text: root.titleText
        color: "#1b2633"
        font.pixelSize: 14
        font.bold: true
        elide: Text.ElideRight
      }

      Item {
        Layout.fillWidth: true
      }

      Text {
        text: root.chartKind === "imu" ? "IMU" : "Motor"
        color: "#67788c"
        font.pixelSize: 10
      }
    }

    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 110
      radius: 8
      color: root.panelColor
      gradient: Gradient {
        orientation: Gradient.Vertical
        GradientStop { position: 0.0; color: root.panelGradientTop }
        GradientStop { position: 1.0; color: root.panelGradientBottom }
      }
      border.color: root.panelBorderColor
      border.width: 1
      clip: true

      HypaQCustomPlotItem {
        id: chartItem
        anchors.fill: parent
        anchors.margins: 1
        chartKind: root.chartKind
        seriesModel: root.seriesModel
        dataProvider: root.dataProvider
        autoYRange: root.autoYRange
        yScale: root.yScale
        yMin: root.yMin
        yMax: root.yMax

        Text {
          anchors.centerIn: parent
          visible: !chartItem.hasData
          text: "Waiting for data or selection"
          color: root.hintColor
          font.pixelSize: 12
        }
      }
    }

    Rectangle {
      id: summaryRect
      Layout.fillWidth: true
      // let content define height, but cap to avoid excessive growth
      property int summaryMaxHeight: 92
      Layout.preferredHeight: Math.min(summaryContent.implicitHeight + 4, summaryMaxHeight)
      implicitHeight: Math.min(summaryContent.implicitHeight + 4, summaryMaxHeight)
      radius: 8
      color: "#f2f7fd"
      gradient: Gradient {
        orientation: Gradient.Vertical
        GradientStop { position: 0.0; color: root.summaryGradientTop }
        GradientStop { position: 1.0; color: root.summaryGradientBottom }
      }
      border.color: root.panelBorderColor
      border.width: 1
      clip: true

      ScrollView {
        id: summaryScroll
        anchors.fill: parent
        anchors.margins: 1
        clip: true

        ColumnLayout {
          id: summaryContent
          width: parent.width
          spacing: 1

          Repeater {
            model: root.selectionModel
            delegate: Rectangle {
              visible: selected
              Layout.fillWidth: true
              height: 22
              radius: 6
              color: "#fbfdff"
              gradient: Gradient {
                orientation: Gradient.Vertical
                GradientStop { position: 0.0; color: root.summaryRowGradientTop }
                GradientStop { position: 1.0; color: root.summaryRowGradientBottom }
              }
              border.color: "#d4dfec"
              border.width: 1

              RowLayout {
                anchors.fill: parent
                anchors.margins: 2
                spacing: 4

                Text {
                  text: name
                  color: Material.primary
                  font.pixelSize: 12
                  font.bold: true
                  elide: Text.ElideRight
                }

                Item { Layout.fillWidth: true }

                Text {
                  id: summaryText
                  text: root.dataRevision >= 0
                        ? root.dataProvider.formatSeriesSummary(root.chartKind, name, root.metrics)
                        : ""
                  color: Material.foreground
                  font.pixelSize: 12
                  font.bold: true
                  elide: Text.ElideRight
                }
              }
            }
          }
        }
      }
    }
  }
}
