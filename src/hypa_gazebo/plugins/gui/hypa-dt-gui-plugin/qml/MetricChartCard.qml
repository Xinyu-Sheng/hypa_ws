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

  property string titleText: ""
  property string chartKind: "imu"
  property var selectionModel
  property var dataProvider
  property var metrics: []
  property int dataRevision: 0
  property bool hasVisibleSeries: chartItem ? chartItem.hasData : false
  property var seriesModel: []
  // Refresh throttle settings (ms)
  property int refreshIntervalMs: 200
  property bool refreshPending: false

  radius: 12
  color: root.cardColor
  border.color: root.cardBorderColor
  border.width: 1
  clip: true

  Layout.fillWidth: true
  Layout.preferredHeight: contentLayout.implicitHeight + 16
  Layout.minimumWidth: 0
  implicitHeight: contentLayout.implicitHeight + 16

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
    console.log("MetricChartCard.rebuildSeriesModel: built seriesModel size=", items.length)
    Qt.callLater(root.refreshSeries)
  }

  function refreshSeries() {
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
        // throttle UI refresh: mark dirty and start timer if needed
        root.refreshPending = true
        if (!refreshTimer.running) refreshTimer.start()
      }
    }

    function onJointDataUpdated() {
      if (root.chartKind === "joint") {
        // throttle UI refresh: mark dirty and start timer if needed
        root.refreshPending = true
        if (!refreshTimer.running) refreshTimer.start()
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
        // rebuild model and refresh series on timer trigger
        root.rebuildSeriesModel()
        root.dataRevision += 1
        root.refreshSeries()
        root.refreshPending = false
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
    root.refreshSeries()
  }

  function _logSizes() {
    console.log("MetricChartCard:", titleText, "w=", root.width, "h=", root.height,
                "implicitW=", root.implicitWidth, "implicitH=", root.implicitHeight)
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
