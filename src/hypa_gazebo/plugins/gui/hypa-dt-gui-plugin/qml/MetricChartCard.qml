import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import QtCharts 2.15

Rectangle {
  id: root

  property string titleText: ""
  property string chartKind: "imu"
  property var selectionModel
  property var dataProvider
  property var metrics: []
  property int dataRevision: 0
  property bool hasVisibleSeries: false
  property var seriesRegistry: ({})
  property var seriesModel: []

  radius: 16
  color: "#ffffff"
  border.color: "#e6edf3"
  border.width: 1
  clip: true

  Layout.fillWidth: true
  Layout.preferredHeight: 200
  Layout.minimumWidth: 0
  implicitHeight: 200

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

    root.seriesRegistry = ({})
    root.seriesModel = items
    Qt.callLater(root.refreshSeries)
  }

  function registerSeries(_sourceName, _metricName, _series) {
    root.seriesRegistry[_sourceName + "|" + _metricName] = _series
  }

  function unregisterSeries(_sourceName, _metricName) {
    delete root.seriesRegistry[_sourceName + "|" + _metricName]
  }

  function refreshSeries() {
    if (!root.dataProvider) {
      root.hasVisibleSeries = false
      return
    }
    var foundPoint = false
    var minX = 0.0
    var maxX = 1.0
    var minY = -1.0
    var maxY = 1.0
    var initialized = false

    for (var key in root.seriesRegistry) {
      var series = root.seriesRegistry[key]
      if (!series)
        continue

      if (!series.visible) {
        series.clear()
        continue
      }

      var parts = key.split("|")
      if (parts.length !== 2)
        continue

      var points = root.dataProvider.getSeriesPoints(root.chartKind, parts[0], parts[1])
      series.clear()

      for (var i = 0; i < points.length; ++i) {
        var point = points[i]
        series.append(point.x, point.y)

        if (!initialized) {
          minX = point.x
          maxX = point.x
          minY = point.y
          maxY = point.y
          initialized = true
        } else {
          if (point.x < minX)
            minX = point.x
          if (point.x > maxX)
            maxX = point.x
          if (point.y < minY)
            minY = point.y
          if (point.y > maxY)
            maxY = point.y
        }
      }

      if (points.length > 0)
        foundPoint = true
    }

    root.hasVisibleSeries = foundPoint

    if (initialized) {
      if (minX === maxX) {
        minX -= 1.0
        maxX += 1.0
      }

      var ySpan = maxY - minY
      if (ySpan < 0.0001) {
        minY -= 1.0
        maxY += 1.0
      } else {
        var padding = ySpan * 0.12
        minY -= padding
        maxY += padding
      }

      xAxis.min = minX
      xAxis.max = maxX
      yAxis.min = minY
      yAxis.max = maxY
    } else {
      xAxis.min = 0.0
      xAxis.max = 1.0
      yAxis.min = -1.0
      yAxis.max = 1.0
    }
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
        root.dataRevision += 1
        root.refreshSeries()
      }
    }

    function onJointDataUpdated() {
      if (root.chartKind === "joint") {
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
    root.refreshSeries()
  }

  function _logSizes() {
    console.log("MetricChartCard:", titleText, "w=", root.width, "h=", root.height,
                "implicitW=", root.implicitWidth, "implicitH=", root.implicitHeight)
  }

  onWidthChanged: _logSizes()
  onHeightChanged: _logSizes()

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 8
    spacing: 6

    RowLayout {
      Layout.fillWidth: true
      spacing: 6

      Text {
        text: root.titleText
        color: Material.foreground
        font.pixelSize: 14
        font.bold: true
        elide: Text.ElideRight
      }

      Item {
        Layout.fillWidth: true
      }

      Text {
        text: root.chartKind === "imu" ? "IMU" : "Motor"
        color: "#6b7280"
        font.pixelSize: 10
      }
    }

    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 150
      radius: 12
      color: "#ffffff"
      border.color: "#e6edf3"
      border.width: 1
      clip: true

      ChartView {
        id: chartView
        anchors.fill: parent
        anchors.margins: 4
        backgroundColor: "transparent"
        legend.visible: false
        animationOptions: ChartView.NoAnimation
        antialiasing: false
        dropShadowEnabled: false

        ValueAxis {
          id: xAxis
          min: 0.0
          max: 1.0
          labelsColor: "#6b7280"
          gridLineColor: "#e6edf3"
          lineVisible: false
          tickCount: 5
        }

        ValueAxis {
          id: yAxis
          min: -1.0
          max: 1.0
          labelsColor: "#6b7280"
          gridLineColor: "#e6edf3"
          lineVisible: false
          tickCount: 5
        }

        Repeater {
          model: root.seriesModel
          delegate: LineSeries {
            id: lineSeries
            property string sourceName: modelData.sourceName
            property string metricName: modelData.metricName
            property bool sourceSelected: modelData.sourceSelected
            name: sourceName + " " + metricName
            axisX: xAxis
            axisY: yAxis
            visible: sourceSelected
            width: 2
            useOpenGL: true

            Component.onCompleted: {
              root.registerSeries(sourceName, metricName, lineSeries)
            }

            Component.onDestruction: {
              root.unregisterSeries(sourceName, metricName)
            }
          }
        }

        Text {
          anchors.centerIn: parent
          visible: !root.hasVisibleSeries
          text: "等待数据或选择对象"
          color: "#475569"
          font.pixelSize: 12
        }
      }
    }

    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 70
      radius: 12
      color: "#f6f8fa"
      border.color: "#e6edf3"
      border.width: 1
      clip: true

      ScrollView {
        id: summaryScroll
        anchors.fill: parent
        anchors.margins: 4
        clip: true

        Flow {
          spacing: 8

          Repeater {
            model: root.selectionModel
            delegate: Rectangle {
              visible: selected
              width: 140
              height: 86
              radius: 8
              color: "#ffffff"
              border.color: "#e6edf3"
              border.width: 1

              ColumnLayout {
                anchors.fill: parent
                anchors.margins: 6
                spacing: 4

                Text {
                  Layout.fillWidth: true
                  text: name
                  color: Material.primary
                  font.pixelSize: 11
                  font.bold: true
                  elide: Text.ElideRight
                }

                Text {
                  id: summaryText
                  Layout.fillWidth: true
                  text: root.dataRevision >= 0
                        ? root.dataProvider.formatSeriesSummary(root.chartKind, name, root.metrics)
                        : ""
                  color: Material.foreground
                  font.pixelSize: 10
                  wrapMode: Text.WordWrap
                }

                RowLayout {
                  Layout.fillWidth: true
                  spacing: 6
                  visible: root.chartKind === "imu"

                  property var oriCov: root.dataRevision >= 0 ? root.dataProvider.getImuCovariance(name, "orientation_covariance") : []
                  property var angCov: root.dataRevision >= 0 ? root.dataProvider.getImuCovariance(name, "angular_velocity_covariance") : []
                  property var linCov: root.dataRevision >= 0 ? root.dataProvider.getImuCovariance(name, "linear_acceleration_covariance") : []

                  Text {
                    text: oriCov.length >= 9 ? ("ori: " + Number(oriCov[0]).toFixed(4) + "," + Number(oriCov[4]).toFixed(4) + "," + Number(oriCov[8]).toFixed(4)) : ""
                    color: "#475569"
                    font.pixelSize: 9
                    elide: Text.ElideRight
                  }

                  Text {
                    text: angCov.length >= 9 ? ("ang: " + Number(angCov[0]).toFixed(4) + "," + Number(angCov[4]).toFixed(4) + "," + Number(angCov[8]).toFixed(4)) : ""
                    color: "#475569"
                    font.pixelSize: 9
                    elide: Text.ElideRight
                  }

                  Text {
                    text: linCov.length >= 9 ? ("acc: " + Number(linCov[0]).toFixed(4) + "," + Number(linCov[4]).toFixed(4) + "," + Number(linCov[8]).toFixed(4)) : ""
                    color: "#475569"
                    font.pixelSize: 9
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
}
