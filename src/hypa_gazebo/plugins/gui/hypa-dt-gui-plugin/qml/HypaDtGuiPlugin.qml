import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import HypaDtGuiPluginBackend 1.0
import "qrc:/HypaDtGuiPlugin"

Rectangle {
  id: root

  anchors.fill: parent
  clip: true

  implicitWidth: 1280
  implicitHeight: 600
  
  // 添加最小宽度设置
  Layout.minimumWidth: 1000

  color: "#ffffff"

  property var imuModel: Backend.imuSelectionModel
  property var jointModel: Backend.jointSelectionModel

  Component.onCompleted: {
    Material.theme = Material.Light
    Material.primary = "#0288d1"
    Material.accent = "#fb8c00"
  }

  ScrollView {
    anchors.fill: parent
    clip: true

    ColumnLayout {
      width: parent.width - 32
      anchors.horizontalCenter: parent.horizontalCenter
      spacing: 8

      Rectangle {
        Layout.fillWidth: true
        radius: 18
        color: "#ffffff"
        border.color: "#e6edf3"
        border.width: 1
        clip: true

        ColumnLayout {
          id: imuCardContent
          Layout.fillWidth: true
          spacing: 8

          RowLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            spacing: 6

            ColumnLayout {
              spacing: 2

              Text {
                text: "IMU 数据"
                color: Material.foreground
                font.pixelSize: 18
                font.bold: true
              }

              Text {
                text: "/hypa/imu/data"
                color: "#6b7280"
                font.pixelSize: 11
              }
            }

            Item {
              Layout.fillWidth: true
            }

            Text {
              text: "三图水平排布，竖屏整体布局"
              color: "#6b7280"
              font.pixelSize: 11
            }
          }

          Rectangle {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            radius: 12
            color: "#f6f8fa"
            border.color: "#e6edf3"
            border.width: 1
            implicitHeight: 48
            clip: true

            ScrollView {
              anchors.fill: parent
              anchors.margins: 6
              clip: true

              Flow {
                width: parent.width
                spacing: 8

                Repeater {
                  model: root.imuModel
                  delegate: CheckBox {
                    text: name
                    checked: selected
                    font.pixelSize: 11
                    hoverEnabled: true
                    ToolTip.visible: hovered
                    ToolTip.text: latestText
                    onClicked: root.imuModel.setSelected(name, checked)
                  }
                }
              }
            }
          }

          GridLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            columns: root.width < 700 ? 1 : 3
            columnSpacing: 8
            rowSpacing: 8

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              Layout.preferredHeight: 200
              titleText: "三轴加速度"
              chartKind: "imu"
              selectionModel: root.imuModel
              dataProvider: Backend
              metrics: ["linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              Layout.preferredHeight: 200
              titleText: "三轴角速度"
              chartKind: "imu"
              selectionModel: root.imuModel
              dataProvider: Backend
              metrics: ["angular_velocity_x", "angular_velocity_y", "angular_velocity_z"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              Layout.preferredHeight: 200
              titleText: "四元数 (w/x/y/z)"
              chartKind: "imu"
              selectionModel: root.imuModel
              dataProvider: Backend
              metrics: ["orientation_w", "orientation_x", "orientation_y", "orientation_z"]
            }
          }

          // CovariancePanel removed; covariance values now shown inline in each MetricChartCard summary
        }

        implicitHeight: imuCardContent.implicitHeight + 28
      }

      Rectangle {
        Layout.fillWidth: true
        radius: 18
        color: "#ffffff"
        border.color: "#e6edf3"
        border.width: 1
        clip: true

        ColumnLayout {
          id: jointCardContent
          Layout.fillWidth: true
          spacing: 8

          RowLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            spacing: 6

            ColumnLayout {
              spacing: 2

              Text {
                text: "电机关节数据"
                color: Material.foreground
                font.pixelSize: 18
                font.bold: true
              }

              Text {
                text: "/hypa/joint_states"
                color: "#6b7280"
                font.pixelSize: 11
              }
            }

            Item {
              Layout.fillWidth: true
            }

            Text {
              text: "自动识别电机数量与名称"
              color: "#6b7280"
              font.pixelSize: 11
            }
          }

          Rectangle {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            radius: 12
            color: "#f6f8fa"
            border.color: "#e6edf3"
            border.width: 1
            implicitHeight: 48
            clip: true

            ScrollView {
              anchors.fill: parent
              anchors.margins: 6
              clip: true

              Flow {
                width: parent.width
                spacing: 8

                Repeater {
                  model: root.jointModel
                  delegate: CheckBox {
                    text: name
                    checked: selected
                    font.pixelSize: 11
                    hoverEnabled: true
                    ToolTip.visible: hovered
                    ToolTip.text: latestText
                    onClicked: root.jointModel.setSelected(name, checked)
                  }
                }
              }
            }
          }

          GridLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            columns: root.width < 700 ? 1 : 3
            columnSpacing: 8
            rowSpacing: 8

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              Layout.preferredHeight: 200
              titleText: "电机位置"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["position"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              Layout.preferredHeight: 200
              titleText: "电机速度"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["velocity"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              Layout.preferredHeight: 200
              titleText: "电机力矩"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["effort"]
            }
          }
        }

        implicitHeight: jointCardContent.implicitHeight + 28
      }
    }
  }
}
