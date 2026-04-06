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
    anchors.topMargin: 8
    clip: true

    ColumnLayout {
      width: parent.width - 10
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

   
            }

            Item {
              Layout.fillWidth: true
            }

            Text {
              text: "/hypa/imu/data"
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
            implicitHeight: 32
            clip: true

            Item {
              anchors.fill: parent
              anchors.margins: 2
              clip: true

              Flow {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                height: parent.height
                spacing: 8

                Repeater {
                  model: root.imuModel
                  delegate: CheckBox {
                    text: name
                    checked: selected
                    font.pixelSize: 12
                    hoverEnabled: true
                    ToolTip.visible: hovered
                    ToolTip.text: latestText
                    height: parent.height
                    onClicked: root.imuModel.setSelected(name, checked)
                  }
                }
              }
            }
          }

          ColumnLayout {
            Layout.fillWidth: true

            // Row 1: Acceleration (3 columns)
            GridLayout {
              Layout.fillWidth: true
              Layout.leftMargin: 12
              Layout.rightMargin: 12
              columns: root.width < 700 ? 1 : 3
              columnSpacing: 8
              rowSpacing: 8

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "加速度 X"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["linear_acceleration_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "加速度 Y"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["linear_acceleration_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "加速度 Z"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["linear_acceleration_z"]
              }
            }

            // Row 2: Angular velocity (3 columns)
            GridLayout {
              Layout.fillWidth: true
              Layout.leftMargin: 12
              Layout.rightMargin: 12
              columns: root.width < 700 ? 1 : 3
              columnSpacing: 8
              rowSpacing: 8

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "角速度 X"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["angular_velocity_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "角速度 Y"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["angular_velocity_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "角速度 Z"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["angular_velocity_z"]
              }
            }

            // Row 3: Orientation (quaternion) — force 4 columns so w/x/y/z are on one row
            GridLayout {
              Layout.fillWidth: true
              Layout.leftMargin: 12
              Layout.rightMargin: 12
              columns: root.width < 900 ? 1 : 4
              columnSpacing: 8
              rowSpacing: 8

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "四元数 w"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_w"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "四元数 x"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "四元数 y"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "四元数 z"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_z"]
              }
            }
          }

          // CovariancePanel removed; covariance values now shown inline in each MetricChartCard summary
        }

        implicitHeight: imuCardContent.implicitHeight + 10
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
    
            }

            Item {
              Layout.fillWidth: true
            }

            Text {
              text: "/hypa/joint_states"
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
            implicitHeight: 32
            clip: true

            Item {
              anchors.fill: parent
              anchors.margins: 2
              clip: true

              Flow {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                height: parent.height
                spacing: 8

                Repeater {
                  model: root.jointModel
                  delegate: CheckBox {
                    text: name
                    checked: selected
                    font.pixelSize: 12
                    hoverEnabled: true
                    ToolTip.visible: hovered
                    ToolTip.text: latestText
                    height: parent.height
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
              titleText: "电机位置"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["position"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              titleText: "电机速度"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["velocity"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              titleText: "电机力矩"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["effort"]
            }
          }
        }

        implicitHeight: jointCardContent.implicitHeight + 10
      }
    }
  }
}
