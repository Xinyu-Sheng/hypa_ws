import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import HypaDtGuiPluginBackend 1.0
import "qrc:/HypaDtGuiPlugin"

Rectangle {
  id: root

  property color pageBackground: "#eef3f9"
  property color sectionCardColor: "#fbfdff"
  property color sectionBorderColor: "#d5e0ec"
  property color selectorBgColor: "#f3f8fe"
  property color selectorBorderColor: "#cedbea"
  property color titleColor: "#16202b"

  anchors.fill: parent
  clip: true

  implicitWidth: 1280
  implicitHeight: 600
  
  // Keep a practical minimum width for multi-column charts
  Layout.minimumWidth: 1100

  color: root.pageBackground

  property var imuModel: Backend.imuSelectionModel
  property var jointModel: Backend.jointSelectionModel
  property var magModel: Backend.magSelectionModel

  Component.onCompleted: {
    Material.theme = Material.Light
    Material.primary = "#1668c7"
    Material.accent = "#e99b2f"
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
        color: root.sectionCardColor
        border.color: root.sectionBorderColor
        border.width: 1
        clip: true

        ColumnLayout {
          id: imuCardContent
          Layout.fillWidth: true
          spacing: 6

          RowLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            Layout.topMargin: 6
            Layout.bottomMargin: 2
            spacing: 4

            ColumnLayout {
              spacing: 2

              Text {
                text: "IMU Data"
                color: root.titleColor
                font.pixelSize: 18
                font.bold: true
              }

   
            }

            Item {
              Layout.preferredWidth: 14
              Layout.minimumWidth: 14
            }

            Rectangle {
              Layout.fillWidth: true
              Layout.preferredHeight: 30
              Layout.maximumHeight: 30
              Layout.minimumWidth: 260
              radius: 10
              color: root.selectorBgColor
              border.color: root.selectorBorderColor
              border.width: 1
              clip: true

              Flow {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                anchors.leftMargin: 8
                anchors.rightMargin: 8
                spacing: 6

                Repeater {
                  model: root.imuModel
                  delegate: CheckBox {
                    text: name
                    checked: selected
                    font.pixelSize: 11
                    hoverEnabled: true
                    ToolTip.visible: hovered
                    ToolTip.text: latestText
                    topPadding: 0
                    bottomPadding: 0
                    indicator.height: 14
                    indicator.width: 14
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
                titleText: "Accel X"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["linear_acceleration_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Accel Y"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["linear_acceleration_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Accel Z"
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
                titleText: "Gyro X"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["angular_velocity_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Gyro Y"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["angular_velocity_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Gyro Z"
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
                titleText: "Quat w"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_w"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Quat x"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Quat y"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Quat z"
                chartKind: "imu"
                selectionModel: root.imuModel
                dataProvider: Backend
                metrics: ["orientation_z"]
              }
            }

            // Row 4: Magnetic field (3 columns)
            GridLayout {
              Layout.fillWidth: true
              Layout.leftMargin: 12
              Layout.rightMargin: 12
              columns: root.width < 700 ? 1 : 3
              columnSpacing: 8
              rowSpacing: 8

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Mag X"
                chartKind: "mag"
                selectionModel: root.magModel
                dataProvider: Backend
                yScale: 5.0
                metrics: ["magnetic_field_x"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Mag Y"
                chartKind: "mag"
                selectionModel: root.magModel
                dataProvider: Backend
                yScale: 5.0
                metrics: ["magnetic_field_y"]
              }

              MetricChartCard {
                Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
                titleText: "Mag Z"
                chartKind: "mag"
                selectionModel: root.magModel
                dataProvider: Backend
                yScale: 5.0
                metrics: ["magnetic_field_z"]
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
        color: root.sectionCardColor
        border.color: root.sectionBorderColor
        border.width: 1
        clip: true

        ColumnLayout {
          id: jointCardContent
          Layout.fillWidth: true
          spacing: 6

          RowLayout {
            Layout.fillWidth: true
            Layout.leftMargin: 12
            Layout.rightMargin: 12
            Layout.topMargin: 6
            Layout.bottomMargin: 2
            spacing: 4

            ColumnLayout {
              spacing: 2

              Text {
                text: "Joint Data"
                color: root.titleColor
                font.pixelSize: 18
                font.bold: true
              }
    
            }

            Item {
              Layout.preferredWidth: 14
              Layout.minimumWidth: 14
            }

            Rectangle {
              Layout.fillWidth: true
              Layout.preferredHeight: 30
              Layout.maximumHeight: 30
              Layout.minimumWidth: 260
              radius: 10
              color: root.selectorBgColor
              border.color: root.selectorBorderColor
              border.width: 1
              clip: true

              Flow {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                anchors.leftMargin: 8
                anchors.rightMargin: 8
                spacing: 6

                Repeater {
                  model: root.jointModel
                  delegate: CheckBox {
                    text: name
                    checked: selected
                    font.pixelSize: 11
                    hoverEnabled: true
                    ToolTip.visible: hovered
                    ToolTip.text: latestText
                    topPadding: 0
                    bottomPadding: 0
                    indicator.height: 14
                    indicator.width: 14
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
              titleText: "Joint Position"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["position"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              titleText: "Joint Velocity"
              chartKind: "joint"
              selectionModel: root.jointModel
              dataProvider: Backend
              metrics: ["velocity"]
            }

            MetricChartCard {
              Layout.preferredWidth: (root.width - 32 - (parent.columnSpacing * (parent.columns - 1))) / parent.columns
              titleText: "Joint Effort"
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
