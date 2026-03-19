import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import QtQuick.Dialogs 1.3

Rectangle {
  id: root
  anchors.fill: parent
  clip: true
  color: Material.backgroundColor

  Component.onCompleted: {
    // 设置深色主题
    Material.primary = "#2196F3"
    Material.accent = "#FF5722"
    Material.theme = Material.Dark
  }

  // 获取关节列表模型
  property var jointListModel: HypaStateDisplayPlugin.getJointListModel()

  // 关节数据缓存
  property var jointData: ({})
  property var imuData: ({
    roll: 0.0, pitch: 0.0, yaw: 0.0,
    linearAccelX: 0.0, linearAccelY: 0.0, linearAccelZ: 0.0,
    angularVelX: 0.0, angularVelY: 0.0, angularVelZ: 0.0
  })

  // 日志列表
  property var logMessages: []

  // 连接 C++ 信号
  Connections {
    target: HypaStateDisplayPlugin
    function onJointDataUpdated() {
      jointListView.model = jointListModel
    }
    function onImuDataUpdated() {
      imuScrollView.update()
    }
    function onLogMessageReceived(message) {
      logMessages.unshift(message)
      if (logMessages.length > 1000) logMessages.pop()
      logListView.model = logMessages.slice()
    }
  }

  // 主布局：使用 ColumnLayout 撑满 root
  ColumnLayout {
    anchors.fill: parent
    spacing: 0

    // 使用 ScrollView 包裹所有内容
    ScrollView {
      Layout.fillWidth: true
      Layout.fillHeight: true
      clip: true
      ScrollBar.vertical.policy: ScrollBar.AlwaysOn

      // 内容容器
      ColumnLayout {
        width: parent.width - 20 // 留出滚动条宽度
        spacing: 12
        anchors.margins: 10

        // ======================== 顶部：IMU 数据面板 ========================
        Rectangle {
          Layout.fillWidth: true
          height: 240
          color: "#2a2a2a"
          radius: 4
          border.color: Material.primary
          border.width: 1

          GridLayout {
            anchors.fill: parent
            anchors.margins: 12
            columns: 3
            columnSpacing: 8
            rowSpacing: 8

            Text {
              text: "IMU 数据"
              font.bold: true
              font.pixelSize: 16
              color: Material.primary
              Layout.columnSpan: 3
            }

            // 姿态角卡片
            Rectangle {
              Layout.fillWidth: true; height: 100; color: "#1e1e1e"; radius: 4
              ColumnLayout {
                anchors.fill: parent; anchors.margins: 8; spacing: 2
                Text { text: "姿态角 (°)"; font.bold: true; color: Material.primary }
                Text { text: "Roll: " + imuData.roll.toFixed(2); color: Material.foreground }
                Text { text: "Pitch: " + imuData.pitch.toFixed(2); color: Material.foreground }
                Text { text: "Yaw: " + imuData.yaw.toFixed(2); color: Material.foreground }
              }
            }
            // 线加速度卡片
            Rectangle {
              Layout.fillWidth: true; height: 100; color: "#1e1e1e"; radius: 4
              ColumnLayout {
                anchors.fill: parent; anchors.margins: 8; spacing: 2
                Text { text: "线加速度 (m/s²)"; font.bold: true; color: Material.primary }
                Text { text: "X: " + imuData.linearAccelX.toFixed(2); color: Material.foreground }
                Text { text: "Y: " + imuData.linearAccelY.toFixed(2); color: Material.foreground }
                Text { text: "Z: " + imuData.linearAccelZ.toFixed(2); color: Material.foreground }
              }
            }
            // 角速度卡片
            Rectangle {
              Layout.fillWidth: true; height: 100; color: "#1e1e1e"; radius: 4
              ColumnLayout {
                anchors.fill: parent; anchors.margins: 8; spacing: 2
                Text { text: "角速度 (rad/s)"; font.bold: true; color: Material.primary }
                Text { text: "X: " + imuData.angularVelX.toFixed(3); color: Material.foreground }
                Text { text: "Y: " + imuData.angularVelY.toFixed(3); color: Material.foreground }
                Text { text: "Z: " + imuData.angularVelZ.toFixed(3); color: Material.foreground }
              }
            }

            RowLayout {
              Layout.columnSpan: 3; Layout.fillWidth: true
              Text { text: "IMU 话题："; color: Material.primary }
              TextField {
                id: imuTopicInput
                text: HypaStateDisplayPlugin.getIMUTopic()
                Layout.fillWidth: true
                onEditingFinished: HypaStateDisplayPlugin.setIMUTopic(text)
              }
            }
          }
        }

        // ======================== 中部：关节面板 ========================
        Rectangle {
          Layout.fillWidth: true
          height: 300
          color: "#2a2a2a"
          radius: 4
          border.color: Material.primary
          border.width: 1

          ColumnLayout {
            anchors.fill: parent
            anchors.margins: 8
            spacing: 4

            RowLayout {
              Text { text: "关节状态"; font.bold: true; color: Material.primary }
              Button { text: "全选"; onClicked: HypaStateDisplayPlugin.selectAllJoints() }
              Button { text: "反选"; onClicked: HypaStateDisplayPlugin.deselectAllJoints() }
              Item { Layout.fillWidth: true }
            }

            ScrollView {
              Layout.fillWidth: true; Layout.fillHeight: true; clip: true
              ScrollBar.horizontal.policy: ScrollBar.AlwaysOff  // 禁用水平滚动条
              ListView {
                id: jointListView
                model: jointListModel
                clip: true
                delegate: Rectangle {
                  width: jointListView.width; height: 35; color: index % 2 === 0 ? "#252525" : "#1e1e1e"
                  RowLayout {
                    anchors.fill: parent; anchors.margins: 4; spacing: 4
                    CheckBox { checked: selected; onCheckedChanged: HypaStateDisplayPlugin.toggleJointSelection(name) }
                    Text { text: name; Layout.preferredWidth: 60; color: Material.primary; elide: Text.ElideRight }
                    Text { text: "P: " + position.toFixed(3); Layout.fillWidth: true; color: Material.foreground }
                    Text { text: "A: " + acceleration.toFixed(3); Layout.preferredWidth: 60; color: Material.foreground }
                  }
                }
              }
            }

            RowLayout {
              Text { text: "关节话题："; color: Material.primary }
              TextField {
                text: HypaStateDisplayPlugin.getJointStateTopic()
                Layout.fillWidth: true
                onEditingFinished: HypaStateDisplayPlugin.setJointStateTopic(text)
              }
            }
          }
        }

        // ======================== 中部：曲线图 ========================
        Rectangle {
          Layout.fillWidth: true
          height: 300
          color: "#2a2a2a"
          radius: 4
          border.color: Material.primary
          border.width: 1

          ColumnLayout {
            anchors.fill: parent
            anchors.margins: 8
            Text { text: "实时曲线图"; font.bold: true; color: Material.primary }
            Rectangle {
              Layout.fillWidth: true; Layout.fillHeight: true; color: "#1a1a1a"; radius: 4
              Text { anchors.centerIn: parent; text: "曲线图区域"; color: "gray" }
            }
            CheckBox { text: "启用曲线"; checked: true }
          }
        }

        // ======================== 底部：日志面板 ========================
        Rectangle {
          Layout.fillWidth: true
          height: 350
          color: "#2a2a2a"
          radius: 4
          border.color: Material.primary
          border.width: 1

          ColumnLayout {
            anchors.fill: parent
            spacing: 0

            TabBar {
              id: bottomTabBar; Layout.fillWidth: true; height: 35
              TabButton { text: "实时日志"; font.pixelSize: 11 }
              TabButton { text: "文件日志"; font.pixelSize: 11 }
              TabButton { text: "扩展功能"; font.pixelSize: 11 }
            }

            StackLayout {
              Layout.fillWidth: true; Layout.fillHeight: true
              currentIndex: bottomTabBar.currentIndex

              // -------- Tab 1: 实时日志 --------
              ColumnLayout {
                spacing: 4
                anchors.margins: 4

                // 过滤行
                RowLayout {
                  Layout.fillWidth: true
                  spacing: 4

                  Text { text: "等级:"; font.pixelSize: 11; color: Material.primary }
                  ComboBox {
                    id: logLevelFilter
                    model: ["ALL", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"]
                    font.pixelSize: 11; Layout.preferredWidth: 80
                  }
                  Text { text: "搜索:"; font.pixelSize: 11; color: Material.primary }
                  TextField {
                    id: logSearchInput
                    placeholderText: "输入关键词..."
                    font.pixelSize: 11; Layout.fillWidth: true
                  }
                  CheckBox { text: "自动滚动"; font.pixelSize: 11; checked: true }
                  Button {
                    text: "清空"; font.pixelSize: 11; Layout.preferredWidth: 50
                    onClicked: { logMessages = []; logListView.model = [] }
                  }
                }

                // 日志列表
                Rectangle {
                  Layout.fillWidth: true; Layout.fillHeight: true; color: "#0d0d0d"
                  border.color: Material.primary; border.width: 1; radius: 4

                  ScrollView {
                    anchors.fill: parent; anchors.margins: 1; clip: true
                    ListView {
                      id: logListView
                      model: logMessages
                      spacing: 1
                      delegate: Rectangle {
                        width: ListView.view.width; height: 24
                        color: {
                          if (modelData.includes("ERROR") || modelData.includes("FATAL")) return "#4b0000"
                          else if (modelData.includes("WARN")) return "#4b4b00"
                          else if (modelData.includes("DEBUG")) return "#001a4d"
                          else return "#1a1a1a"
                        }
                        Text {
                          anchors.fill: parent; anchors.margins: 4; text: modelData
                          font.pixelSize: 10; font.family: "Courier New"
                          color: {
                            if (modelData.includes("ERROR") || modelData.includes("FATAL")) return "#ff6666"
                            else if (modelData.includes("WARN")) return "#ffff66"
                            else if (modelData.includes("DEBUG")) return "#66b3ff"
                            else return "#ffffff"
                          }
                          elide: Text.ElideRight; verticalAlignment: Text.AlignVCenter
                        }
                      }
                    }
                  }
                }
              }

              // -------- Tab 2: 文件日志 --------
              ColumnLayout {
                spacing: 4
                anchors.margins: 4

                RowLayout {
                  Layout.fillWidth: true
                  spacing: 4
                  Button {
                    text: "选择日志文件..."
                    font.pixelSize: 11
                    onClicked: fileDialog.open()
                  }
                  TextField {
                    id: logFileInput
                    placeholderText: "选择 .log 或 .txt 文件..."
                    Layout.fillWidth: true
                    font.pixelSize: 11
                    readOnly: true
                  }
                  Button { text: "刷新"; font.pixelSize: 11; Layout.preferredWidth: 50 }
                }

                Rectangle {
                  Layout.fillWidth: true; Layout.fillHeight: true; color: "#0d0d0d"
                  border.color: Material.primary; border.width: 1; radius: 4
                  Text {
                    anchors.centerIn: parent; text: "选择文件后显示内容"
                    color: Material.foreground; opacity: 0.5
                  }
                }
              }

              // -------- Tab 3: 扩展功能 --------
              ColumnLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 8
                Rectangle {
                  Layout.fillWidth: true; Layout.fillHeight: true; color: "#1a1a1a"
                  border.color: Material.accent; border.width: 1; radius: 4
                  ColumnLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 8
                    Text { text: "扩展功能区域"; font.bold: true; font.pixelSize: 16; color: Material.accent }
                    Text {
                      text: "预留空间用于后续添加其他传感器模块（如激光雷达、摄像头等）"
                      font.pixelSize: 12; color: Material.foreground; wrapMode: Text.WordWrap; Layout.fillWidth: true
                    }
                    Item { Layout.fillHeight: true }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  FileDialog {
    id: fileDialog
    title: "选择日志文件"
    folder: StandardPaths.standardLocations(StandardPaths.HomeLocation)[0]
    nameFilters: ["日志文件 (*.log *.txt)", "所有文件 (*)"]
    onAccepted: logFileInput.text = fileUrl
  }
}
