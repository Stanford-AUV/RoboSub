import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Rectangle {
  color: "transparent"
  anchors.fill: parent
  Layout.minimumWidth: 250
  Layout.minimumHeight: 100
  Button {
    text: qsTr("Show Path!")
    onClicked: {
      PathVisualizer.TogglePath();
    }
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.verticalCenter: parent.verticalCenter
  }
  Text {
    text: qsTr("Number of Points: " + PathVisualizer.pathData.poses_size())
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: parent.top
    anchors.topMargin: 30
  }
}
