import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

import Custom.Ros

/// rqt_image_view-style recognition-video panel. The dropdown lists live
/// `sensor_msgs/Image` topics from `RosBridge`; picking one switches the
/// subscription. `RosVideoView` renders the decoded frames, letterboxed.
Rectangle {
    id: root

    color: qgcPal.window
    radius: 4
    border.color: qgcPal.groupBorder
    border.width: 1
    opacity: 0.95

    readonly property real _margin: ScreenTools.defaultFontPixelWidth * 0.75

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: root._margin
        spacing: root._margin

        // --- Topic selector row (rqt style) ---
        RowLayout {
            Layout.fillWidth: true
            spacing: root._margin

            QGCLabel {
                text: qsTr("Camera")
                font.pointSize: ScreenTools.smallFontPointSize
            }

            QGCComboBox {
                id: topicCombo
                Layout.fillWidth: true
                model: RosBridge.imageTopics
                currentIndex: RosBridge.imageTopics.indexOf(RosBridge.imageTopic)
                // alternateText overrides the shown text whenever it is non-empty,
                // so clear it once a topic is picked to show the topic name itself.
                alternateText: RosBridge.imageTopics.length === 0 ? qsTr("no image topics")
                             : RosBridge.imageTopic === ""        ? qsTr("select topic")
                             : ""
                onActivated: RosBridge.setImageTopic(currentText)
            }

            QGCLabel {
                text: RosBridge.imageTopic === "" ? "" : (RosBridge.imageFps + qsTr(" fps"))
                color: qgcPal.buttonHighlight
                font.pointSize: ScreenTools.smallFontPointSize
            }
        }

        // --- Video surface ---
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "black"
            radius: 2
            clip: true

            RosVideoView {
                id: videoView
                anchors.fill: parent
            }

            QGCLabel {
                anchors.centerIn: parent
                visible: RosBridge.imageTopic === "" || RosBridge.imageFps === 0
                text: RosBridge.imageTopic === "" ? qsTr("Select a camera topic")
                                                  : qsTr("Waiting for frames…")
                color: "white"
                font.pointSize: ScreenTools.smallFontPointSize
            }
        }
    }
}
