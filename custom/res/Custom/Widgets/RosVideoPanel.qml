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

    color: Qt.rgba(0.03, 0.08, 0.14, 0.94)
    radius: 6
    border.color: Qt.rgba(0.22, 0.74, 0.97, 0.70)
    border.width: 1
    opacity: 0.98

    readonly property real _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color _accent: "#38BDF8"
    readonly property color _panel: "#0B1D33"
    readonly property color _mutedText: "#94A3B8"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: root._margin
        spacing: root._margin

        // --- Recognition feed header ---
        ColumnLayout {
            Layout.fillWidth: true
            spacing: root._margin * 0.35

            RowLayout {
                Layout.fillWidth: true
                spacing: root._margin

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 0

                    QGCLabel {
                        text: qsTr("AI VISION FEED")
                        color: "white"
                        font.bold: true
                    }

                    QGCLabel {
                        text: qsTr("표적 탐지 카메라")
                        color: root._accent
                        font.pointSize: ScreenTools.smallFontPointSize
                    }
                }

                Rectangle {
                    Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 6.5
                    Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.6
                    radius: 4
                    color: RosBridge.imageFps > 0 ? Qt.rgba(0.13, 0.77, 0.37, 0.18)
                                                  : Qt.rgba(0.96, 0.62, 0.04, 0.18)
                    border.color: RosBridge.imageFps > 0 ? "#22C55E" : "#F59E0B"
                    border.width: 1

                    QGCLabel {
                        anchors.centerIn: parent
                        text: RosBridge.imageFps > 0 ? (RosBridge.imageFps + qsTr(" FPS")) : qsTr("WAIT")
                        color: RosBridge.imageFps > 0 ? "#86EFAC" : "#FCD34D"
                        font.bold: true
                        font.pointSize: ScreenTools.smallFontPointSize
                    }
                }
            }

            QGCComboBox {
                id: topicCombo
                Layout.fillWidth: true
                model: RosBridge.imageTopics
                currentIndex: RosBridge.imageTopics.indexOf(RosBridge.imageTopic)
                alternateText: RosBridge.imageTopics.length === 0 ? qsTr("no image topics")
                             : RosBridge.imageTopic === ""        ? qsTr("select topic")
                             : ""
                onActivated: RosBridge.setImageTopic(currentText)
            }
        }

        // --- Video surface ---
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#020617"
            radius: 5
            border.color: Qt.rgba(0.22, 0.74, 0.97, 0.35)
            border.width: 1
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
                color: root._mutedText
                font.pointSize: ScreenTools.smallFontPointSize
            }
        }
    }
}
