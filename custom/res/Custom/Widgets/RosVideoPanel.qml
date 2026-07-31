import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets

import Custom.Ros

/// rqt_image_view-style recognition-video panel. The dropdown lists live
/// `sensor_msgs/Image` topics from `RosBridge`; picking one switches the
/// subscription. `RosVideoView` renders the decoded frames, letterboxed.
Rectangle {
    id: root

    color: FalconTheme.surface1
    radius: FalconTheme.radiusPanel
    border.color: FalconTheme.hairline
    border.width: 0
    opacity: 0.98

    readonly property real _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color _accent: FalconTheme.accent
    readonly property color _panel: FalconTheme.surface2
    readonly property color _mutedText: FalconTheme.textMuted

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: root._margin
        spacing: root._margin

        // Source and topic selection live in VideoSourceSelector, which sits
        // above this panel and serves both video sources — keeping a second
        // topic dropdown here would be a duplicate control.

        // --- Video surface ---
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: FalconTheme.sunken
            radius: FalconTheme.radiusPanel
            border.color: FalconTheme.hairline
            border.width: 0
            clip: true

            RosVideoView {
                id: videoView
                anchors.fill: parent
            }

            // AR waypoint + path overlay, projected onto the live camera feed.
            ARWaypointOverlay {
                id: arOverlay
                anchors.fill: parent
                visible: RosBridge.imageFps > 0
            }

            QGCLabel {
                anchors.centerIn: parent
                visible: RosBridge.imageTopic === "" || RosBridge.imageFps === 0
                text: RosBridge.imageTopic === "" ? qsTr("위 '영상 소스'에서 카메라 토픽을 선택하세요")
                                                  : qsTr("%1 — 프레임 대기 중…").arg(RosBridge.imageTopic)
                color: root._mutedText
                font.pointSize: ScreenTools.smallFontPointSize
            }
        }
    }
}
