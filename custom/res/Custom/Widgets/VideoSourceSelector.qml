import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets
import QGroundControl.FactControls

import Custom.Ros

/// Video source selector for the fly-view video pane (VTOL-GCS).
///
/// One field decides where the video comes from — the ROS image topic bridge or
/// the stock RTSP/UDP stream — and the row below it shows whatever that choice
/// needs: the live topic list for ROS, or the stream type plus its address for
/// RTSP/UDP. The address facts are the same ones under
/// Application Settings > Video, so edits here persist there and vice versa.
Rectangle {
    id: root

    /// Two-way with the fly view: true = ROS topic feed, false = RTSP/UDP stream.
    property bool useRosSource: true

    color: FalconTheme.surface1
    radius: FalconTheme.radiusPanel
    border.color: FalconTheme.hairline
    border.width: 0

    readonly property real  _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color _accent: FalconTheme.accent
    readonly property color _mutedText: FalconTheme.textMuted

    readonly property var _videoSettings: QGroundControl.settingsManager.videoSettings
    readonly property string _sourceName: _videoSettings.videoSource.rawValue

    // Which address fact applies depends on the stream type the operator picked.
    // Sources such as "No Video Available" or the vendor presets carry no address,
    // in which case the field is hidden rather than bound to something unrelated.
    readonly property var _addressFact: {
        const name = _sourceName
        if (name.indexOf("RTSP") >= 0) {
            return _videoSettings.rtspUrl
        }
        if (name.indexOf("UDP") >= 0) {
            return _videoSettings.udpUrl
        }
        if (name.indexOf("TCP") >= 0) {
            return _videoSettings.tcpUrl
        }
        return null
    }

    implicitHeight: layout.implicitHeight + (_margin * 2)

    ColumnLayout {
        id: layout
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.margins: root._margin
        spacing: root._margin * 0.6

        // --- the selection field itself ---
        RowLayout {
            Layout.fillWidth: true
            spacing: root._margin

            QGCLabel {
                text: qsTr("영상 소스")
                color: root._mutedText
                font.pointSize: ScreenTools.smallFontPointSize
            }

            QGCComboBox {
                id: sourceCombo
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                model: [qsTr("ROS 토픽"), qsTr("RTSP / UDP")]
                currentIndex: root.useRosSource ? 0 : 1
                onActivated: root.useRosSource = (currentIndex === 0)
            }

            // Live-ness indicator for whichever source is selected.
            Rectangle {
                Layout.preferredWidth: statusLabel.implicitWidth + ScreenTools.defaultFontPixelWidth * 1.4
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.6
                radius: FalconTheme.radiusControl
                readonly property bool live: root.useRosSource
                                                 ? RosBridge.imageFps > 0
                                                 : QGroundControl.videoManager.decoding
                color: live ? Qt.rgba(0.13, 0.77, 0.37, 0.18) : Qt.rgba(0.96, 0.62, 0.04, 0.18)
                border.color: live ? FalconTheme.ok : "#F59E0B"
                border.width: 1

                QGCLabel {
                    id: statusLabel
                    anchors.centerIn: parent
                    text: root.useRosSource
                              ? (RosBridge.imageFps > 0 ? (RosBridge.imageFps + qsTr(" FPS")) : qsTr("WAIT"))
                              : (parent.live ? qsTr("LIVE") : qsTr("WAIT"))
                    color: parent.live ? "#86EFAC" : FalconTheme.caution
                    font.bold: true
                    font.pointSize: ScreenTools.smallFontPointSize
                }
            }
        }

        // --- ROS detail: the discovered topic list ---
        RowLayout {
            Layout.fillWidth: true
            visible: root.useRosSource
            spacing: root._margin

            QGCComboBox {
                id: topicCombo
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                model: RosBridge.imageTopics
                currentIndex: RosBridge.imageTopics.indexOf(RosBridge.imageTopic)
                alternateText: RosBridge.imageTopics.length === 0 ? qsTr("이미지 토픽 없음")
                             : RosBridge.imageTopic === ""        ? qsTr("토픽 선택")
                             : ""
                onActivated: RosBridge.setImageTopic(currentText)
            }

            // Re-scan the ROS graph for image topics (rqt-style refresh).
            QGCButton {
                Layout.preferredWidth: ScreenTools.defaultFontPixelHeight * 2.2
                text: "⟳"
                onClicked: RosBridge.refreshTopics()
            }
        }

        // --- stream detail: type + address ---
        RowLayout {
            Layout.fillWidth: true
            visible: !root.useRosSource
            spacing: root._margin

            FactComboBox {
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                fact: root._videoSettings.videoSource
                indexModel: false
            }

            FactTextField {
                Layout.fillWidth: true
                Layout.minimumWidth: 0
                visible: root._addressFact !== null
                fact: root._addressFact
            }

            QGCLabel {
                Layout.fillWidth: true
                visible: root._addressFact === null
                text: qsTr("주소 없음")
                color: root._mutedText
                font.pointSize: ScreenTools.smallFontPointSize
                elide: Text.ElideRight
            }
        }
    }
}
