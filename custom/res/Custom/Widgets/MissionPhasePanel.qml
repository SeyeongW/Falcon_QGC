import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

import Custom.Ros

/// Mission phase orchestrator panel (VTOL-GCS).
///
/// Shows the dynamic phase catalog published by the onboard mission computer.
/// Clicking any idle row publishes its catalog id on `command/run_phase`; the
/// onboard orchestrator runs the matching local script and streams live status
/// back. Completed phases remain available for independent repeated execution.
Rectangle {
    id: root

    color: Qt.rgba(0.03, 0.08, 0.14, 0.94)
    radius: 6
    border.color: Qt.rgba(0.22, 0.74, 0.97, 0.70)
    border.width: 1
    opacity: 0.98

    readonly property real   _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color  _accent: "#38BDF8"
    readonly property color  _accentBlue: "#1D4ED8"
    readonly property color  _panel: "#0B1D33"
    readonly property color  _mutedText: "#94A3B8"

    // Dynamic metadata from the MC's command/catalog topic. Catalog ids may be
    // sparse (for example, a standalone gripper demo can use id 100), so the
    // delegate must use modelData.id rather than its visual list index.
    readonly property var _phases: RosBridge.phaseCatalog

    // --- live orchestrator status (from RosBridge / command/status) ---
    readonly property bool   _linkOk:     RosBridge.phaseLinkOk
    readonly property string _state:      RosBridge.phaseState        // idle|running|done|failed
    readonly property int    _activePhase: RosBridge.phase
    readonly property var    _done:       RosBridge.phaseDone
    readonly property bool   _busy:       _state === "running"

    // Connection state machine: attempt to reach the orchestrator for up to 60 s;
    // if command/status never arrives, surface a red "연결 실패" + a retry button.
    // Recovers automatically if status resumes (link flips back to connected).
    property bool _attempting: true
    readonly property bool _connFailed: !_linkOk && !_attempting

    Timer {
        id: connectTimer
        interval: 60000     // 1 minute
        repeat: false
        running: true
        onTriggered: root._attempting = false
    }

    function _retry() {
        RosBridge.retryPhaseLink()
        root._attempting = true
        connectTimer.restart()
    }

    function _isDone(phaseId)    { return _done.indexOf(phaseId) >= 0 }
    function _isRunning(phaseId) { return _busy && _activePhase === phaseId }
    function _clickable(phaseId) { return _linkOk && !_busy }

    implicitHeight: layout.implicitHeight + (_margin * 2)

    ColumnLayout {
        id: layout
        anchors.fill: parent
        anchors.margins: root._margin
        spacing: root._margin

        // --- header ---
        RowLayout {
            Layout.fillWidth: true
            spacing: root._margin

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 0

                QGCLabel {
                    text: qsTr("MISSION PHASE CONTROL")
                    color: "white"
                    font.bold: true
                    Layout.fillWidth: true
                }
            }
            Rectangle {   // link indicator dot
                width: ScreenTools.defaultFontPixelWidth * 1.1
                height: width
                radius: width / 2
                color: root._linkOk ? "#22C55E"
                                    : root._connFailed ? qgcPal.colorRed
                                                       : qgcPal.colorYellow
            }
            QGCLabel {
                text: root._linkOk ? qsTr("연결됨")
                                   : root._connFailed ? qsTr("연결 실패")
                                                      : qsTr("연결 시도 중…")
                font.pointSize: ScreenTools.smallFontPointSize
                color: root._linkOk ? "#86EFAC"
                                    : root._connFailed ? qgcPal.colorRed
                                                       : qgcPal.colorYellow
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: Qt.rgba(0.22, 0.74, 0.97, 0.28) }

        // --- phase rows (scrollable when they overflow the panel height) ---
        ScrollView {
            id: phaseScroll
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: phaseScroll.availableWidth
                spacing: root._margin

        Repeater {
            model: root._phases

            delegate: Rectangle {
                id: phaseRow
                required property var modelData

                readonly property int  phaseId:   Number(modelData.id)
                readonly property bool done:      root._isDone(phaseId)
                readonly property bool running:   root._isRunning(phaseId)
                readonly property bool clickable: root._clickable(phaseId)

                Layout.fillWidth: true
                Layout.preferredHeight: rowCol.implicitHeight + (root._margin * 1.5)
                radius: 5
                color: running ? Qt.rgba(0.06, 0.18, 0.34, 0.96)
                                : done ? Qt.rgba(0.04, 0.16, 0.12, 0.86)
                                       : root._panel
                border.width: running ? 1 : 0
                border.color: root._accent
                opacity: (!clickable && !running) ? 0.55 : 1.0

                MouseArea {
                    anchors.fill: parent
                    enabled: phaseRow.clickable
                    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                    hoverEnabled: true
                    onClicked: RosBridge.runPhase(phaseRow.phaseId)
                }

                ColumnLayout {
                    id: rowCol
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.verticalCenter: parent.verticalCenter
                    anchors.margins: root._margin
                    spacing: root._margin * 0.4

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: root._margin

                        // phase number / check badge
                        Rectangle {
                            width: ScreenTools.defaultFontPixelHeight * 1.4
                            height: width
                            radius: width / 2
                            color: phaseRow.done ? "#22C55E"
                                                 : phaseRow.running ? root._accentBlue
                                                                    : "#111827"
                            border.width: 1
                            border.color: phaseRow.running ? root._accent : Qt.rgba(0.22, 0.74, 0.97, 0.35)
                            QGCLabel {
                                anchors.centerIn: parent
                                text: phaseRow.done ? "✓" : phaseRow.phaseId.toString()
                                color: (phaseRow.done || phaseRow.running) ? "white" : qgcPal.text
                                font.bold: true
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 0
                            QGCLabel {
                                text: phaseRow.modelData.title === ("Phase " + phaseRow.phaseId)
                                          ? phaseRow.modelData.title
                                          : qsTr("Phase %1 · %2").arg(phaseRow.phaseId).arg(phaseRow.modelData.title)
                                color: "white"
                                font.bold: true
                                Layout.fillWidth: true
                                elide: Text.ElideRight
                            }
                            QGCLabel {
                                // While running, show the live section description
                                // (WP2 이동 중, 고정익 천이 중, …); otherwise the static blurb.
                                text: phaseRow.running && RosBridge.phaseMsg.length > 0
                                          ? RosBridge.phaseMsg : phaseRow.modelData.desc
                                font.pointSize: ScreenTools.smallFontPointSize
                                color: phaseRow.running ? root._accent : root._mutedText
                                Layout.fillWidth: true
                                elide: Text.ElideRight
                            }
                        }

                        // state chip
                        QGCLabel {
                            text: phaseRow.running ? qsTr("진행 중")
                                                   : phaseRow.done && phaseRow.clickable ? qsTr("재실행")
                                                                                       : phaseRow.done ? qsTr("완료")
                                                                                                       : phaseRow.clickable ? qsTr("실행")
                                                                                                                            : qsTr("대기")
                            font.pointSize: ScreenTools.smallFontPointSize
                            font.bold: phaseRow.running
                            color: phaseRow.done ? "#22C55E"
                                                 : phaseRow.running ? root._accent : root._mutedText
                        }
                    }

                    // live progress bar (indeterminate when progress < 0)
                    ProgressBar {
                        Layout.fillWidth: true
                        visible: phaseRow.running
                        indeterminate: phaseRow.running && RosBridge.phaseProgress < 0
                        from: 0; to: 1
                        value: Math.max(0, RosBridge.phaseProgress)
                    }
                }
            }
        }
            }
        }

        // --- status footer ---
        Rectangle { Layout.fillWidth: true; height: 1; color: Qt.rgba(0.22, 0.74, 0.97, 0.28) }

        RowLayout {
            Layout.fillWidth: true
            spacing: root._margin

            QGCLabel {
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                font.pointSize: ScreenTools.smallFontPointSize
                text: {
                    if (root._connFailed)
                        return qsTr("연결 실패 — command/orchestrator.py 가 실행 중인지 확인 후 재시도하세요")
                    if (!root._linkOk)
                        return qsTr("오케스트레이터 연결 시도 중… (최대 60초)")
                    if (root._state === "failed")
                        return qsTr("실패: %1").arg(RosBridge.phaseMsg)
                    if (root._phases.length === 0)
                        return qsTr("MC에서 phase 목록을 기다리는 중…")
                    if (root._busy)
                        return qsTr("Phase %1 진행 중 — %2").arg(root._activePhase).arg(RosBridge.phaseMsg)
                    if (root._phases.length > 0 && root._done.length >= root._phases.length)
                        return qsTr("모든 임무 단계 완료 ✓")
                    return RosBridge.phaseMsg.length > 0 ? RosBridge.phaseMsg : qsTr("대기 중")
                }
                color: (root._connFailed || root._state === "failed") ? qgcPal.colorRed : qgcPal.text
            }

            QGCButton {
                // Take control back from the orchestrator: abort the running phase
                // and hand the vehicle to the GCS (PX4 switches to HOLD / hover).
                text: root._busy ? qsTr("임무 중단 · 제어권 회수") : qsTr("제어권 회수 (HOLD)")
                visible: root._linkOk
                onClicked: RosBridge.abortMission()
            }

            QGCButton {
                text: qsTr("재시도")
                visible: root._connFailed
                onClicked: root._retry()
            }
        }
    }
}
