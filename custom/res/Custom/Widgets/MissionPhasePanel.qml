import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

import Custom.Ros

/// Mission phase orchestrator panel (VTOL-GCS).
///
/// Shows the four mission phases as a sequential checklist. Clicking an enabled
/// phase publishes `command/run_phase` via `RosBridge`; the orchestrator runs the
/// matching `phaseN.py` and streams `command/status` back, which drives the live
/// progress bar, the current-section text (e.g. "WP2 이동 중", "고정익 천이 중"),
/// and the completed/greyed-out state. Phase N stays locked until N-1 completes.
Rectangle {
    id: root

    color: qgcPal.window
    radius: 4
    border.color: qgcPal.groupBorder
    border.width: 1
    opacity: 0.95

    readonly property real   _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color  _accent: "#1D4ED8"   // FALCON signature deep blue

    // Static phase metadata (title + one-line description). Index == phase id.
    readonly property var _phases: [
        { title: qsTr("사전 점검"),        desc: qsTr("센서 · GPS · 배터리 확인") },
        { title: qsTr("이륙 · 정찰"),      desc: qsTr("VTOL 이륙 후 고정익 천이 · 정찰") },
        { title: qsTr("대상 탐지 · 접근"), desc: qsTr("짐벌 정렬 · 정밀 착륙 접근") },
        { title: qsTr("복귀 · 착륙"),      desc: qsTr("고정익 복귀 후 VTOL 착륙") }
    ]

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

    function _isDone(i)     { return _done.indexOf(i) >= 0 }
    function _isRunning(i)  { return _busy && _activePhase === i }
    function _prevDone(i)   { return i === 0 || _done.indexOf(i - 1) >= 0 }
    // Clickable only when the orchestrator is up, nothing is running, this phase
    // is not already done, and the previous phase has completed (sequential gate).
    function _clickable(i)  { return _linkOk && !_busy && !_isDone(i) && _prevDone(i) }

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

            QGCLabel {
                text: qsTr("임무 시퀀스")
                font.bold: true
                Layout.fillWidth: true
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
                color: root._linkOk ? qgcPal.text
                                    : root._connFailed ? qgcPal.colorRed
                                                       : qgcPal.colorYellow
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: qgcPal.groupBorder }

        // --- phase rows ---
        Repeater {
            model: root._phases

            delegate: Rectangle {
                id: phaseRow
                required property int index
                required property var modelData

                readonly property bool done:      root._isDone(index)
                readonly property bool running:   root._isRunning(index)
                readonly property bool clickable: root._clickable(index)

                Layout.fillWidth: true
                Layout.preferredHeight: rowCol.implicitHeight + (root._margin * 1.5)
                radius: 4
                color: running ? Qt.rgba(0.11, 0.30, 0.85, 0.18)
                                : done ? Qt.rgba(qgcPal.text.r, qgcPal.text.g, qgcPal.text.b, 0.04)
                                       : qgcPal.windowShade
                border.width: running ? 1 : 0
                border.color: root._accent
                opacity: (done || (!clickable && !running)) ? 0.55 : 1.0

                MouseArea {
                    anchors.fill: parent
                    enabled: phaseRow.clickable
                    cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
                    hoverEnabled: true
                    onClicked: RosBridge.runPhase(phaseRow.index)
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
                                                 : phaseRow.running ? root._accent
                                                                    : qgcPal.window
                            border.width: 1
                            border.color: phaseRow.running ? root._accent : qgcPal.groupBorder
                            QGCLabel {
                                anchors.centerIn: parent
                                text: phaseRow.done ? "✓" : phaseRow.index.toString()
                                color: (phaseRow.done || phaseRow.running) ? "white" : qgcPal.text
                                font.bold: true
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 0
                            QGCLabel {
                                text: qsTr("Phase %1 · %2").arg(phaseRow.index).arg(phaseRow.modelData.title)
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
                                color: phaseRow.running ? root._accent : qgcPal.colorGrey
                                Layout.fillWidth: true
                                elide: Text.ElideRight
                            }
                        }

                        // state chip
                        QGCLabel {
                            text: phaseRow.done ? qsTr("완료")
                                                : phaseRow.running ? qsTr("진행 중")
                                                                   : phaseRow.clickable ? qsTr("실행")
                                                                                        : qsTr("대기")
                            font.pointSize: ScreenTools.smallFontPointSize
                            font.bold: phaseRow.running
                            color: phaseRow.done ? "#22C55E"
                                                 : phaseRow.running ? root._accent : qgcPal.colorGrey
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

        // --- status footer ---
        Rectangle { Layout.fillWidth: true; height: 1; color: qgcPal.groupBorder }

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
                    if (root._busy)
                        return qsTr("Phase %1 진행 중 — %2").arg(root._activePhase).arg(RosBridge.phaseMsg)
                    if (root._done.length >= root._phases.length)
                        return qsTr("모든 임무 단계 완료 ✓")
                    return RosBridge.phaseMsg.length > 0 ? RosBridge.phaseMsg : qsTr("대기 중")
                }
                color: (root._connFailed || root._state === "failed") ? qgcPal.colorRed : qgcPal.text
            }

            QGCButton {
                text: qsTr("재시도")
                visible: root._connFailed
                onClicked: root._retry()
            }
        }
    }
}
