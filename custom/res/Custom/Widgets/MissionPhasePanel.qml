import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets

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

    color: FalconTheme.surface1
    radius: FalconTheme.radiusPanel
    border.color: FalconTheme.hairline
    border.width: 0
    opacity: 0.98

    // --- fit-to-box sizing ---------------------------------------------------
    // The console pane shrinks with the mission phase, and the phase list must
    // never be clipped or run under the neighbouring pane. The content is laid
    // out at its natural size and then scaled down by a render transform to fit
    // the box exactly. A transform does not feed back into the layout, so
    // `layout.implicitHeight` stays stable and there is no binding loop --
    // which a font-size-driven scale could not guarantee.
    readonly property real _scale: fitBox.fitScale

    readonly property real   _fontPixelSize:      ScreenTools.defaultFontPixelHeight
    readonly property real   _smallFontPixelSize: ScreenTools.defaultFontPixelHeight * 0.82

    readonly property real   _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color  _accent: FalconTheme.accent
    readonly property color  _accentBlue: "#1D4ED8"
    readonly property color  _panel: FalconTheme.surface2
    readonly property color  _mutedText: FalconTheme.textMuted

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

    // Public phase readout for consumers outside this panel (e.g. the adaptive
    // fly-view layout). -1 while the orchestrator link is down so callers can
    // tell "unknown" from phase 0.
    readonly property int    activePhase:  _linkOk ? _activePhase : -1
    readonly property int    phaseCount:   _phases.length
    readonly property string currentPhaseText: {
        if (!_linkOk || _activePhase < 0 || _activePhase >= _phases.length) {
            return "--"
        }
        return qsTr("Phase %1 · %2").arg(_activePhase).arg(_phases[_activePhase].title)
    }

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

    Item {
        id: fitBox

        anchors.fill: parent
        anchors.margins: root._margin

        // 1.0 while the content fits; below that, exactly the factor needed to
        // bring the natural height inside the available height.
        readonly property real fitScale:
            Math.min(1, height / Math.max(1, layout.implicitHeight))

        ColumnLayout {
            id: layout
            width: fitBox.width
            spacing: root._margin

            // Scaled about the top centre so the panel stays anchored under its
            // header as it shrinks.
            transform: Scale {
                origin.x: fitBox.width / 2
                origin.y: 0
                xScale: fitBox.fitScale
                yScale: fitBox.fitScale
            }

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
                    font.pixelSize: root._fontPixelSize
                    Layout.fillWidth: true
                    elide: Text.ElideRight
                }
            }
            Rectangle {   // link indicator dot
                // Layout.* rather than width/height: a RowLayout drives its
                // children from implicitWidth, so a plain width binding is
                // discarded and the dot collapses.
                Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 1.1
                Layout.preferredHeight: Layout.preferredWidth
                radius: Layout.preferredWidth / 2
                color: root._linkOk ? FalconTheme.ok
                                    : root._connFailed ? qgcPal.colorRed
                                                       : qgcPal.colorYellow
            }
            QGCLabel {
                text: root._linkOk ? qsTr("연결됨")
                                   : root._connFailed ? qsTr("연결 실패")
                                                      : qsTr("연결 시도 중…")
                font.pixelSize: root._smallFontPixelSize
                color: root._linkOk ? "#86EFAC"
                                    : root._connFailed ? qgcPal.colorRed
                                                       : qgcPal.colorYellow
            }
        }

        Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: FalconTheme.hairline }

        // --- phase rows ---
        // No ScrollView: the fit-to-box scale guarantees the list fits, and a
        // fillHeight ScrollView would hide the natural height that scale needs.
        ColumnLayout {
            Layout.fillWidth: true
            spacing: root._margin

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
                radius: FalconTheme.radiusPanel
                color: running ? Qt.rgba(0.06, 0.18, 0.34, 0.96)
                                : done ? Qt.rgba(0.04, 0.16, 0.12, 0.86)
                                       : root._panel
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
                            Layout.preferredWidth: ScreenTools.defaultFontPixelHeight * 1.4
                            Layout.preferredHeight: Layout.preferredWidth
                            radius: Layout.preferredWidth / 2
                            color: phaseRow.done ? FalconTheme.ok
                                                 : phaseRow.running ? root._accentBlue
                                                                    : "#111827"
                            border.width: 1
                            border.color: phaseRow.running ? root._accent : Qt.rgba(0.22, 0.74, 0.97, 0.35)
                            QGCLabel {
                                anchors.centerIn: parent
                                text: phaseRow.done ? "✓" : phaseRow.index.toString()
                                color: (phaseRow.done || phaseRow.running) ? "white" : qgcPal.text
                                font.bold: true
                                font.pixelSize: root._smallFontPixelSize
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 0
                            QGCLabel {
                                text: qsTr("Phase %1 · %2").arg(phaseRow.index).arg(phaseRow.modelData.title)
                                color: "white"
                                font.bold: true
                                font.pixelSize: root._fontPixelSize
                                Layout.fillWidth: true
                                elide: Text.ElideRight
                            }
                            QGCLabel {
                                // While running, show the live section description
                                // (WP2 이동 중, 고정익 천이 중, …); otherwise the static blurb.
                                text: phaseRow.running && RosBridge.phaseMsg.length > 0
                                          ? RosBridge.phaseMsg : phaseRow.modelData.desc
                                font.pixelSize: root._smallFontPixelSize
                                color: phaseRow.running ? root._accent : root._mutedText
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
                            font.pixelSize: root._smallFontPixelSize
                            font.bold: phaseRow.running
                            color: phaseRow.done ? FalconTheme.ok
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

        // --- status footer ---
        Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: FalconTheme.hairline }

        // Status line and buttons are stacked, never side by side. In a Row the
        // wrapping status label (fillWidth) is the only shrinkable item, so once
        // the buttons' implicit width exceeds the console pane the label
        // collapses to a few pixels and renders one glyph per line. That is what
        // broke the layout when a phase started running: the abort button grows
        // ("임무 중단 · 제어권 회수") and the status text grows at the same moment.
        ColumnLayout {
            Layout.fillWidth: true
            spacing: root._margin * 0.6

            QGCLabel {
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                maximumLineCount: 2
                elide: Text.ElideRight
                font.pixelSize: root._smallFontPixelSize
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

            RowLayout {
                Layout.fillWidth: true
                spacing: root._margin

                QGCButton {
                    // Take control back from the orchestrator: abort the running phase
                    // and hand the vehicle to the GCS (PX4 switches to HOLD / hover).
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    text: root._busy ? qsTr("임무 중단") : qsTr("제어권 회수 (HOLD)")
                    font.pixelSize: root._smallFontPixelSize
                    visible: root._linkOk
                    onClicked: RosBridge.abortMission()
                }

                QGCButton {
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    text: qsTr("재시도")
                    font.pixelSize: root._smallFontPixelSize
                    visible: root._connFailed
                    onClicked: root._retry()
                }
            }
        }
    }
    }
}
