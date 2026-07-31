import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets

/// Pre-flight / pre-arm status panel (VTOL-GCS).
///
/// Shown in the lower-left pane during mission phase 0, where the 3D mission
/// route has nothing to say yet. Answers the only question that matters before
/// takeoff — "is the airframe safe to arm?" — by combining the arm-readiness
/// verdict, the prearm error text, the GPS/battery/link vitals and the per-sensor
/// health flags from SYS_STATUS.
Rectangle {
    id: root

    property var vehicle: null

    color: FalconTheme.surface1
    radius: FalconTheme.radiusPanel
    border.color: FalconTheme.hairline
    border.width: 0
    clip: true

    readonly property real  _margin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color _accent: FalconTheme.accent
    readonly property color _mutedText: FalconTheme.textMuted
    readonly property color _okColor: FalconTheme.ok
    readonly property color _warnColor: FalconTheme.caution
    readonly property color _failColor: FalconTheme.warn

    // Arm-readiness has three possible sources, in descending authority:
    //
    //  1. HEALTH_AND_ARMING_CHECKS (`canArm`) -- what modern PX4 actually gates
    //     arming on, and what QGC's own arm button uses.
    //  2. The SYS_STATUS MAV_SYS_STATUS_PREARM_CHECK bit (`readyToFly`).
    //  3. The SYS_STATUS sensor-health aggregate, for stacks with neither.
    //
    // Consulting only (2) is what produced a bare "NOT READY" while every sensor
    // row read Normal: the prearm bit is not one of the rows, and on PX4 the real
    // reason lives in the check report below, which was not being shown at all.
    readonly property var  _checkReport: vehicle ? vehicle.healthAndArmingCheckReport : null
    readonly property bool _checksSupported: _checkReport ? _checkReport.supported : false

    readonly property bool _ready: {
        if (!vehicle) {
            return false
        }
        if (_checksSupported) {
            return _checkReport.canArm
        }
        return vehicle.readyToFlyAvailable ? vehicle.readyToFly : vehicle.allSensorsHealthy
    }

    readonly property string _prearmError: vehicle ? vehicle.prearmError : ""
    readonly property var _battery: (vehicle && vehicle.batteries.count > 0) ? vehicle.batteries.get(0) : null

    function _factText(fact, showUnits) {
        if (!fact) {
            return qsTr("–")
        }
        return fact.enumOrValueString + (showUnits && fact.units ? " " + fact.units : "")
    }

    // SysStatusSensorInfo emits "Normal" / "Error" / "Disabled". These are matched
    // as literals rather than through qsTr() because the strings are translated in
    // the SysStatusSensorInfo context, which a qsTr() here would not resolve to.
    function _problemColor(severity) {
        switch (severity) {
        case "error":   return _failColor
        case "warning": return _warnColor
        default:        return _mutedText
        }
    }

    /// Only the sensors that are not reporting Normal. A healthy airframe has a
    /// dozen rows that all say the same thing, which buries the one row that
    /// matters; the panel collapses that case to a single "정상" line and
    /// enumerates sensors only when something is actually wrong.
    readonly property var _sensorFaults: {
        if (!vehicle) {
            return []
        }
        const names = vehicle.sysStatusSensorInfo.sensorNames
        const statuses = vehicle.sysStatusSensorInfo.sensorStatus
        const faults = []
        for (let i = 0; i < names.length; i++) {
            if (statuses[i] !== "Normal") {
                faults.push({ name: names[i], status: statuses[i] })
            }
        }
        return faults
    }

    readonly property int _sensorCount: vehicle ? vehicle.sysStatusSensorInfo.sensorNames.length : 0

    function _sensorStatusColor(status) {
        switch (status) {
        case "Error":    return _failColor
        case "Disabled": return _mutedText
        default:         return _okColor
        }
    }

    /// One "LABEL   value" line, colour-coded by the caller.
    component VitalRow: RowLayout {
        property string label
        property string value
        property color  valueColor: "white"

        Layout.fillWidth: true
        spacing: root._margin

        QGCLabel {
            text: parent.label
            color: root._mutedText
            font.pointSize: ScreenTools.smallFontPointSize
        }

        QGCLabel {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            text: parent.value
            color: parent.valueColor
            font.bold: true
            font.pointSize: ScreenTools.smallFontPointSize
            elide: Text.ElideRight
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: root._margin
        spacing: root._margin

        // --- verdict header ---
        RowLayout {
            Layout.fillWidth: true
            spacing: root._margin

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 0

                QGCLabel {
                    text: qsTr("PRE-FLIGHT CHECK")
                    color: "white"
                    font.bold: true
                }

                QGCLabel {
                    text: qsTr("사전 점검 · 센서 상태")
                    color: root._accent
                    font.pointSize: ScreenTools.smallFontPointSize
                }
            }

            Rectangle {
                Layout.preferredWidth: verdictLabel.implicitWidth + ScreenTools.defaultFontPixelWidth * 2
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.7
                radius: FalconTheme.radiusControl
                color: root._ready ? Qt.rgba(0.13, 0.77, 0.37, 0.18) : Qt.rgba(0.97, 0.44, 0.44, 0.18)
                border.color: root._ready ? root._okColor : root._failColor
                border.width: 1

                QGCLabel {
                    id: verdictLabel
                    anchors.centerIn: parent
                    text: !root.vehicle ? qsTr("NO VEHICLE")
                                        : root._ready ? qsTr("READY") : qsTr("NOT READY")
                    color: root._ready ? "#86EFAC" : "#FCA5A5"
                    font.bold: true
                    font.pointSize: ScreenTools.smallFontPointSize
                }
            }
        }

        Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: FalconTheme.hairline }

        // --- prearm blocker (only when the firmware reports one) ---
        QGCLabel {
            Layout.fillWidth: true
            visible: root._prearmError.length > 0
            text: qsTr("⚠ %1").arg(root._prearmError)
            color: root._failColor
            font.pointSize: ScreenTools.smallFontPointSize
            wrapMode: Text.WordWrap
            maximumLineCount: 3
            elide: Text.ElideRight
        }

        // --- vitals ---
        VitalRow {
            label: qsTr("GPS")
            value: root._factText(root.vehicle ? root.vehicle.gps.lock : null, false)
            valueColor: {
                if (!root.vehicle) {
                    return root._mutedText
                }
                // GPS_FIX_TYPE: 3 == 3D fix, anything below is not flight-worthy.
                return root.vehicle.gps.lock.rawValue >= 3 ? root._okColor : root._failColor
            }
        }

        VitalRow {
            label: qsTr("SATS / HDOP")
            value: root.vehicle
                       ? qsTr("%1  /  %2").arg(root._factText(root.vehicle.gps.count, false))
                                          .arg(root._factText(root.vehicle.gps.hdop, false))
                       : qsTr("–")
            valueColor: {
                if (!root.vehicle) {
                    return root._mutedText
                }
                const sats = root.vehicle.gps.count.rawValue
                const hdop = root.vehicle.gps.hdop.rawValue
                return (sats > 8 && hdop < 1.4) ? root._okColor
                                                : (sats > 5) ? root._warnColor : root._failColor
            }
        }

        VitalRow {
            label: qsTr("BATTERY")
            value: root._battery
                       ? qsTr("%1  /  %2").arg(root._factText(root._battery.percentRemaining, true))
                                          .arg(root._factText(root._battery.voltage, true))
                       : qsTr("–")
            valueColor: {
                if (!root._battery) {
                    return root._mutedText
                }
                const pct = root._battery.percentRemaining.rawValue
                return pct > 50 ? root._okColor : pct > 25 ? root._warnColor : root._failColor
            }
        }

        VitalRow {
            label: qsTr("ARM STATE")
            value: root.vehicle ? (root.vehicle.armed ? qsTr("ARMED") : qsTr("DISARMED")) : qsTr("–")
            valueColor: root.vehicle ? (root.vehicle.armed ? root._warnColor : root._mutedText)
                                     : root._mutedText
        }

        VitalRow {
            label: qsTr("FLIGHT MODE")
            value: root.vehicle ? root.vehicle.flightMode : qsTr("–")
            valueColor: "white"
        }

        Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: FalconTheme.hairline }

        QGCLabel {
            text: root._checksSupported ? qsTr("ARMING CHECKS") : qsTr("SENSOR HEALTH")
            color: root._mutedText
            font.pointSize: ScreenTools.smallFontPointSize
            font.bold: true
        }

        // --- PX4 health & arming checks: the reasons behind the verdict ---
        // Without this a failing check reads as an unexplained "NOT READY",
        // because the blocking condition is not one of the SYS_STATUS rows.
        ColumnLayout {
            Layout.fillWidth: true
            visible: root._checksSupported
            spacing: root._margin * 0.35

            Repeater {
                model: root._checkReport ? root._checkReport.problemsForCurrentMode : null

                delegate: RowLayout {
                    required property var object

                    Layout.fillWidth: true
                    spacing: root._margin

                    Rectangle {
                        Layout.alignment: Qt.AlignTop
                        Layout.topMargin: ScreenTools.defaultFontPixelHeight * 0.35
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 0.9
                        Layout.preferredHeight: Layout.preferredWidth
                        radius: Layout.preferredWidth / 2
                        color: root._problemColor(parent.object.severity)
                    }

                    QGCLabel {
                        Layout.fillWidth: true
                        text: parent.object.message
                        color: root._problemColor(parent.object.severity)
                        font.pointSize: ScreenTools.smallFontPointSize
                        wrapMode: Text.WordWrap
                        maximumLineCount: 2
                        elide: Text.ElideRight
                    }
                }
            }

            QGCLabel {
                Layout.fillWidth: true
                visible: !root._checkReport
                         || root._checkReport.problemsForCurrentMode.count === 0
                text: root._ready ? qsTr("차단 항목 없음 — 시동 가능")
                                  : qsTr("현재 비행모드에 대한 차단 항목이 보고되지 않았습니다")
                color: root._ready ? root._okColor : root._mutedText
                font.pointSize: ScreenTools.smallFontPointSize
                wrapMode: Text.WordWrap
            }
        }

        // --- per-sensor health from SYS_STATUS, faults only ---
        ScrollView {
            id: sensorScroll
            Layout.fillWidth: true
            Layout.fillHeight: true
            visible: !root._checksSupported
            clip: true
            contentWidth: availableWidth

            ColumnLayout {
                width: sensorScroll.availableWidth
                spacing: root._margin * 0.35

                // Everything healthy: one line instead of a dozen identical rows.
                RowLayout {
                    Layout.fillWidth: true
                    visible: root._sensorCount > 0 && root._sensorFaults.length === 0
                    spacing: root._margin

                    Rectangle {
                        Layout.alignment: Qt.AlignVCenter
                        Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 0.9
                        Layout.preferredHeight: Layout.preferredWidth
                        radius: Layout.preferredWidth / 2
                        color: root._okColor
                    }

                    QGCLabel {
                        Layout.fillWidth: true
                        text: qsTr("전 센서 정상 (%1개)").arg(root._sensorCount)
                        color: root._okColor
                        font.bold: true
                        font.pointSize: ScreenTools.smallFontPointSize
                        elide: Text.ElideRight
                    }
                }

                Repeater {
                    model: root._sensorFaults

                    delegate: RowLayout {
                        required property var modelData

                        Layout.fillWidth: true
                        spacing: root._margin

                        Rectangle {
                            Layout.alignment: Qt.AlignVCenter
                            Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 0.9
                            Layout.preferredHeight: Layout.preferredWidth
                            radius: Layout.preferredWidth / 2
                            color: root._sensorStatusColor(parent.modelData.status)
                        }

                        QGCLabel {
                            Layout.fillWidth: true
                            text: parent.modelData.name
                            color: "white"
                            font.pointSize: ScreenTools.smallFontPointSize
                            elide: Text.ElideRight
                        }

                        QGCLabel {
                            text: parent.modelData.status
                            color: root._sensorStatusColor(parent.modelData.status)
                            font.pointSize: ScreenTools.smallFontPointSize
                        }
                    }
                }

                QGCLabel {
                    Layout.fillWidth: true
                    visible: root._sensorCount === 0
                    text: root.vehicle ? qsTr("SYS_STATUS 센서 정보 대기 중…")
                                       : qsTr("기체 연결 대기 중…")
                    color: root._mutedText
                    font.pointSize: ScreenTools.smallFontPointSize
                    wrapMode: Text.WordWrap
                }
            }
        }

        Item { Layout.fillWidth: true; Layout.fillHeight: true; visible: root._checksSupported }
    }
}
