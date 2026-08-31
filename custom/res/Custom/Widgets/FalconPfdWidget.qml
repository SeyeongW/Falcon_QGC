import QtQuick

import QGroundControl.Controls

Rectangle {
    id: root

    readonly property real _airSpeed: factValue(vehicle ? vehicle.airSpeed : null)
    readonly property bool _airSpeedValid: vehicle && isFinite(_airSpeed) && _airSpeed > 0
    readonly property real _altitudeAMSL: factValue(vehicle ? vehicle.altitudeAMSL : null)
    readonly property real _altitudeRelative: factValue(vehicle ? vehicle.altitudeRelative : null)
    readonly property string _altitudeUnits: vehicle && vehicle.altitudeRelative ? vehicle.altitudeRelative.units : ""
    readonly property bool _altitudeValid: vehicle && isFinite(_altitudeRelative)
    readonly property string _amslUnits: vehicle && vehicle.altitudeAMSL ? vehicle.altitudeAMSL.units : ""
    readonly property bool _attitudeValid: vehicle && isFinite(_roll) && isFinite(_pitch)
    readonly property real _heading: rawFactValue(vehicle ? vehicle.heading : null)
    readonly property real _headingToNextWP: rawFactValue(vehicle ? vehicle.headingToNextWP : null)
    readonly property bool _headingValid: vehicle && isFinite(_heading)
    readonly property bool _nextWPValid: vehicle && isFinite(_headingToNextWP)
    readonly property real _pitch: rawFactValue(vehicle ? vehicle.pitch : null)
    readonly property real _roll: rawFactValue(vehicle ? vehicle.roll : null)
    readonly property string _speedUnits: vehicle && vehicle.airSpeed ? vehicle.airSpeed.units : ""
    property var vehicle: null

    function factValue(fact) {
        if (!fact || !isFinite(Number(fact.value))) {
            return NaN;
        }
        return Number(fact.value);
    }

    function rawFactValue(fact) {
        if (!fact || !isFinite(Number(fact.rawValue))) {
            return NaN;
        }
        return Number(fact.rawValue);
    }

    border.color: "#5796B4"
    border.width: Math.max(1, width * 0.004)
    clip: true
    color: "#04101D"
    radius: Math.min(width, height) * 0.035

    PfdValueTape {
        decimalPlaces: 1
        height: root.height * 0.69
        label: qsTr("A/S")
        majorEvery: 5
        pointerOnRight: true
        selectedValid: false
        stepSize: 1
        units: root._speedUnits
        value: root._airSpeedValid ? root._airSpeed : 0
        valueValid: root._airSpeedValid
        width: root.width * 0.19
        x: root.width * 0.018
        y: root.height * 0.055
    }

    PfdAttitudeView {
        dataValid: root._attitudeValid
        height: root.height * 0.69
        pitchAngle: root._attitudeValid ? root._pitch : 0
        rollAngle: root._attitudeValid ? root._roll : 0
        width: root.width * 0.565
        x: root.width * 0.205
        y: root.height * 0.055
    }

    PfdValueTape {
        decimalPlaces: 1
        height: root.height * 0.69
        label: qsTr("ALT REL")
        majorEvery: 2
        pointerOnRight: false
        selectedValid: false
        stepSize: 5
        units: root._altitudeUnits
        value: root._altitudeValid ? root._altitudeRelative : 0
        valueValid: root._altitudeValid
        width: root.width * 0.18
        x: root.width * 0.765
        y: root.height * 0.055
    }

    PfdHeadingArc {
        heading: root._headingValid ? root._heading : 0
        headingValid: root._headingValid
        height: root.height * 0.24
        selectedHeading: root._headingToNextWP
        selectedValid: root._nextWPValid
        width: root.width * 0.58
        x: root.width * 0.20
        y: root.height * 0.75
    }

    QGCLabel {
        anchors.bottom: parent.bottom
        anchors.bottomMargin: root.height * 0.025
        anchors.right: parent.right
        anchors.rightMargin: root.width * 0.025
        color: "#86DDF7"
        font.bold: true
        font.pixelSize: Math.max(8, root.height * 0.032)
        text: isFinite(root._altitudeAMSL) ? qsTr("AMSL %1 %2").arg(root._altitudeAMSL.toFixed(0)).arg(root._amslUnits) : qsTr("AMSL —")
    }
}
