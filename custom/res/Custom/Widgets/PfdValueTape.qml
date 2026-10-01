pragma ComponentBehavior: Bound

import QtQuick

import QGroundControl.Controls

Rectangle {
    id: root

    readonly property real _baseValue: Math.floor(value / stepSize) * stepSize
    readonly property real _centerY: height * 0.52
    readonly property real _fraction: valueValid ? (value - _baseValue) / stepSize : 0
    readonly property real _tickSpacing: height * 0.055
    property int decimalPlaces: 0
    property string label
    property int majorEvery: 5
    property bool pointerOnRight: true
    property bool selectedValid: false
    property real selectedValue: 0
    property real stepSize: 1
    property string units
    property real value: 0
    property bool valueValid: false

    border.color: Qt.rgba(0.38, 0.85, 0.96, 0.62)
    border.width: Math.max(1, width * 0.012)
    clip: true
    color: Qt.rgba(0.02, 0.07, 0.12, 0.92)

    QGCLabel {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: parent.height * 0.018
        color: "#86DDF7"
        font.bold: true
        font.pixelSize: Math.max(8, Math.min(root.height * 0.060, root.width * 0.15))
        text: root.label + (root.units.length > 0 ? "  " + root.units : "")
    }

    Repeater {
        model: 31

        Item {
            id: tapeTick

            readonly property bool _major: Math.abs(Math.round(_tickValue / root.stepSize)) % root.majorEvery === 0
            readonly property int _offset: index - 15
            readonly property real _tickValue: root._baseValue - (_offset * root.stepSize)
            required property int index

            height: 1
            visible: y > root.height * 0.10 && y < root.height * 0.96
            width: root.width
            x: 0
            y: root._centerY + ((_offset + root._fraction) * root._tickSpacing)

            Rectangle {
                color: tapeTick._major ? "white" : "#86DDF7"
                height: Math.max(1, root.height * 0.006)
                width: parent.width * (tapeTick._major ? 0.24 : 0.14)
                x: root.pointerOnRight ? parent.width - width - (parent.width * 0.04) : parent.width * 0.04
            }

            QGCLabel {
                anchors.verticalCenter: parent.verticalCenter
                color: "white"
                font.bold: true
                font.pixelSize: Math.max(7, Math.min(root.height * 0.050, root.width * 0.13))
                horizontalAlignment: root.pointerOnRight ? Text.AlignLeft : Text.AlignRight
                text: Math.round(tapeTick._tickValue).toString()
                visible: tapeTick._major
                width: root.width * 0.54
                x: root.pointerOnRight ? root.width * 0.08 : root.width * 0.38
            }
        }
    }

    Rectangle {
        border.color: "#86DDF7"
        border.width: Math.max(1, root.width * 0.015)
        color: "#102B3E"
        height: root.height * 0.115
        width: root.width * 0.70
        x: root.pointerOnRight ? root.width * 0.30 : 0
        y: root._centerY - (height / 2)

        QGCLabel {
            anchors.centerIn: parent
            color: root.valueValid ? "white" : "#FCA5A5"
            font.bold: true
            font.pixelSize: Math.max(9, Math.min(root.height * 0.072, root.width * 0.16))
            text: root.valueValid ? root.value.toFixed(root.decimalPlaces) : qsTr("—")
        }
    }

    QGCLabel {
        readonly property real _selectedY: root._centerY - ((root.selectedValue - root.value) / root.stepSize * root._tickSpacing)

        color: "#F472B6"
        font.bold: true
        font.pixelSize: Math.max(9, root.height * 0.060)
        text: root.pointerOnRight ? "▶" : "◀"
        visible: root.selectedValid && root.valueValid && _selectedY > root.height * 0.12 && _selectedY < root.height * 0.94
        x: root.pointerOnRight ? root.width * 0.02 : root.width * 0.78
        y: _selectedY - (height / 2)
    }

    Rectangle {
        anchors.fill: parent
        color: Qt.rgba(0.02, 0.05, 0.09, 0.40)
        visible: !root.valueValid
    }
}
