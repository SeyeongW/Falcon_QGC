pragma ComponentBehavior: Bound

import QtQuick

import QGroundControl.Controls

Item {
    id: root

    readonly property real _centerY: height * 1.10
    readonly property real _radius: height * 0.78
    property real heading: 0
    property bool headingValid: false
    property real selectedHeading: 0
    property bool selectedValid: false

    function headingLabel(value) {
        const normalized = normalizedHeading(value);
        switch (normalized) {
        case 0:
            return "N";
        case 90:
            return "E";
        case 180:
            return "S";
        case 270:
            return "W";
        default:
            const tens = Math.round(normalized / 10);
            return tens < 10 ? "0" + tens : tens.toString();
        }
    }

    function normalizedHeading(value) {
        return ((Math.round(value) % 360) + 360) % 360;
    }

    function shortestDelta(target, current) {
        return ((target - current + 540) % 360) - 180;
    }

    Repeater {
        model: 25

        Item {
            id: headingTick

            readonly property real _delta: _tickHeading - root.heading
            readonly property bool _labelled: _major
            readonly property bool _major: _normalized % 10 === 0
            readonly property int _normalized: root.normalizedHeading(_tickHeading)
            readonly property real _tickHeading: (Math.floor(root.heading / 5) * 5) + ((index - 12) * 5)
            required property int index

            height: root.height * 0.26
            rotation: _delta
            visible: Math.abs(_delta) <= 60
            width: root.width * 0.07
            x: (root.width / 2) + (root._radius * Math.sin(_delta * Math.PI / 180)) - (width / 2)
            y: root._centerY - (root._radius * Math.cos(_delta * Math.PI / 180)) - (height * 0.04)

            Rectangle {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.top
                color: headingTick._major ? "white" : "#86DDF7"
                height: parent.height * (headingTick._major ? 0.32 : 0.18)
                width: Math.max(1, root.width * 0.006)
            }

            QGCLabel {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.top
                anchors.topMargin: parent.height * 0.34
                color: root.normalizedHeading(headingTick._tickHeading) === 0 ? "#FCA5A5" : "white"
                font.bold: true
                font.pixelSize: Math.max(7, root.height * 0.10)
                rotation: -headingTick.rotation
                text: root.headingLabel(headingTick._tickHeading)
                visible: headingTick._labelled
            }
        }
    }

    Rectangle {
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        border.color: "#86DDF7"
        border.width: Math.max(1, root.width * 0.004)
        color: "#102B3E"
        height: parent.height * 0.24
        width: parent.width * 0.18

        QGCLabel {
            anchors.centerIn: parent
            color: root.headingValid ? "white" : "#FCA5A5"
            font.bold: true
            font.pixelSize: Math.max(8, root.height * 0.15)
            text: root.headingValid ? root.normalizedHeading(root.heading).toString().padStart(3, "0") + "°" : qsTr("—")
        }
    }

    QGCLabel {
        readonly property real _delta: root.shortestDelta(root.selectedHeading, root.heading)

        color: "#67E8F9"
        font.bold: true
        font.pixelSize: Math.max(10, root.height * 0.25)
        text: "◇"
        visible: root.selectedValid && root.headingValid && Math.abs(_delta) <= 60
        x: (root.width / 2) + (root._radius * Math.sin(_delta * Math.PI / 180)) - (width / 2)
        y: root._centerY - (root._radius * Math.cos(_delta * Math.PI / 180)) - (height * 0.75)
    }

    QGCLabel {
        anchors.bottom: parent.bottom
        anchors.bottomMargin: parent.height * 0.01
        anchors.horizontalCenter: parent.horizontalCenter
        color: "#86DDF7"
        font.bold: true
        font.pixelSize: Math.max(7, root.height * 0.14)
        text: qsTr("MAG")
    }
}
