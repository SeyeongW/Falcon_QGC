pragma ComponentBehavior: Bound

import QtQuick

import QGroundControl.Controls

Item {
    id: root

    readonly property real _clampedRoll: Math.max(-60, Math.min(60, rollAngle))
    readonly property real _pitchPixelsPerDegree: height / 45
    property color cornerMaskColor: "#04101D"
    property bool dataValid: false
    property real pitchAngle: 0
    property real rollAngle: 0

    clip: true

    Item {
        id: instrument

        anchors.fill: parent
        visible: true

        Rectangle {
            anchors.fill: parent
            color: "#071526"
        }

        Item {
            id: movingScene

            height: root.height * 6
            rotation: -root.rollAngle
            width: root.width * 4
            x: (root.width - width) / 2
            y: (root.height - height) / 2 + (root.pitchAngle * root._pitchPixelsPerDegree)

            Rectangle {
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.top: parent.top
                color: "#1769AA"
                height: parent.height / 2
            }

            Rectangle {
                anchors.bottom: parent.bottom
                anchors.left: parent.left
                anchors.right: parent.right
                color: "#87502E"
                height: parent.height / 2
            }

            Rectangle {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.verticalCenter: parent.verticalCenter
                color: "white"
                height: Math.max(2, root.height * 0.012)
                width: parent.width
            }

            Repeater {
                model: 17

                Item {
                    id: pitchMarker

                    readonly property int _degrees: (index - 8) * 5
                    readonly property bool _major: Math.abs(_degrees) % 10 === 0
                    required property int index

                    height: root.height * 0.10
                    visible: _degrees !== 0
                    width: root.width * 0.72
                    x: (movingScene.width - width) / 2
                    y: (movingScene.height / 2) - (_degrees * root._pitchPixelsPerDegree) - (height / 2)

                    Rectangle {
                        id: pitchLine

                        anchors.centerIn: parent
                        color: "white"
                        height: Math.max(1, root.height * 0.009)
                        width: parent.width * (pitchMarker._major ? 0.48 : 0.28)
                    }

                    QGCLabel {
                        anchors.right: pitchLine.left
                        anchors.rightMargin: parent.width * 0.045
                        anchors.verticalCenter: parent.verticalCenter
                        color: "white"
                        font.bold: true
                        font.pixelSize: Math.max(8, root.height * 0.065)
                        text: Math.abs(pitchMarker._degrees)
                        visible: pitchMarker._major
                    }

                    QGCLabel {
                        anchors.left: pitchLine.right
                        anchors.leftMargin: parent.width * 0.045
                        anchors.verticalCenter: parent.verticalCenter
                        color: "white"
                        font.bold: true
                        font.pixelSize: Math.max(8, root.height * 0.065)
                        text: Math.abs(pitchMarker._degrees)
                        visible: pitchMarker._major
                    }
                }
            }
        }

        // Fixed bank scale; only the yellow pointer follows vehicle roll.
        Repeater {
            model: [-60, -30, -20, -10, 0, 10, 20, 30, 60]

            Item {
                id: bankTick

                readonly property real _radius: root.height * 0.43
                required property real modelData

                height: root.height * 0.09
                rotation: modelData
                width: root.width * 0.06
                x: (root.width / 2) + (_radius * Math.sin(modelData * Math.PI / 180)) - (width / 2)
                y: (root.height / 2) - (_radius * Math.cos(modelData * Math.PI / 180)) - (height / 2)

                Rectangle {
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.top: parent.top
                    color: "white"
                    height: root.height * (bankTick.modelData === 0 ? 0.070 : 0.045)
                    width: Math.max(1, root.width * 0.009)
                }
            }
        }

        QGCLabel {
            readonly property real _radius: root.height * 0.37

            color: "#FCD34D"
            font.bold: true
            font.pixelSize: Math.max(10, root.height * 0.075)
            text: "▼"
            x: (root.width / 2) + (_radius * Math.sin(root._clampedRoll * Math.PI / 180)) - (width / 2)
            y: (root.height / 2) - (_radius * Math.cos(root._clampedRoll * Math.PI / 180)) - (height / 2)
        }

        Item {
            anchors.centerIn: parent
            height: parent.height * 0.12
            width: parent.width * 0.58

            Rectangle {
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter
                color: "white"
                height: Math.max(2, root.height * 0.015)
                width: parent.width * 0.38
            }

            Rectangle {
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                color: "white"
                height: Math.max(2, root.height * 0.015)
                width: parent.width * 0.38
            }

            Rectangle {
                anchors.bottom: parent.verticalCenter
                anchors.horizontalCenter: parent.horizontalCenter
                color: "white"
                height: parent.height * 0.42
                width: Math.max(2, root.width * 0.012)
            }

            Rectangle {
                anchors.centerIn: parent
                border.color: "white"
                border.width: Math.max(1, root.width * 0.007)
                color: "#071526"
                height: width
                radius: width / 2
                width: Math.max(5, root.width * 0.035)
            }
        }
    }

    Canvas {
        id: cornerMask

        anchors.fill: parent

        onHeightChanged: requestPaint()
        onPaint: {
            const context = getContext("2d");
            const radius = Math.min(width, height) * 0.065;

            context.clearRect(0, 0, width, height);
            context.fillStyle = root.cornerMaskColor;

            context.beginPath();
            context.moveTo(0, radius);
            context.lineTo(0, 0);
            context.lineTo(radius, 0);
            context.arc(radius, radius, radius, -Math.PI / 2, -Math.PI, true);
            context.closePath();
            context.fill();

            context.beginPath();
            context.moveTo(width - radius, 0);
            context.lineTo(width, 0);
            context.lineTo(width, radius);
            context.arc(width - radius, radius, radius, 0, -Math.PI / 2, true);
            context.closePath();
            context.fill();

            context.beginPath();
            context.moveTo(width, height - radius);
            context.lineTo(width, height);
            context.lineTo(width - radius, height);
            context.arc(width - radius, height - radius, radius, Math.PI / 2, 0, true);
            context.closePath();
            context.fill();

            context.beginPath();
            context.moveTo(radius, height);
            context.lineTo(0, height);
            context.lineTo(0, height - radius);
            context.arc(radius, height - radius, radius, Math.PI, Math.PI / 2, true);
            context.closePath();
            context.fill();
        }
        onWidthChanged: requestPaint()
    }

    Rectangle {
        anchors.fill: parent
        border.color: "#86DDF7"
        border.width: Math.max(1, width * 0.006)
        color: "transparent"
        radius: Math.min(width, height) * 0.065
    }

    Rectangle {
        anchors.fill: parent
        color: Qt.rgba(0.02, 0.05, 0.09, 0.72)
        radius: Math.min(width, height) * 0.065
        visible: !root.dataValid

        QGCLabel {
            anchors.centerIn: parent
            color: "#FCA5A5"
            font.bold: true
            font.pixelSize: Math.max(9, root.height * 0.075)
            text: qsTr("NO ATT DATA")
        }
    }
}
