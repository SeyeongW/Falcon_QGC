import QtQuick
import QtQuick.Effects

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlightMap

Item {
    id: root

    property bool showPitch:    true
    property var  vehicle:      null
    property real size
    property bool showHeading:  false

    readonly property real _rollAngle:    vehicle && isFinite(Number(vehicle.roll.rawValue))
                                                  ? Number(vehicle.roll.rawValue)
                                                  : 0
    readonly property real _pitchAngle:   vehicle && isFinite(Number(vehicle.pitch.rawValue))
                                                  ? Number(vehicle.pitch.rawValue)
                                                  : 0
    readonly property real _headingAngle: vehicle && isFinite(Number(vehicle.heading.rawValue))
                                                  ? ((Number(vehicle.heading.rawValue) % 360) + 360) % 360
                                                  : 0
    readonly property string _headingText: {
        if (!vehicle) {
            return "---"
        }

        const heading = Math.round(_headingAngle) % 360
        if (heading < 10) {
            return "00" + heading
        }
        if (heading < 100) {
            return "0" + heading
        }
        return heading.toString()
    }
    readonly property string _bankText: {
        if (!vehicle) {
            return "---"
        }

        const bank = Math.round(_rollAngle)
        if (bank < 0) {
            return Math.abs(bank) + "° L"
        }
        if (bank > 0) {
            return bank + "° R"
        }
        return "0°"
    }

    function headingTapeLabel(heading) {
        const normalizedHeading = ((Math.round(heading) % 360) + 360) % 360
        switch (normalizedHeading) {
        case 0:
            return "N"
        case 90:
            return "E"
        case 180:
            return "S"
        case 270:
            return "W"
        default:
            const headingTens = Math.round(normalizedHeading / 10)
            return headingTens < 10 ? "0" + headingTens : headingTens.toString()
        }
    }

    width:  size
    height: size

    Item {
        id:             instrument
        anchors.fill:   parent
        visible:        false

        //----------------------------------------------------
        //-- Artificial Horizon
        CustomArtificialHorizon {
            rollAngle:          _rollAngle
            pitchAngle:         _pitchAngle
            skyColor1:          "#0a2e50"
            skyColor2:          "#2f85d4"
            groundColor1:       "#897459"
            groundColor2:       "#4b3820"
            anchors.fill:       parent
        }
        //----------------------------------------------------
        //-- Pitch
        QGCPitchIndicator {
            id:                 pitchWidget
            visible:            root.showPitch
            size:               root.size * 0.5
            anchors.verticalCenter: parent.verticalCenter
            pitchAngle:         _pitchAngle
            rollAngle:          _rollAngle
            reticleColor:       "#a3e635"
            color:              Qt.rgba(0,0,0,0)
        }
        //----------------------------------------------------
        //-- Cross Hair
        Image {
            id:                 crossHair
            anchors.centerIn:   parent
            source:             "/custom/img/attitude_crosshair.svg"
            mipmap:             true
            width:              size * 0.75
            sourceSize.width:   width
            fillMode:           Image.PreserveAspectFit
        }
    }

    MultiEffect {
        source:       instrument
        anchors.fill: instrument
        maskEnabled:  true
        maskSource:   mask
    }

    Item {
        id:            mask
        width:         instrument.width
        height:        instrument.height
        layer.enabled: true
        visible:       false
        Rectangle {
            width:  parent.width
            height: parent.height
            radius: width / 2
            color:  "black"
        }
    }

    Rectangle {
        id:             borderRect
        anchors.fill:   parent
        radius:         width / 2
        color:          Qt.rgba(0,0,0,0)
        border.color:   Qt.rgba(0.22, 0.74, 0.97, 0.82)
        border.width:   Math.max(1, root.size * 0.008)
    }

    // Fixed bank-angle scale with a pointer that follows the current roll.
    Item {
        id:             bankScale
        anchors.fill:   parent

        Repeater {
            model: 29

            Item {
                id:                     bankTick
                width:                  root.size * 0.12
                height:                 root.size * 0.10
                x:                      (root.width / 2)
                                        + (root.size * 0.38 * Math.sin(_angle * Math.PI / 180))
                                        - (width / 2)
                y:                      (root.height / 2)
                                        - (root.size * 0.38 * Math.cos(_angle * Math.PI / 180))
                                        - (height / 2)
                rotation:               _angle

                readonly property real _angle:    (modelData * 5) - 70
                readonly property bool _labelled: Math.abs(_angle) % 10 === 0
                readonly property bool _major:    Math.abs(_angle) % 10 === 0

                Rectangle {
                    anchors.top:              parent.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    width:                    Math.max(1, root.size * (bankTick._major ? 0.011 : 0.006))
                    height:                   root.size * (bankTick._major ? 0.060 : 0.032)
                    radius:                   width / 2
                    color:                    "#E2E8F0"
                    opacity:                  bankTick._major ? 0.98 : 0.62
                    antialiasing:             true
                }

                QGCLabel {
                    anchors.top:              parent.top
                    anchors.topMargin:        root.size * 0.060
                    anchors.horizontalCenter: parent.horizontalCenter
                    visible:                  bankTick._labelled
                    text:                     Math.abs(bankTick._angle).toString()
                    color:                    "#CBD5E1"
                    font.pointSize:           Math.max(5, root.size * 0.030)
                    rotation:                 -bankTick.rotation
                }
            }
        }

        Item {
            anchors.fill: parent
            rotation:     Math.max(-70, Math.min(70, root._rollAngle))

            QGCLabel {
                anchors.top:              parent.top
                anchors.topMargin:        root.size * 0.070
                anchors.horizontalCenter: parent.horizontalCenter
                text:                     "▼"
                color:                    "#FCD34D"
                font.bold:                true
                font.pointSize:           Math.max(7, root.size * 0.052)
            }
        }

        Rectangle {
            anchors.top:              parent.top
            anchors.topMargin:       root.size * 0.225
            anchors.horizontalCenter: parent.horizontalCenter
            width:                    bankValueLabel.implicitWidth + (root.size * 0.09)
            height:                   root.size * 0.10
            radius:                   root.size * 0.018
            color:                    Qt.rgba(0.03, 0.08, 0.14, 0.78)

            QGCLabel {
                id:               bankValueLabel
                anchors.centerIn: parent
                text:             root._bankText
                color:            "#FCD34D"
                font.bold:        true
                font.pointSize:   Math.max(7, root.size * 0.038)
            }
        }
    }

    // Horizontal heading tape. Ten-degree ticks flow beneath a fixed center
    // reference as the vehicle heading changes.
    Rectangle {
        id:                       headingTape
        anchors.bottom:           parent.top
        anchors.bottomMargin:     root.size * 0.045
        anchors.horizontalCenter: parent.horizontalCenter
        width:                    root.size * 0.96
        height:                   root.size * 0.18
        radius:                   root.size * 0.025
        color:                    Qt.rgba(0.03, 0.08, 0.14, 0.94)
        border.color:             "#38BDF8"
        border.width:             Math.max(1, root.size * 0.006)
        visible:                  root.showHeading
        clip:                     true

        readonly property real _stepWidth:       width / 14
        readonly property real _baseHeading:     Math.floor(root._headingAngle / 5) * 5
        readonly property real _headingFraction: (root._headingAngle - _baseHeading) / 5

        Repeater {
            model: 21

            Item {
                id:                 tapeTick
                width:              headingTape._stepWidth
                height:             headingTape.height
                x:                  (headingTape.width / 2)
                                    + ((modelData - 10 - headingTape._headingFraction)
                                       * headingTape._stepWidth)
                                    - (width / 2)

                readonly property real _tickHeading: headingTape._baseHeading
                                                       + ((modelData - 10) * 5)
                readonly property bool _labelled:    Math.abs(Math.round(_tickHeading)) % 10 === 0
                readonly property bool _cardinal:    ((Math.round(_tickHeading) % 90) + 90) % 90 === 0

                QGCLabel {
                    anchors.top:              parent.top
                    anchors.topMargin:        root.size * 0.018
                    anchors.horizontalCenter: parent.horizontalCenter
                    visible:                  tapeTick._labelled
                    text:                     root.headingTapeLabel(tapeTick._tickHeading)
                    color:                    tapeTick._cardinal
                                              ? (text === "N" ? "#FCA5A5" : "#F8FAFC")
                                              : "#CBD5E1"
                    font.bold:                tapeTick._cardinal
                    font.pointSize:           Math.max(6, root.size * 0.032)
                }

                Rectangle {
                    anchors.bottom:           parent.bottom
                    anchors.bottomMargin:     root.size * 0.018
                    anchors.horizontalCenter: parent.horizontalCenter
                    width:                    Math.max(1, root.size * 0.006)
                    height:                   root.size * (tapeTick._cardinal
                                                           ? 0.065
                                                           : tapeTick._labelled ? 0.045 : 0.026)
                    color:                    tapeTick._cardinal ? "#F8FAFC" : "#38BDF8"
                    opacity:                  tapeTick._cardinal
                                              ? 1.0
                                              : tapeTick._labelled ? 0.78 : 0.52
                }
            }
        }

        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom:           parent.bottom
            width:                    headingValueLabel.implicitWidth + (root.size * 0.10)
            height:                   root.size * 0.085
            radius:                   root.size * 0.015
            color:                    "#0B1D33"
            border.color:             "#38BDF8"
            border.width:             Math.max(1, root.size * 0.005)

            QGCLabel {
                id:               headingValueLabel
                anchors.centerIn: parent
                text:             root._headingText + "°"
                color:            "white"
                font.bold:        true
                font.pointSize:   Math.max(7, root.size * 0.038)
            }
        }

        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom:           parent.bottom
            anchors.bottomMargin:     root.size * 0.095
            width:                    Math.max(2, root.size * 0.014)
            height:                   root.size * 0.035
            color:                    "#38BDF8"
        }
    }
}
