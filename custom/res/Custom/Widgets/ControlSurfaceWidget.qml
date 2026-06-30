import QtQuick

/// Top-down VTOL (quadplane) silhouette. Shape adapts to flight mode: in hover
/// the four lift rotors spin; in forward flight the pusher spins and the three
/// control surfaces (ailerons, elevator, rudder) light up. Each surface shows
/// its deflection direction with both color and an arrow:
///   ▲ trailing edge UP   ▼ trailing edge DOWN   ◀ ▶ rudder left/right.
/// Drive *Deflection in [-1,1] (sign = direction) and throttles in [0,1].
/// Bind `fixedWingMode` to vehicle.vtolInFwdFlight. Mocked for now (Phase 5).
Item {
    id: root

    // Flight mode: true = forward (fixed-wing), false = hover (multirotor).
    property bool fixedWingMode: true

    // Control-surface deflections, normalized to [-1, 1].
    // Convention: aileron/elevator +1 = trailing edge DOWN, -1 = UP.
    //             rudder +1 = right, -1 = left.
    property real aileronLeftDeflection:  0
    property real aileronRightDeflection: 0
    property real elevatorDeflection:     0
    property real rudderDeflection:       0

    // Motor throttles, normalized to [0, 1].
    property real liftThrottleFL: 0
    property real liftThrottleFR: 0
    property real liftThrottleRL: 0
    property real liftThrottleRR: 0
    property real pusherThrottle: 0

    // Palette
    property color airframeColor: Qt.rgba(0.20, 0.22, 0.26, 1.0)
    property color airframeEdge:  Qt.rgba(0.45, 0.48, 0.55, 1.0)
    property color neutralColor:  Qt.rgba(0.35, 0.38, 0.44, 1.0)
    property color downColor:     Qt.rgba(0.20, 0.80, 0.45, 1.0)   // trailing edge down (+)
    property color upColor:       Qt.rgba(0.95, 0.40, 0.30, 1.0)   // trailing edge up (-)

    // Max visual hinge tilt (deg) for up/down surfaces, and swing for the rudder.
    property real maxTiltAngle:   48
    property real maxRudderAngle: 26

    /// Blend neutral -> direction color by |deflection|.
    function surfaceColor(v) {
        var m = Math.min(1, Math.abs(v))
        var target = v >= 0 ? root.downColor : root.upColor
        return Qt.rgba(neutralColor.r + (target.r - neutralColor.r) * m,
                       neutralColor.g + (target.g - neutralColor.g) * m,
                       neutralColor.b + (target.b - neutralColor.b) * m,
                       1.0)
    }

    // ---- A control surface that physically tilts (up/down) or swings (rudder) ----
    component Surface: Item {
        id: surf
        property real value: 0          // -1..1  (+ down / - up ; rudder + right / - left)
        property bool lateral: false    // false = aileron/elevator tilt, true = rudder swing

        Rectangle {
            id: bar
            anchors.fill: parent
            radius: 2
            antialiasing: true
            color: root.surfaceColor(surf.value)
            border.color: root.airframeEdge
            border.width: 1

            // Light/shadow sells which way the plate tilts: down -> shaded, up -> lit.
            Rectangle {
                anchors.fill: parent
                radius: 2
                color: surf.value >= 0 ? "#000000" : "#ffffff"
                opacity: surf.lateral ? 0 : Math.min(0.42, Math.abs(surf.value) * 0.5)
            }

            // Hinge at the leading (top) edge. Up/down surfaces rotate about the
            // span axis (x) for a real 3D tilt; the rudder swings about z.
            transform: Rotation {
                id: hinge
                origin.x: bar.width / 2
                origin.y: 0
                axis.x: surf.lateral ? 0 : 1
                axis.y: 0
                axis.z: surf.lateral ? 1 : 0
                angle: surf.value * (surf.lateral ? root.maxRudderAngle : root.maxTiltAngle)
                Behavior on angle { NumberAnimation { duration: 130 } }
            }
        }
    }

    // ---- Reusable rotor (top-down spinning prop) ----
    component Rotor: Item {
        id: rotor
        property real throttle: 0
        property bool active: true

        Rectangle {
            anchors.fill: parent
            radius: width / 2
            color: root.downColor
            opacity: rotor.active ? 0.12 + 0.30 * Math.min(1, rotor.throttle) : 0.05
            Behavior on opacity { NumberAnimation { duration: 200 } }
        }
        Item {
            id: blades
            anchors.fill: parent
            Repeater {
                model: 2
                Rectangle {
                    width:  blades.width * 0.94
                    height: Math.max(2, blades.height * 0.09)
                    radius: height / 2
                    anchors.centerIn: parent
                    rotation: index * 90
                    color: rotor.active ? root.airframeEdge : Qt.rgba(0.4, 0.4, 0.45, 0.6)
                }
            }
            RotationAnimator on rotation {
                from: 0; to: 360
                loops: Animation.Infinite
                running: rotor.active && rotor.throttle > 0.02
                duration: Math.max(90, 700 - 600 * Math.min(1, rotor.throttle))
            }
        }
        Rectangle {
            anchors.centerIn: parent
            width: parent.width * 0.24; height: width; radius: width / 2
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }
    }

    // Square design space, centered, so the plane keeps its proportions.
    Item {
        id: plane
        readonly property real s: Math.min(root.width, root.height)
        width:  s
        height: s
        anchors.centerIn: parent

        // ---- Lift system (booms + 4 rotors): prominent in hover ----
        Item {
            id: liftSystem
            anchors.fill: parent
            opacity: root.fixedWingMode ? 0.30 : 1.0
            Behavior on opacity { NumberAnimation { duration: 250 } }

            Rectangle {
                x: plane.s * 0.265; width: plane.s * 0.03
                y: plane.s * 0.24;  height: plane.s * 0.50
                radius: width * 0.5
                color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
            }
            Rectangle {
                x: plane.s * 0.705; width: plane.s * 0.03
                y: plane.s * 0.24;  height: plane.s * 0.50
                radius: width * 0.5
                color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
            }

            Rotor { width: plane.s * 0.18; height: width
                    x: plane.s * 0.28 - width / 2; y: plane.s * 0.26 - height / 2
                    active: !root.fixedWingMode; throttle: root.liftThrottleFL }
            Rotor { width: plane.s * 0.18; height: width
                    x: plane.s * 0.72 - width / 2; y: plane.s * 0.26 - height / 2
                    active: !root.fixedWingMode; throttle: root.liftThrottleFR }
            Rotor { width: plane.s * 0.18; height: width
                    x: plane.s * 0.28 - width / 2; y: plane.s * 0.72 - height / 2
                    active: !root.fixedWingMode; throttle: root.liftThrottleRL }
            Rotor { width: plane.s * 0.18; height: width
                    x: plane.s * 0.72 - width / 2; y: plane.s * 0.72 - height / 2
                    active: !root.fixedWingMode; throttle: root.liftThrottleRR }
        }

        // ---- Airframe (drawn back-to-front) ----
        Rectangle {   // horizontal stabilizer
            x: plane.s * 0.34; width: plane.s * 0.32
            y: plane.s * 0.78; height: plane.s * 0.045
            radius: height * 0.4
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }
        Rectangle {   // main wing
            x: plane.s * 0.10; width: plane.s * 0.80
            y: plane.s * 0.42; height: plane.s * 0.09
            radius: height * 0.35
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }
        Rectangle {   // fuselage
            x: plane.s * 0.465; width: plane.s * 0.07
            y: plane.s * 0.10;  height: plane.s * 0.80
            radius: width * 0.5
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }
        Rectangle {   // nose
            x: plane.s * 0.475; width: plane.s * 0.05
            y: plane.s * 0.05; height: plane.s * 0.09
            radius: width * 0.5
            color: root.airframeEdge
        }

        // ---- Pusher motor (tail): prominent in forward flight ----
        Rotor {
            width: plane.s * 0.13; height: width
            x: plane.s * 0.50 - width / 2; y: plane.s * 0.90 - height / 2
            active: root.fixedWingMode; throttle: root.pusherThrottle
        }

        // ---- Control surfaces: ailerons, elevator, rudder ----
        Item {
            id: controlSurfaces
            anchors.fill: parent
            opacity: root.fixedWingMode ? 1.0 : 0.30
            Behavior on opacity { NumberAnimation { duration: 250 } }

            Surface {   // left aileron
                x: plane.s * 0.12; width: plane.s * 0.22
                y: plane.s * 0.51; height: plane.s * 0.05
                value: root.aileronLeftDeflection
            }
            Surface {   // right aileron
                x: plane.s * 0.66; width: plane.s * 0.22
                y: plane.s * 0.51; height: plane.s * 0.05
                value: root.aileronRightDeflection
            }
            Surface {   // elevator
                x: plane.s * 0.34; width: plane.s * 0.32
                y: plane.s * 0.825; height: plane.s * 0.045
                value: root.elevatorDeflection
            }
            Surface {   // rudder
                x: plane.s * 0.475; width: plane.s * 0.05
                y: plane.s * 0.83;  height: plane.s * 0.11
                value: root.rudderDeflection
                lateral: true
            }
        }
    }
}
