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
    property real ruddervatorLeftDeflection:  0
    property real ruddervatorRightDeflection: 0

    // Motor throttles, normalized to [0, 1].
    property real liftThrottleFL: 0
    property real liftThrottleFR: 0
    property real liftThrottleRL: 0
    property real liftThrottleRR: 0
    property real pusherThrottle: 0
    property bool actuatorDataValid: false
    property bool vehicleArmed: false

    // Palette
    property color airframeColor: Qt.rgba(0.20, 0.22, 0.26, 1.0)
    property color airframeEdge:  Qt.rgba(0.45, 0.48, 0.55, 1.0)
    property color neutralColor:  Qt.rgba(0.35, 0.38, 0.44, 1.0)
    property color downColor:     Qt.rgba(0.20, 0.80, 0.45, 1.0)   // trailing edge down (+)
    property color upColor:       Qt.rgba(0.95, 0.40, 0.30, 1.0)   // trailing edge up (-)
    property color ruddervatorLeftColor:  "#4D8DFF"
    property color ruddervatorRightColor: "#FF6555"

    // Max visual hinge tilt (deg) for up/down surfaces, and swing for the rudder.
    property real maxTiltAngle:   48
    property real maxRudderAngle: 26
    property real aileronVisualGain: 1.4
    property real aircraftVisualYOffsetRatio: -0.08

    // Hinge positions within the aspect-fitted aircraft image.
    property real leftAileronHingeXRatio:  0.21
    property real rightAileronHingeXRatio: 0.79
    property real aileronHingeYRatio:      0.52

    // Propeller hub centers measured from each PNG's non-transparent bounding box.
    property real liftPropFLHubXRatio: 0.3450
    property real liftPropFLHubYRatio: 0.3255
    property real liftPropFRHubXRatio: 0.6550
    property real liftPropFRHubYRatio: 0.3255
    property real liftPropRLHubXRatio: 0.3455
    property real liftPropRLHubYRatio: 0.5615
    property real liftPropRRHubXRatio: 0.6550
    property real liftPropRRHubYRatio: 0.5605
    property real pusherPropHubXRatio: 0.5000
    property real pusherPropHubYRatio: 0.7195

    property real liftPropFLAngle: 0
    property real liftPropFRAngle: 0
    property real liftPropRLAngle: 0
    property real liftPropRRAngle: 0
    property real pusherPropAngle: 0

    property real propellerRunThreshold: 0.05
    property real maxPropellerDegreesPerSecond: 1080

    readonly property bool anyMotorRunning: motorMagnitude(liftThrottleFL) > propellerRunThreshold
                                            || motorMagnitude(liftThrottleFR) > propellerRunThreshold
                                            || motorMagnitude(liftThrottleRL) > propellerRunThreshold
                                            || motorMagnitude(liftThrottleRR) > propellerRunThreshold
                                            || motorMagnitude(pusherThrottle) > propellerRunThreshold

    // Temporary visualization directions. Confirm the aircraft's actual CW/CCW
    // motor layout before replacing these values.
    property int liftPropFLDirection: 1
    property int liftPropFRDirection: -1
    property int liftPropRLDirection: -1
    property int liftPropRRDirection: 1
    property int pusherPropDirection: 1

    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value))
    }

    function motorMagnitude(value) {
        return Math.max(0, Math.min(1, value))
    }

    function nextPropellerAngle(angle, direction, throttle, elapsedMilliseconds) {
        var magnitude = root.motorMagnitude(throttle)
        if (magnitude <= root.propellerRunThreshold) {
            return angle
        }

        var updatedAngle = (angle
                            + direction * magnitude * root.maxPropellerDegreesPerSecond * elapsedMilliseconds / 1000.0) % 360
        return updatedAngle < 0 ? updatedAngle + 360 : updatedAngle
    }

    /// Blend neutral -> direction color by |deflection|.
    function surfaceColor(v) {
        var m = Math.min(1, Math.abs(v))
        var target = v >= 0 ? root.downColor : root.upColor
        return Qt.rgba(neutralColor.r + (target.r - neutralColor.r) * m,
                       neutralColor.g + (target.g - neutralColor.g) * m,
                       neutralColor.b + (target.b - neutralColor.b) * m,
                       1.0)
    }

    // Visualizes PX4 motor output commands, not measured ESC RPM feedback.
    Timer {
        id: propellerTimer
        interval: 16
        repeat: true
        running: root.actuatorDataValid && root.vehicleArmed && root.anyMotorRunning

        onTriggered: {
            root.liftPropFLAngle = root.nextPropellerAngle(root.liftPropFLAngle, root.liftPropFLDirection,
                                                           root.liftThrottleFL, propellerTimer.interval)
            root.liftPropFRAngle = root.nextPropellerAngle(root.liftPropFRAngle, root.liftPropFRDirection,
                                                           root.liftThrottleFR, propellerTimer.interval)
            root.liftPropRLAngle = root.nextPropellerAngle(root.liftPropRLAngle, root.liftPropRLDirection,
                                                           root.liftThrottleRL, propellerTimer.interval)
            root.liftPropRRAngle = root.nextPropellerAngle(root.liftPropRRAngle, root.liftPropRRDirection,
                                                           root.liftThrottleRR, propellerTimer.interval)
            root.pusherPropAngle = root.nextPropellerAngle(root.pusherPropAngle, root.pusherPropDirection,
                                                           root.pusherThrottle, propellerTimer.interval)
        }
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
        anchors.verticalCenterOffset: root.height * root.aircraftVisualYOffsetRatio

        // ---- Lift system (booms + 4 rotors): prominent in hover ----
        Item {
            id: liftSystem
            visible: false
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
        Image {
            anchors.fill: parent
            source: "/custom/img/aircraft_body.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
        }

        Image {
            id: leftAileronImage
            anchors.fill: parent
            source: "/custom/img/aileron_left.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true

            transform: Rotation {
                origin.x: (leftAileronImage.width - leftAileronImage.paintedWidth) / 2
                          + leftAileronImage.paintedWidth * root.leftAileronHingeXRatio
                origin.y: (leftAileronImage.height - leftAileronImage.paintedHeight) / 2
                          + leftAileronImage.paintedHeight * root.aileronHingeYRatio
                axis.x: 1
                axis.y: 0
                axis.z: 0
                angle: Math.max(-70, Math.min(70, root.aileronLeftDeflection * root.maxTiltAngle * root.aileronVisualGain))
                Behavior on angle { NumberAnimation { duration: 130 } }
            }
        }

        Image {
            id: rightAileronImage
            anchors.fill: parent
            source: "/custom/img/aileron_right.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true

            transform: Rotation {
                origin.x: (rightAileronImage.width - rightAileronImage.paintedWidth) / 2
                          + rightAileronImage.paintedWidth * root.rightAileronHingeXRatio
                origin.y: (rightAileronImage.height - rightAileronImage.paintedHeight) / 2
                          + rightAileronImage.paintedHeight * root.aileronHingeYRatio
                axis.x: 1
                axis.y: 0
                axis.z: 0
                angle: Math.max(-70, Math.min(70, root.aileronRightDeflection * root.maxTiltAngle * root.aileronVisualGain))
                Behavior on angle { NumberAnimation { duration: 130 } }
            }
        }

        // Ruddervator visualization is temporarily fixed at neutral until the
        // final hinge-axis representation and actual aircraft servo mapping are implemented.
        Image {
            id: leftRuddervatorImage
            anchors.fill: parent
            source: "/custom/img/ruddervator_left.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0
        }

        Image {
            id: rightRuddervatorImage
            anchors.fill: parent
            source: "/custom/img/ruddervator_right.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0
        }

        Image {
            id: liftPropFLImage
            anchors.fill: parent
            source: "/custom/img/lift_prop_fl.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0

            transform: Rotation {
                origin.x: (liftPropFLImage.width - liftPropFLImage.paintedWidth) / 2
                          + liftPropFLImage.paintedWidth * root.liftPropFLHubXRatio
                origin.y: (liftPropFLImage.height - liftPropFLImage.paintedHeight) / 2
                          + liftPropFLImage.paintedHeight * root.liftPropFLHubYRatio
                angle: root.liftPropFLAngle
            }
        }

        Image {
            id: liftPropFRImage
            anchors.fill: parent
            source: "/custom/img/lift_prop_fr.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0

            transform: Rotation {
                origin.x: (liftPropFRImage.width - liftPropFRImage.paintedWidth) / 2
                          + liftPropFRImage.paintedWidth * root.liftPropFRHubXRatio
                origin.y: (liftPropFRImage.height - liftPropFRImage.paintedHeight) / 2
                          + liftPropFRImage.paintedHeight * root.liftPropFRHubYRatio
                angle: root.liftPropFRAngle
            }
        }

        Image {
            id: liftPropRLImage
            anchors.fill: parent
            source: "/custom/img/lift_prop_rl.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0

            transform: Rotation {
                origin.x: (liftPropRLImage.width - liftPropRLImage.paintedWidth) / 2
                          + liftPropRLImage.paintedWidth * root.liftPropRLHubXRatio
                origin.y: (liftPropRLImage.height - liftPropRLImage.paintedHeight) / 2
                          + liftPropRLImage.paintedHeight * root.liftPropRLHubYRatio
                angle: root.liftPropRLAngle
            }
        }

        Image {
            id: liftPropRRImage
            anchors.fill: parent
            source: "/custom/img/lift_prop_rr.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0

            transform: Rotation {
                origin.x: (liftPropRRImage.width - liftPropRRImage.paintedWidth) / 2
                          + liftPropRRImage.paintedWidth * root.liftPropRRHubXRatio
                origin.y: (liftPropRRImage.height - liftPropRRImage.paintedHeight) / 2
                          + liftPropRRImage.paintedHeight * root.liftPropRRHubYRatio
                angle: root.liftPropRRAngle
            }
        }

        // The pusher's screen-plane rotation is a simplified indication that the
        // cruise motor is active; its real top-view rotation axis is different.
        Image {
            id: pusherPropImage
            anchors.fill: parent
            source: "/custom/img/pusher_prop.png"
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            antialiasing: true
            opacity: 1.0

            transform: Rotation {
                origin.x: (pusherPropImage.width - pusherPropImage.paintedWidth) / 2
                          + pusherPropImage.paintedWidth * root.pusherPropHubXRatio
                origin.y: (pusherPropImage.height - pusherPropImage.paintedHeight) / 2
                          + pusherPropImage.paintedHeight * root.pusherPropHubYRatio
                angle: root.pusherPropAngle
            }
        }

        // ---- Pusher motor (tail): prominent in forward flight ----
        Rotor {
            visible: false
            width: plane.s * 0.13; height: width
            x: plane.s * 0.50 - width / 2; y: plane.s * 0.90 - height / 2
            active: root.fixedWingMode; throttle: root.pusherThrottle
        }

        // ---- Control surfaces: ailerons, elevator, rudder ----
        Item {
            id: controlSurfaces
            visible: false
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
