import QtQuick

/// Top-down VTOL plane silhouette whose control surfaces change color (and tilt
/// slightly) with their deflection. Drive the *Deflection properties from -1..1
/// (sign = direction, magnitude = travel). Data source is mocked for now; later
/// these get bound to actuator-output Facts from PX4 (see project plan Phase 5).
Item {
    id: root

    // Deflection inputs, normalized to [-1, 1].
    property real aileronLeftDeflection:  0
    property real aileronRightDeflection: 0
    property real flapLeftDeflection:     0
    property real flapRightDeflection:    0
    property real elevatorDeflection:     0
    property real rudderDeflection:       0

    // Palette
    property color airframeColor:   Qt.rgba(0.20, 0.22, 0.26, 1.0)
    property color airframeEdge:     Qt.rgba(0.45, 0.48, 0.55, 1.0)
    property color neutralColor:     Qt.rgba(0.35, 0.38, 0.44, 1.0)
    property color positiveColor:    Qt.rgba(0.20, 0.80, 0.45, 1.0)   // e.g. trailing edge down
    property color negativeColor:    Qt.rgba(0.95, 0.55, 0.15, 1.0)   // e.g. trailing edge up

    // Max visual hinge rotation (deg) at full deflection.
    property real maxSurfaceAngle:   22

    /// Blend neutral -> active color by |deflection|; hue picks direction.
    function surfaceColor(v) {
        var m = Math.min(1, Math.abs(v))
        var target = v >= 0 ? root.positiveColor : root.negativeColor
        return Qt.rgba(neutralColor.r + (target.r - neutralColor.r) * m,
                       neutralColor.g + (target.g - neutralColor.g) * m,
                       neutralColor.b + (target.b - neutralColor.b) * m,
                       1.0)
    }

    // Square design space, centered, so the plane keeps its proportions.
    Item {
        id: plane
        readonly property real s: Math.min(root.width, root.height)
        width:  s
        height: s
        anchors.centerIn: parent

        // ---- Airframe (drawn back-to-front) ----

        // Horizontal stabilizer
        Rectangle {
            x: plane.s * 0.30; width: plane.s * 0.40
            y: plane.s * 0.74; height: plane.s * 0.05
            radius: height * 0.4
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }

        // Vertical fin base (fuselage tail)
        // Main wing
        Rectangle {
            x: plane.s * 0.06; width: plane.s * 0.88
            y: plane.s * 0.40; height: plane.s * 0.10
            radius: height * 0.35
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }

        // Fuselage
        Rectangle {
            x: plane.s * 0.455; width: plane.s * 0.09
            y: plane.s * 0.08;  height: plane.s * 0.78
            radius: width * 0.5
            color: root.airframeColor; border.color: root.airframeEdge; border.width: 1
        }

        // Nose
        Rectangle {
            x: plane.s * 0.47; width: plane.s * 0.06
            y: plane.s * 0.03; height: plane.s * 0.10
            radius: width * 0.5
            color: root.airframeEdge
        }

        // ---- Control surfaces (hinge at their leading edge) ----

        // Flaps (inboard wing trailing edge)
        Rectangle {
            x: plane.s * 0.30; width: plane.s * 0.16
            y: plane.s * 0.50; height: plane.s * 0.045
            color: root.surfaceColor(root.flapLeftDeflection)
            border.color: root.airframeEdge; border.width: 1
            transformOrigin: Item.Top
            rotation: -root.flapLeftDeflection * root.maxSurfaceAngle
            Behavior on rotation { NumberAnimation { duration: 120 } }
        }
        Rectangle {
            x: plane.s * 0.54; width: plane.s * 0.16
            y: plane.s * 0.50; height: plane.s * 0.045
            color: root.surfaceColor(root.flapRightDeflection)
            border.color: root.airframeEdge; border.width: 1
            transformOrigin: Item.Top
            rotation: root.flapRightDeflection * root.maxSurfaceAngle
            Behavior on rotation { NumberAnimation { duration: 120 } }
        }

        // Ailerons (outboard wing trailing edge)
        Rectangle {
            x: plane.s * 0.08; width: plane.s * 0.20
            y: plane.s * 0.50; height: plane.s * 0.045
            color: root.surfaceColor(root.aileronLeftDeflection)
            border.color: root.airframeEdge; border.width: 1
            transformOrigin: Item.Top
            rotation: -root.aileronLeftDeflection * root.maxSurfaceAngle
            Behavior on rotation { NumberAnimation { duration: 120 } }
        }
        Rectangle {
            x: plane.s * 0.72; width: plane.s * 0.20
            y: plane.s * 0.50; height: plane.s * 0.045
            color: root.surfaceColor(root.aileronRightDeflection)
            border.color: root.airframeEdge; border.width: 1
            transformOrigin: Item.Top
            rotation: root.aileronRightDeflection * root.maxSurfaceAngle
            Behavior on rotation { NumberAnimation { duration: 120 } }
        }

        // Elevator (stabilizer trailing edge, full span)
        Rectangle {
            x: plane.s * 0.30; width: plane.s * 0.40
            y: plane.s * 0.79; height: plane.s * 0.04
            color: root.surfaceColor(root.elevatorDeflection)
            border.color: root.airframeEdge; border.width: 1
            transformOrigin: Item.Top
            rotation: root.elevatorDeflection * root.maxSurfaceAngle
            Behavior on rotation { NumberAnimation { duration: 120 } }
        }

        // Rudder (tail centerline, deflects left/right -> shown as horizontal swing)
        Rectangle {
            x: plane.s * 0.47; width: plane.s * 0.06
            y: plane.s * 0.80; height: plane.s * 0.13
            radius: width * 0.4
            color: root.surfaceColor(root.rudderDeflection)
            border.color: root.airframeEdge; border.width: 1
            transformOrigin: Item.Top
            rotation: root.rudderDeflection * root.maxSurfaceAngle
            Behavior on rotation { NumberAnimation { duration: 120 } }
        }
    }
}
