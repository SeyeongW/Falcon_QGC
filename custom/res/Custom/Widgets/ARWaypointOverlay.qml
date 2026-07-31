pragma ComponentBehavior: Bound

import QtQuick
import QtQuick3D

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets

/// Augmented-reality overlay for the camera feed. A transparent View3D is used
/// purely as a projection engine: its PerspectiveCamera mirrors the real camera
/// pose (vehicle attitude + a fixed mount tilt), so mission waypoints placed in
/// the local scene project onto the same screen pixels as the live video. The
/// waypoint markers and the path connecting them are then drawn as a 2D HUD on
/// top of the video. This reuses the same geo->local transform and
/// mapFrom3DScene projection approach as MissionRoute3DView.
Item {
    id: root

    // Data sources. Default to the active vehicle / fly-view mission controller
    // so the overlay self-wires, but they can be overridden by the parent.
    property var activeVehicle:     QGroundControl.multiVehicleManager.activeVehicle
    property var missionController:  (typeof globals !== "undefined" && globals.planMasterControllerFlyView)
                                        ? globals.planMasterControllerFlyView.missionController : null

    // Camera intrinsics / mount.
    property real cameraFov:         60     ///< Vertical field of view in degrees. Match the real camera.
    property real cameraPitchOffset: 0      ///< Fixed mount/gimbal tilt in degrees, positive tilts the view down.
    property bool showPath:          true   ///< Draw the path line connecting upcoming waypoints.

    readonly property color _accent: FalconTheme.accent

    // 1 degree of latitude in meters (equirectangular approximation, same value
    // used by MissionRoute3DView).
    readonly property real _metersPerDegree: 111319.49079327358

    // --- Vehicle attitude (degrees). Drives the projection camera. ---
    readonly property real _heading: (activeVehicle && activeVehicle.heading && isFinite(activeVehicle.heading.rawValue))
                                        ? activeVehicle.heading.rawValue : 0
    readonly property real _pitch:   (activeVehicle && activeVehicle.pitch && isFinite(activeVehicle.pitch.rawValue))
                                        ? activeVehicle.pitch.rawValue : 0
    readonly property real _roll:    (activeVehicle && activeVehicle.roll && isFinite(activeVehicle.roll.rawValue))
                                        ? activeVehicle.roll.rawValue : 0

    readonly property bool _vehicleValid: activeVehicle
                                          && _isValidCoordinate(activeVehicle.coordinate)
                                          && isFinite(_vehicleAltitudeAMSL())

    function _isValidCoordinate(coordinate) {
        return coordinate
                && isFinite(coordinate.latitude)
                && isFinite(coordinate.longitude)
                && (Math.abs(coordinate.latitude) > 0.0000001
                    || Math.abs(coordinate.longitude) > 0.0000001)
    }

    function _vehicleAltitudeAMSL() {
        if (!activeVehicle) {
            return Number.NaN
        }
        const factAltitude = activeVehicle.altitudeAMSL
                ? Number(activeVehicle.altitudeAMSL.rawValue)
                : Number.NaN
        if (isFinite(factAltitude)) {
            return factAltitude
        }
        return activeVehicle.coordinate
                ? Number(activeVehicle.coordinate.altitude)
                : Number.NaN
    }

    // Geographic coordinate -> local scene position relative to the vehicle.
    // Right-handed, Y up: x = east, y = up, z = north. The vehicle is the origin
    // so the projection camera sits at (0, 0, 0).
    function _localForCoordinate(coordinate, altitudeAMSL) {
        const reference = activeVehicle.coordinate
        const referenceAltitude = _vehicleAltitudeAMSL()
        const cosLatitude = Math.cos(reference.latitude * Math.PI / 180)
        const east = (coordinate.longitude - reference.longitude)
                * cosLatitude * _metersPerDegree
        const north = (coordinate.latitude - reference.latitude) * _metersPerDegree
        const up = (isFinite(altitudeAMSL) ? altitudeAMSL : referenceAltitude)
                - referenceAltitude
        return Qt.vector3d(east, up, north)
    }

    // Upcoming mission waypoints as { seq, coordinate, altitudeAMSL }. Rebuilt
    // only when the mission changes; the per-frame projection happens below.
    property var _waypoints: []

    function _rebuildWaypoints() {
        const list = []
        const items = missionController ? missionController.visualItems : null
        if (items) {
            for (let i = 0; i < items.count; ++i) {
                const item = items.get(i)
                if (!item || !item.specifiesCoordinate) {
                    continue
                }
                if (!_isValidCoordinate(item.coordinate)) {
                    continue
                }
                list.push({
                    seq: item.sequenceNumber,
                    coordinate: item.coordinate,
                    altitudeAMSL: item.amslEntryAlt
                })
            }
        }
        _waypoints = list
    }

    // Projected screen points for every waypoint, recomputed reactively as the
    // vehicle moves (position) or rotates (attitude). Each entry carries the
    // 2D screen position, whether it is in front of / inside the view, and the
    // horizontal distance to the waypoint in meters.
    readonly property var projectedPoints: {
        // Touch attitude so this binding re-evaluates when the camera rotates
        // (mapFrom3DScene itself is a plain function call, not a tracked read).
        const attitudeTick = _heading + _pitch + _roll + cameraPitchOffset
        const points = []
        if (arView.width <= 0 || arView.height <= 0 || !_vehicleValid) {
            return points
        }
        for (let i = 0; i < _waypoints.length; ++i) {
            const waypoint = _waypoints[i]
            const local = _localForCoordinate(waypoint.coordinate, waypoint.altitudeAMSL)
            const screen = arView.mapFrom3DScene(local)
            const inFront = screen.z > 0
            points.push({
                seq: waypoint.seq,
                x: screen.x,
                y: screen.y,
                inFront: inFront,
                onScreen: inFront
                          && screen.x >= 0 && screen.x <= arView.width
                          && screen.y >= 0 && screen.y <= arView.height,
                distance: Math.sqrt((local.x * local.x) + (local.z * local.z))
            })
        }
        return points
    }

    onActiveVehicleChanged: _rebuildWaypoints()
    onMissionControllerChanged: _rebuildWaypoints()

    Connections {
        target: root.missionController
        ignoreUnknownSignals: true
        function onVisualItemsReset() { root._rebuildWaypoints() }
        function onNewItemsFromVehicle() { root._rebuildWaypoints() }
    }

    Connections {
        target: root.missionController ? root.missionController.visualItems : null
        ignoreUnknownSignals: true
        function onCountChanged() { root._rebuildWaypoints() }
        function onModelReset() { root._rebuildWaypoints() }
    }

    Component.onCompleted: _rebuildWaypoints()

    // --- Projection engine: transparent, no visible 3D content of its own. ---
    View3D {
        id: arView
        anchors.fill: parent
        camera: arCamera

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Transparent
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.Medium
        }

        Node {
            id: cameraRig

            // Vehicle body attitude. Local axes are the body axes (x right, y up,
            // z forward), signs negated to match aircraft convention. cameraPitchOffset
            // adds a fixed downward mount tilt on top of the vehicle pitch.
            eulerRotation.x: -root._pitch + root.cameraPitchOffset
            eulerRotation.y: root._heading
            eulerRotation.z: -root._roll

            PerspectiveCamera {
                id: arCamera
                // A default camera looks down -Z; rotate 180 so it faces +Z (the
                // scene's forward/north direction) at heading 0.
                eulerRotation.y: 180
                fieldOfView: root.cameraFov
                fieldOfViewOrientation: PerspectiveCamera.Vertical
                clipNear: 0.5
                clipFar: 100000
            }
        }
    }

    // --- Path line connecting the projected waypoints (2D HUD). ---
    Canvas {
        id: pathCanvas
        anchors.fill: parent
        visible: root.showPath

        onPaint: {
            const ctx = getContext("2d")
            ctx.reset()
            ctx.lineWidth = 2
            ctx.strokeStyle = root._accent
            ctx.lineJoin = "round"
            ctx.beginPath()
            let drawing = false
            const points = root.projectedPoints
            for (let i = 0; i < points.length; ++i) {
                const point = points[i]
                if (!point.inFront) {
                    drawing = false
                    continue
                }
                if (!drawing) {
                    ctx.moveTo(point.x, point.y)
                    drawing = true
                } else {
                    ctx.lineTo(point.x, point.y)
                }
            }
            ctx.stroke()
        }

        Connections {
            target: root
            function onProjectedPointsChanged() { pathCanvas.requestPaint() }
        }
        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
    }

    // --- Waypoint markers + labels (2D HUD). ---
    Repeater {
        model: root.projectedPoints

        delegate: Item {
            id: marker

            required property var modelData

            visible: modelData.onScreen
            x: modelData.x - (width / 2)
            y: modelData.y - (height / 2)
            width: ScreenTools.defaultFontPixelHeight * 1.4
            height: width
            z: 5

            // Diamond marker.
            Rectangle {
                anchors.centerIn: parent
                width: parent.width * 0.6
                height: width
                rotation: 45
                color: Qt.rgba(0.22, 0.74, 0.97, 0.28)
                border.color: root._accent
                border.width: 2
            }

            // Sequence number above, distance below.
            QGCLabel {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.top
                text: qsTr("WP %1").arg(marker.modelData.seq)
                color: "white"
                font.bold: true
                font.pointSize: ScreenTools.smallFontPointSize
                style: Text.Outline
                styleColor: Qt.rgba(0, 0, 0, 0.85)
            }

            QGCLabel {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.top: parent.bottom
                text: qsTr("%1 m").arg(Math.round(marker.modelData.distance))
                color: root._accent
                font.pointSize: ScreenTools.smallFontPointSize
                style: Text.Outline
                styleColor: Qt.rgba(0, 0, 0, 0.85)
            }
        }
    }
}
