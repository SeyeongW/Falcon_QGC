pragma ComponentBehavior: Bound

import QtQuick
import QtQuick3D

Item {
    id: root

    property var  missionController
    property var  activeVehicle
    property bool missionAvailable: false
    property bool showDebugGeometry: false
    property bool showDebugSequenceNumbers: false

    property real altitudeVisualScale: 1.0
    property real routeThicknessScale: 0.8
    property real waypointSizeScale:   0.8
    property real groundPlaneExtentScale: 1.6

    readonly property int cameraOverview: 0
    readonly property int cameraFollow:   1
    // Body-relative attitude views. The camera rides a rig that carries only the
    // aircraft's heading, so roll and pitch still animate against a fixed frame
    // and can actually be read off the model.
    readonly property int cameraTop:      2   // from above: heading + roll
    readonly property int cameraSide:     3   // from the right wing: pitch
    readonly property int cameraRear:     4   // from the tail: roll

    readonly property bool _attitudeView: cameraMode >= cameraTop

    property real attitudeViewDistance: 26
    property real attitudeViewHeight:   4
    // The top view frames the aircraft against the ground and the surrounding
    // waypoints, not just the airframe, so it pulls back much further than the
    // side and rear views, which exist to read pitch and roll up close.
    property real attitudeTopViewDistance: 72

    property int  cameraMode:          cameraOverview
    property bool automaticCameraMode: true
    property bool followCameraEnabled: true

    property real overviewPitchDegrees:    30
    property real followPitchDegrees:      15
    property real followDistance:          45
    property real followZoomScale:         0.75
    property real minimumFollowDistance:   12
    property real followHeightOffset:      2
    property real followLookAheadDistance: 18

    property int      activeWaypointIndex: -1
    property int      currentLegIndex:     -1
    property real     currentLegProgress:  0.0
    property vector3d aircraftPosition:    Qt.vector3d(0, 0, 0)
    property real     aircraftHeading:     0
    property real     aircraftRoll:        (activeVehicle && activeVehicle.roll && isFinite(activeVehicle.roll.rawValue))
                                               ? activeVehicle.roll.rawValue : 0
    property real     aircraftPitch:       (activeVehicle && activeVehicle.pitch && isFinite(activeVehicle.pitch.rawValue))
                                               ? activeVehicle.pitch.rawValue : 0
    property real     aircraftModelScale:  4.5
    property vector3d aircraftModelRotationOffset: Qt.vector3d(0, -90, 0)
    property var      waypointStates:      []
    property var      missionOccurrences:     []
    property var      physicalWaypointGroups: []
    property var      traversalOccurrences:   []
    property int      missionProgressRevision: 0
    property bool     routeDataValid:      false

    readonly property int waypointPending: 0
    readonly property int waypointActive:  1
    readonly property int waypointPassed:  2
    readonly property int waypointFailed:  3

    readonly property real _metersPerDegree:      111319.49079327358
    readonly property real _coordinateTolerance:  0.0000001
    readonly property real _altitudeTolerance:    0.1
    readonly property real _targetSceneExtent:    100
    readonly property real _minimumRouteExtent:   1
    readonly property real _cameraFillRatio:      0.82
    readonly property real _routeThicknessStyleScale: 0.65
    readonly property real _waypointSizeStyleScale:   0.70
    readonly property real _effectiveFollowDistance:
        Math.max(minimumFollowDistance, followDistance * followZoomScale)
    readonly property real _overviewPitchRadians: overviewPitchDegrees * Math.PI / 180
    readonly property real _cameraOffsetX:        Math.cos(_overviewPitchRadians) * 0.33035
    readonly property real _cameraOffsetY:        Math.sin(_overviewPitchRadians)
    readonly property real _cameraOffsetZ:        Math.cos(_overviewPitchRadians) * 0.94386
    readonly property vector3d worldUp:            Qt.vector3d(0, 1, 0)
    readonly property color _pendingColor:        "#8A939F"
    readonly property color _activeColor:         "#D6B84C"
    readonly property color _passedColor:         "#4E9F75"
    readonly property color _failedColor:         "#C75C5C"
    readonly property color _routeColor:          "#52616D"

    property var _routeSegments: []
    property var _waypointMarkers: []
    property vector3d _sceneCenter: Qt.vector3d(0, 0, 0)
    property real _sceneExtent: 50
    property real _sceneScale: 1
    property real _horizontalSpan: 0
    property real _verticalSpan: 0
    property real _projectedRouteWidth: 50
    property real _projectedRouteHeight: 50
    property real _groundWidth: 50
    property real _groundDepth: 50
    property real _groundY: -1
    property real _markerRadius: 1.5
    property real _segmentRadius: 0.35
    property var _referenceCoordinate: null
    property real _referenceAltitudeAMSL: 0
    property real _minimumRawAltitudeMeters: 0
    property bool _routeTransformValid: false
    property bool _vehiclePositionValid: false
    property bool _lastVehiclePositionValid: false
    property vector3d _lastVehiclePosition: Qt.vector3d(0, 0, 0)
    property vector3d _fallbackAircraftPosition: Qt.vector3d(0, 0, 0)
    property vector3d _desiredFollowCameraPosition: Qt.vector3d(0, 20, -45)
    property vector3d _desiredFollowTarget: Qt.vector3d(0, 0, 18)

    function _isValidCoordinate(coordinate) {
        return coordinate
                && coordinate.isValid
                && isFinite(coordinate.latitude)
                && isFinite(coordinate.longitude)
    }

    function _coordinatesMatch(first, second) {
        return Math.abs(first.latitude - second.latitude) <= _coordinateTolerance
                && Math.abs(first.longitude - second.longitude) <= _coordinateTolerance
    }

    function _physicalWaypointKey(coordinate) {
        if (!_isValidCoordinate(coordinate)) {
            return ""
        }
        return Number(coordinate.latitude).toFixed(7)
                + ","
                + Number(coordinate.longitude).toFixed(7)
    }

    function _physicalWaypointGroup(groupIndex) {
        return physicalWaypointGroups
                && groupIndex >= 0
                && groupIndex < physicalWaypointGroups.length
                ? physicalWaypointGroups[groupIndex]
                : null
    }

    function _itemAltitude(item, useExitAltitude) {
        if (!item) {
            return Number.NaN
        }

        const amslAltitude = Number(useExitAltitude ? item.amslExitAlt : item.amslEntryAlt)
        if (isFinite(amslAltitude)) {
            return amslAltitude
        }

        const alternateAMSLAltitude = Number(useExitAltitude
                                             ? item.amslEntryAlt
                                             : item.amslExitAlt)
        if (isFinite(alternateAMSLAltitude)) {
            return alternateAMSLAltitude
        }
        return Number.NaN
    }

    function _altitudeForCoordinate(coordinate, visualItems, useExitAltitude) {
        if (!visualItems) {
            return Number.NaN
        }

        for (let index = 0; index < visualItems.count; index++) {
            const item = visualItems.get(index)
            if (item && _isValidCoordinate(item.coordinate)
                    && _coordinatesMatch(item.coordinate, coordinate)) {
                const altitude = _itemAltitude(item, useExitAltitude)
                if (isFinite(altitude)) {
                    return altitude
                }
            }
        }
        return Number.NaN
    }

    function _waypointState(markerIndex, physicalWaypointIndex) {
        const group = _physicalWaypointGroup(physicalWaypointIndex)
        if (group) {
            return Number(group.state)
        }

        if (markerIndex >= 0 && markerIndex === activeWaypointIndex) {
            return waypointActive
        }

        if (waypointStates && markerIndex >= 0 && markerIndex < waypointStates.length) {
            const state = Number(waypointStates[markerIndex])
            if (state >= waypointPending && state <= waypointFailed) {
                return state
            }
        }
        return waypointPending
    }

    function _waypointColor(markerIndex, physicalWaypointIndex) {
        switch (_waypointState(markerIndex, physicalWaypointIndex)) {
        case waypointActive:
            return _activeColor
        case waypointPassed:
            return _passedColor
        case waypointFailed:
            return _failedColor
        default:
            return _pendingColor
        }
    }

    function _segmentState(segmentIndex, destinationMarkerIndex, traversalIndex) {
        if (traversalOccurrences
                && traversalIndex >= 0
                && traversalIndex < traversalOccurrences.length) {
            return Number(traversalOccurrences[traversalIndex].state)
        }
        if (_waypointState(destinationMarkerIndex, -1) === waypointFailed) {
            return waypointFailed
        }
        if (segmentIndex === currentLegIndex) {
            return waypointActive
        }
        if (_waypointState(destinationMarkerIndex, -1) === waypointPassed) {
            return waypointPassed
        }
        return waypointPending
    }

    function _segmentColor(segmentIndex, destinationMarkerIndex, traversalIndex) {
        switch (_segmentState(segmentIndex,
                              destinationMarkerIndex,
                              traversalIndex)) {
        case waypointActive:
            return _activeColor
        case waypointPassed:
            return _passedColor
        case waypointFailed:
            return _failedColor
        default:
            return _routeColor
        }
    }

    function _physicalWaypointLabel(groupIndex) {
        const group = _physicalWaypointGroup(groupIndex)
        return group ? group.label : ""
    }

    function _physicalWaypointDebugLabel(groupIndex) {
        const group = _physicalWaypointGroup(groupIndex)
        return group && group.sequenceNumbers.length > 0
                ? qsTr("SEQ %1").arg(group.sequenceNumbers.join("·"))
                : ""
    }

    function _rawPoint(coordinate, altitude, referenceCoordinate, referenceAltitude) {
        const cosLatitude = Math.cos(referenceCoordinate.latitude * Math.PI / 180)
        const xMeters = (coordinate.longitude - referenceCoordinate.longitude)
                * cosLatitude * _metersPerDegree
        // Z is NEGATED north. The overview camera sits on +Z and looks toward the
        // origin, so its forward is -Z and its right is +X: north has to live at
        // -Z to recede into the screen (north-up) while east stays on the right.
        // With north at +Z the scene came out mirrored against the 2D map.
        // aircraftHeading is derived from these same scene coordinates, so it
        // follows the flip automatically.
        const zMeters = -(coordinate.latitude - referenceCoordinate.latitude)
                * _metersPerDegree
        const relativeAltitude = altitude - referenceAltitude

        return {
            rawX: xMeters,
            rawAltitude: relativeAltitude,
            rawZ: zMeters,
            position: Qt.vector3d(xMeters,
                                  relativeAltitude * altitudeVisualScale,
                                  zMeters)
        }
    }

    function _scaledPosition(position, scale) {
        return Qt.vector3d(position.x * scale,
                           position.y * scale,
                           position.z * scale)
    }

    function localPositionForCoordinate(coordinate, altitudeAMSL) {
        if (!_routeTransformValid
                || !_isValidCoordinate(coordinate)
                || !isFinite(altitudeAMSL)) {
            return null
        }

        const rawPoint = _rawPoint(coordinate,
                                   altitudeAMSL,
                                   _referenceCoordinate,
                                   _referenceAltitudeAMSL)
        return _scaledPosition(rawPoint.position, _sceneScale)
    }

    function _distance(first, second) {
        const dx = second.x - first.x
        const dy = second.y - first.y
        const dz = second.z - first.z
        return Math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
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

        const coordinateAltitude = activeVehicle.coordinate
                ? Number(activeVehicle.coordinate.altitude)
                : Number.NaN
        return isFinite(coordinateAltitude) ? coordinateAltitude : Number.NaN
    }

    /// Repaint the imported airframe in the light-sky accent. The balsam-generated
    /// model ships a near-white base colour which, under this scene's dim lighting
    /// and against the black background, resolved to an almost invisible silhouette.
    /// The materials are shared instances inside the generated component, so
    /// assigning through the exposed nodes recolours the whole airframe.
    function _tintAircraft(aircraft) {
        if (!aircraft) {
            return
        }
        const bodyColor = "#82CFFF"     // FalconTheme.accent
        const propColor = "#33B1FF"     // FalconTheme.accentDeep
        const parts = [
            { node: aircraft.aircraftBodyNode, color: bodyColor },
            { node: aircraft.liftPropFlNode,   color: propColor },
            { node: aircraft.liftPropFrNode,   color: propColor },
            { node: aircraft.liftPropRlNode,   color: propColor },
            { node: aircraft.liftPropRrNode,   color: propColor },
            { node: aircraft.pusherPropNode,   color: propColor }
        ]
        for (let i = 0; i < parts.length; i++) {
            const part = parts[i]
            if (!part.node || !part.node.materials) {
                continue
            }
            for (let m = 0; m < part.node.materials.length; m++) {
                const material = part.node.materials[m]
                if (material) {
                    material.baseColor = part.color
                }
            }
        }
    }

    function _currentForwardDirection() {
        if (activeVehicle && activeVehicle.heading) {
            const headingDegrees = Number(activeVehicle.heading.rawValue)
            if (isFinite(headingDegrees)
                    && headingDegrees >= 0
                    && headingDegrees < 360) {
                const headingRadians = headingDegrees * Math.PI / 180
                // Z is negated north (see _rawPoint), so a compass heading maps
                // to -cos on Z. Without the negation heading 0 pointed at +Z,
                // which is south in this scene, and the aircraft flew tail-first.
                return Qt.vector3d(Math.sin(headingRadians),
                                   0,
                                   -Math.cos(headingRadians))
            }
        }

        if (currentLegIndex >= 0 && currentLegIndex < _routeSegments.length) {
            const segment = _routeSegments[currentLegIndex]
            const directionX = segment.point2.x - segment.point1.x
            const directionZ = segment.point2.z - segment.point1.z
            const directionLength = Math.sqrt((directionX * directionX)
                                              + (directionZ * directionZ))
            if (directionLength > 0.001) {
                return Qt.vector3d(directionX / directionLength,
                                   0,
                                   directionZ / directionLength)
            }
        }

        const fallbackLength = Math.max(0.001,
                                        Math.sqrt((_cameraOffsetX * _cameraOffsetX)
                                                  + (_cameraOffsetZ * _cameraOffsetZ)))
        return Qt.vector3d(_cameraOffsetX / fallbackLength,
                           0,
                           _cameraOffsetZ / fallbackLength)
    }

    function _updateFollowCameraPose() {
        const forwardDirection = _currentForwardDirection()
        const pitchRadians = followPitchDegrees * Math.PI / 180
        const cameraHeight = followHeightOffset
                + (Math.tan(pitchRadians) * _effectiveFollowDistance)

        _desiredFollowTarget = Qt.vector3d(
                    aircraftPosition.x
                    + (forwardDirection.x * followLookAheadDistance),
                    aircraftPosition.y,
                    aircraftPosition.z
                    + (forwardDirection.z * followLookAheadDistance))
        _desiredFollowCameraPosition = Qt.vector3d(
                    aircraftPosition.x
                    - (forwardDirection.x * _effectiveFollowDistance),
                    aircraftPosition.y + cameraHeight,
                    aircraftPosition.z
                    - (forwardDirection.z * _effectiveFollowDistance))
        aircraftHeading = Math.atan2(forwardDirection.x,
                                     forwardDirection.z) * 180 / Math.PI
    }

    function _stableFollowCameraRotation(cameraPosition, cameraTarget) {
        const targetOffset = cameraTarget.minus(cameraPosition)
        if (targetOffset.length() < 0.001) {
            return Qt.quaternion(1, 0, 0, 0)
        }

        const forward = targetOffset.normalized()
        let right = forward.crossProduct(worldUp)
        if (right.length() < 0.001) {
            const fallbackUp = Math.abs(forward.z) < 0.9
                    ? Qt.vector3d(0, 0, 1)
                    : Qt.vector3d(1, 0, 0)
            right = forward.crossProduct(fallbackUp)
        }
        right = right.normalized()

        let correctedUp = right.crossProduct(forward).normalized()
        if (correctedUp.dotProduct(worldUp) < 0) {
            right = right.times(-1)
            correctedUp = correctedUp.times(-1)
        }

        const backward = forward.times(-1)
        const m00 = right.x
        const m01 = correctedUp.x
        const m02 = backward.x
        const m10 = right.y
        const m11 = correctedUp.y
        const m12 = backward.y
        const m20 = right.z
        const m21 = correctedUp.z
        const m22 = backward.z
        const trace = m00 + m11 + m22

        let scalar
        let x
        let y
        let z
        if (trace > 0) {
            const scale = Math.sqrt(trace + 1) * 2
            scalar = 0.25 * scale
            x = (m21 - m12) / scale
            y = (m02 - m20) / scale
            z = (m10 - m01) / scale
        } else if (m00 > m11 && m00 > m22) {
            const scale = Math.sqrt(1 + m00 - m11 - m22) * 2
            scalar = (m21 - m12) / scale
            x = 0.25 * scale
            y = (m01 + m10) / scale
            z = (m02 + m20) / scale
        } else if (m11 > m22) {
            const scale = Math.sqrt(1 + m11 - m00 - m22) * 2
            scalar = (m02 - m20) / scale
            x = (m01 + m10) / scale
            y = 0.25 * scale
            z = (m12 + m21) / scale
        } else {
            const scale = Math.sqrt(1 + m22 - m00 - m11) * 2
            scalar = (m10 - m01) / scale
            x = (m02 + m20) / scale
            y = (m12 + m21) / scale
            z = 0.25 * scale
        }

        return Qt.quaternion(scalar, x, y, z).normalized()
    }

    function _updateAutomaticCameraMode() {
        if (!automaticCameraMode) {
            return
        }

        if (!followCameraEnabled
                || !activeVehicle
                || !_vehiclePositionValid
                || !activeVehicle.armed) {
            cameraMode = cameraOverview
            return
        }

        const altitudeAMSL = _vehicleAltitudeAMSL()
        const relativeAltitude = altitudeAMSL - _referenceAltitudeAMSL
        const aboveTakeoffThreshold = isFinite(relativeAltitude)
                && (relativeAltitude - _minimumRawAltitudeMeters) > 2
        cameraMode = activeVehicle.flying
                || activeVehicle.landing
                || aboveTakeoffThreshold
                ? cameraFollow
                : cameraOverview
    }

    function _updateVehiclePosition() {
        if (!activeVehicle
                || !_routeTransformValid
                || !_isValidCoordinate(activeVehicle.coordinate)) {
            _vehiclePositionValid = false
            _lastVehiclePositionValid = false
            aircraftPosition = _fallbackAircraftPosition
            _updateFollowCameraPose()
            _updateAutomaticCameraMode()
            return
        }

        const altitudeAMSL = _vehicleAltitudeAMSL()
        const candidatePosition = localPositionForCoordinate(activeVehicle.coordinate,
                                                             altitudeAMSL)
        if (candidatePosition === null) {
            _vehiclePositionValid = false
            aircraftPosition = _fallbackAircraftPosition
            _updateFollowCameraPose()
            _updateAutomaticCameraMode()
            return
        }

        const maximumAcceptedJump = Math.max(35, _sceneExtent * 0.4)
        if (_lastVehiclePositionValid
                && _distance(_lastVehiclePosition, candidatePosition)
                   > maximumAcceptedJump) {
            _updateAutomaticCameraMode()
            return
        }

        _lastVehiclePosition = candidatePosition
        _lastVehiclePositionValid = true
        _vehiclePositionValid = true
        aircraftPosition = candidatePosition
        _updateFollowCameraPose()
        _updateAutomaticCameraMode()
    }

    function _midpoint(first, second) {
        return Qt.vector3d((first.x + second.x) / 2,
                           (first.y + second.y) / 2,
                           (first.z + second.z) / 2)
    }

    function _findMarkerIndex(markers, coordinate, altitude) {
        for (let index = 0; index < markers.length; index++) {
            const marker = markers[index]
            if (_coordinatesMatch(marker.coordinate, coordinate)
                    && Math.abs(marker.altitude - altitude) <= _altitudeTolerance) {
                return index
            }
        }
        return -1
    }

    function positionForLegProgress(legIndex, legProgress) {
        if (legIndex < 0 || legIndex >= _routeSegments.length) {
            return aircraftPosition
        }

        const segment = _routeSegments[legIndex]
        const numericProgress = Number(legProgress)
        const progress = isFinite(numericProgress)
                ? Math.max(0, Math.min(1, numericProgress))
                : 0
        return Qt.vector3d(segment.point1.x + ((segment.point2.x - segment.point1.x) * progress),
                           segment.point1.y + ((segment.point2.y - segment.point1.y) * progress),
                           segment.point1.z + ((segment.point2.z - segment.point1.z) * progress))
    }

    function logDebugState(reason) {
        console.log("[MissionRoute3DView Debug]", reason,
                    "component created:", true,
                    "missionController:", missionController !== null
                                         && missionController !== undefined,
                    "waypoints:", _waypointMarkers.length,
                    "segments:", _routeSegments.length,
                    "routeDataValid:", routeDataValid,
                    "sceneScale:", _sceneScale,
                    "horizontal span:", _horizontalSpan,
                    "vertical span:", _verticalSpan,
                    "View3D size:", routeView3D.width, "x", routeView3D.height,
                    "camera valid:", routeView3D.camera !== null,
                    "camera position:", routeView3D.camera.position,
                    "clipNear:", routeView3D.camera.clipNear,
                    "clipFar:", routeView3D.camera.clipFar)
    }

    function _clearScene() {
        _routeSegments = []
        _waypointMarkers = []
        _sceneCenter = Qt.vector3d(0, 0, 0)
        _sceneExtent = 50
        _sceneScale = 1
        _horizontalSpan = 0
        _verticalSpan = 0
        _projectedRouteWidth = 50
        _projectedRouteHeight = 50
        _groundWidth = 50
        _groundDepth = 50
        _groundY = -1
        _markerRadius = 1.5
        _segmentRadius = 0.35
        _referenceCoordinate = null
        _referenceAltitudeAMSL = 0
        _minimumRawAltitudeMeters = 0
        _routeTransformValid = false
        _vehiclePositionValid = false
        _lastVehiclePositionValid = false
        _fallbackAircraftPosition = Qt.vector3d(0, 0, 0)
        aircraftPosition = Qt.vector3d(0, 0, 0)
        routeDataValid = false
        _updateFollowCameraPose()
        _updateAutomaticCameraMode()
    }

    function rebuildScene() {
        if (!missionAvailable || !missionController) {
            _clearScene()
            return
        }

        _routeTransformValid = false
        _lastVehiclePositionValid = false

        const visualItems = missionController.visualItems
        const flightSegments = missionController.simpleFlightPathSegments
        const geoSegments = []
        let referenceCoordinate = null
        let referenceAltitude = Number.NaN

        if (flightSegments) {
            for (let index = 0; index < flightSegments.count; index++) {
                const segment = flightSegments.get(index)
                if (!segment
                        || !_isValidCoordinate(segment.coordinate1)
                        || !_isValidCoordinate(segment.coordinate2)) {
                    continue
                }

                let altitude1 = Number(segment.coord1AMSLAlt)
                let altitude2 = Number(segment.coord2AMSLAlt)
                if (!isFinite(altitude1)) {
                    altitude1 = _altitudeForCoordinate(segment.coordinate1,
                                                       visualItems,
                                                       true)
                }
                if (!isFinite(altitude2)) {
                    altitude2 = _altitudeForCoordinate(segment.coordinate2,
                                                       visualItems,
                                                       false)
                }

                geoSegments.push({
                    coordinate1: segment.coordinate1,
                    coordinate2: segment.coordinate2,
                    altitude1: altitude1,
                    altitude2: altitude2
                })

                if (!referenceCoordinate) {
                    referenceCoordinate = segment.coordinate1
                    if (isFinite(altitude1)) {
                        referenceAltitude = altitude1
                    }
                }
            }
        }

        if (!referenceCoordinate && visualItems) {
            for (let index = 0; index < visualItems.count; index++) {
                const item = visualItems.get(index)
                if (item && item.homePosition && _isValidCoordinate(item.coordinate)) {
                    referenceCoordinate = item.coordinate
                    referenceAltitude = _itemAltitude(item)
                    break
                }
            }
        }

        if (!referenceCoordinate && visualItems) {
            for (let index = 0; index < visualItems.count; index++) {
                const item = visualItems.get(index)
                if (item && _isValidCoordinate(item.coordinate)
                        && (item.homePosition
                            || (item.specifiesCoordinate && !item.isStandaloneCoordinate))) {
                    referenceCoordinate = item.coordinate
                    referenceAltitude = _itemAltitude(item)
                    break
                }
            }
        }

        if (!referenceCoordinate) {
            _clearScene()
            return
        }

        if (!isFinite(referenceAltitude)) {
            for (let index = 0; index < geoSegments.length; index++) {
                if (isFinite(geoSegments[index].altitude1)) {
                    referenceAltitude = geoSegments[index].altitude1
                    break
                }
                if (isFinite(geoSegments[index].altitude2)) {
                    referenceAltitude = geoSegments[index].altitude2
                    break
                }
            }
        }
        if (!isFinite(referenceAltitude)) {
            referenceAltitude = 0
        }

        const physicalGroupIndexByKey = ({})
        for (let groupIndex = 0;
             groupIndex < physicalWaypointGroups.length;
             groupIndex++) {
            const coordinateKeys = physicalWaypointGroups[groupIndex].coordinateKeys
            if (coordinateKeys) {
                for (let keyIndex = 0; keyIndex < coordinateKeys.length; keyIndex++) {
                    physicalGroupIndexByKey[coordinateKeys[keyIndex]] = groupIndex
                }
            } else {
                physicalGroupIndexByKey[
                            physicalWaypointGroups[groupIndex].physicalWaypointKey] = groupIndex
            }
        }

        const geoMarkers = []
        if (visualItems) {
            for (let index = 0; index < visualItems.count; index++) {
                const item = visualItems.get(index)
                if (!item
                        || !_isValidCoordinate(item.coordinate)
                        || (!item.homePosition
                            && (!item.specifiesCoordinate || item.isStandaloneCoordinate))) {
                    continue
                }

                let altitude = _itemAltitude(item)
                if (!isFinite(altitude)) {
                    altitude = referenceAltitude
                }
                const dedicatedMarker = item.homePosition
                        || item.isTakeoffItem
                        || item.isLandCommand
                const sequenceNumber = dedicatedMarker
                        ? -1
                        : Number(item.sequenceNumber)
                const physicalKey = _physicalWaypointKey(item.coordinate)
                const physicalGroupIndex = physicalGroupIndexByKey[physicalKey] === undefined
                        ? -1
                        : physicalGroupIndexByKey[physicalKey]
                const markerIndex = _findMarkerIndex(geoMarkers, item.coordinate, altitude)
                if (markerIndex < 0) {
                    geoMarkers.push({
                        coordinate: item.coordinate,
                        altitude: altitude,
                        physicalWaypointKey: physicalKey,
                        physicalWaypointIndex: physicalGroupIndex,
                        isHome: Boolean(item.homePosition),
                        isTakeoff: Boolean(item.isTakeoffItem),
                        isLand: Boolean(item.isLandCommand),
                        sequenceNumber: isFinite(sequenceNumber) ? sequenceNumber : -1
                    })
                } else {
                    const existingMarker = geoMarkers[markerIndex]
                    existingMarker.isHome = existingMarker.isHome || item.homePosition
                    existingMarker.isTakeoff = existingMarker.isTakeoff || item.isTakeoffItem
                    existingMarker.isLand = existingMarker.isLand || item.isLandCommand
                    if (existingMarker.isHome
                            || existingMarker.isTakeoff
                            || existingMarker.isLand) {
                        existingMarker.sequenceNumber = -1
                    } else if (existingMarker.sequenceNumber < 0
                               && isFinite(sequenceNumber)) {
                        existingMarker.sequenceNumber = sequenceNumber
                    }
                }
            }
        }

        for (let index = 0; index < geoSegments.length; index++) {
            const segment = geoSegments[index]
            const altitude1 = isFinite(segment.altitude1) ? segment.altitude1 : referenceAltitude
            const altitude2 = isFinite(segment.altitude2) ? segment.altitude2 : referenceAltitude
            if (_findMarkerIndex(geoMarkers, segment.coordinate1, altitude1) < 0) {
                geoMarkers.push({
                    coordinate: segment.coordinate1,
                    altitude: altitude1,
                    physicalWaypointKey: _physicalWaypointKey(segment.coordinate1),
                    physicalWaypointIndex: physicalGroupIndexByKey[
                                _physicalWaypointKey(segment.coordinate1)] === undefined
                            ? -1
                            : physicalGroupIndexByKey[
                                _physicalWaypointKey(segment.coordinate1)],
                    isHome: false,
                    isTakeoff: false,
                    isLand: false,
                    sequenceNumber: -1
                })
            }
            if (_findMarkerIndex(geoMarkers, segment.coordinate2, altitude2) < 0) {
                geoMarkers.push({
                    coordinate: segment.coordinate2,
                    altitude: altitude2,
                    physicalWaypointKey: _physicalWaypointKey(segment.coordinate2),
                    physicalWaypointIndex: physicalGroupIndexByKey[
                                _physicalWaypointKey(segment.coordinate2)] === undefined
                            ? -1
                            : physicalGroupIndexByKey[
                                _physicalWaypointKey(segment.coordinate2)],
                    isHome: false,
                    isTakeoff: false,
                    isLand: false,
                    sequenceNumber: -1
                })
            }
        }

        const labeledCoordinates = []
        for (let index = 0; index < geoMarkers.length; index++) {
            const marker = geoMarkers[index]
            if (marker.sequenceNumber < 0) {
                continue
            }

            let coordinateAlreadyLabeled = false
            for (let coordinateIndex = 0;
                 coordinateIndex < labeledCoordinates.length;
                 coordinateIndex++) {
                if (_coordinatesMatch(marker.coordinate,
                                      labeledCoordinates[coordinateIndex])) {
                    coordinateAlreadyLabeled = true
                    break
                }
            }

            if (coordinateAlreadyLabeled) {
                marker.sequenceNumber = -1
            } else {
                labeledCoordinates.push(marker.coordinate)
            }
        }

        // QGC intentionally flattens some home-to-takeoff flight-path segments.
        // Restore only vertical legs proven by co-located mission items with
        // different AMSL altitudes; no screen-space height is invented here.
        for (let firstIndex = 0; firstIndex < geoMarkers.length; firstIndex++) {
            for (let secondIndex = firstIndex + 1;
                 secondIndex < geoMarkers.length;
                 secondIndex++) {
                const firstMarker = geoMarkers[firstIndex]
                const secondMarker = geoMarkers[secondIndex]
                if (!_coordinatesMatch(firstMarker.coordinate, secondMarker.coordinate)
                        || Math.abs(firstMarker.altitude - secondMarker.altitude)
                           <= _altitudeTolerance
                        || (!firstMarker.isTakeoff
                            && !firstMarker.isLand
                            && !secondMarker.isTakeoff
                            && !secondMarker.isLand)) {
                    continue
                }

                let segmentAlreadyPresent = false
                for (let segmentIndex = 0;
                     segmentIndex < geoSegments.length;
                     segmentIndex++) {
                    const existingSegment = geoSegments[segmentIndex]
                    const sameDirection = _coordinatesMatch(existingSegment.coordinate1,
                                                            firstMarker.coordinate)
                            && _coordinatesMatch(existingSegment.coordinate2,
                                                 secondMarker.coordinate)
                            && Math.abs(existingSegment.altitude1 - firstMarker.altitude)
                               <= _altitudeTolerance
                            && Math.abs(existingSegment.altitude2 - secondMarker.altitude)
                               <= _altitudeTolerance
                    const reverseDirection = _coordinatesMatch(existingSegment.coordinate1,
                                                               secondMarker.coordinate)
                            && _coordinatesMatch(existingSegment.coordinate2,
                                                 firstMarker.coordinate)
                            && Math.abs(existingSegment.altitude1 - secondMarker.altitude)
                               <= _altitudeTolerance
                            && Math.abs(existingSegment.altitude2 - firstMarker.altitude)
                               <= _altitudeTolerance
                    if (sameDirection || reverseDirection) {
                        segmentAlreadyPresent = true
                        break
                    }
                }

                if (!segmentAlreadyPresent) {
                    const ascending = firstMarker.altitude <= secondMarker.altitude
                    geoSegments.push({
                        coordinate1: ascending ? firstMarker.coordinate : secondMarker.coordinate,
                        coordinate2: ascending ? secondMarker.coordinate : firstMarker.coordinate,
                        altitude1: ascending ? firstMarker.altitude : secondMarker.altitude,
                        altitude2: ascending ? secondMarker.altitude : firstMarker.altitude
                    })
                }
            }
        }

        const rawWaypointMarkers = []
        const emittedPhysicalWaypointKeys = ({})
        for (let index = 0; index < geoMarkers.length; index++) {
            const geoMarker = geoMarkers[index]
            if (geoMarker.physicalWaypointIndex >= 0) {
                if (emittedPhysicalWaypointKeys[geoMarker.physicalWaypointKey]) {
                    continue
                }
                emittedPhysicalWaypointKeys[geoMarker.physicalWaypointKey] = true
            }
            const rawPoint = _rawPoint(geoMarker.coordinate,
                                       geoMarker.altitude,
                                       referenceCoordinate,
                                       referenceAltitude)
            rawWaypointMarkers.push({
                position: rawPoint.position,
                rawAltitude: rawPoint.rawAltitude,
                markerIndex: index,
                sequenceNumber: geoMarker.sequenceNumber,
                physicalWaypointKey: geoMarker.physicalWaypointKey,
                physicalWaypointIndex: geoMarker.physicalWaypointIndex
            })
        }

        const traversalQueues = ({})
        for (let traversalIndex = 0;
             traversalIndex < traversalOccurrences.length;
             traversalIndex++) {
            const traversal = traversalOccurrences[traversalIndex]
            const directionKey = traversal.startPhysicalWaypointKey
                    + ">"
                    + traversal.endPhysicalWaypointKey
            if (traversalQueues[directionKey] === undefined) {
                traversalQueues[directionKey] = []
            }
            traversalQueues[directionKey].push(traversalIndex)
        }

        const rawRouteSegments = []
        for (let index = 0; index < geoSegments.length; index++) {
            const geoSegment = geoSegments[index]
            const altitude1 = isFinite(geoSegment.altitude1)
                    ? geoSegment.altitude1
                    : referenceAltitude
            const altitude2 = isFinite(geoSegment.altitude2)
                    ? geoSegment.altitude2
                    : referenceAltitude
            const rawPoint1 = _rawPoint(geoSegment.coordinate1,
                                        altitude1,
                                        referenceCoordinate,
                                        referenceAltitude)
            const rawPoint2 = _rawPoint(geoSegment.coordinate2,
                                        altitude2,
                                        referenceCoordinate,
                                        referenceAltitude)
            const length = _distance(rawPoint1.position, rawPoint2.position)
            if (length <= 0.0001) {
                continue
            }

            const startPhysicalKey = _physicalWaypointKey(geoSegment.coordinate1)
            const endPhysicalKey = _physicalWaypointKey(geoSegment.coordinate2)
            const directionKey = startPhysicalKey + ">" + endPhysicalKey
            const matchingTraversals = traversalQueues[directionKey]
            const traversalIndex = matchingTraversals && matchingTraversals.length > 0
                    ? matchingTraversals.shift()
                    : -1
            let destinationMarkerIndex = -1
            for (let markerIndex = 0;
                 markerIndex < rawWaypointMarkers.length;
                 markerIndex++) {
                const rawMarker = rawWaypointMarkers[markerIndex]
                if (rawMarker.physicalWaypointKey === endPhysicalKey) {
                    destinationMarkerIndex = markerIndex
                    break
                }
            }
            rawRouteSegments.push({
                point1: rawPoint1.position,
                point2: rawPoint2.position,
                rawAltitude1: rawPoint1.rawAltitude,
                rawAltitude2: rawPoint2.rawAltitude,
                destinationMarkerIndex: destinationMarkerIndex,
                traversalIndex: traversalIndex
            })
        }

        if (rawWaypointMarkers.length === 0 && rawRouteSegments.length === 0) {
            _clearScene()
            return
        }

        let rawMinX = Number.POSITIVE_INFINITY
        let rawMaxX = Number.NEGATIVE_INFINITY
        let rawMinY = Number.POSITIVE_INFINITY
        let rawMaxY = Number.NEGATIVE_INFINITY
        let rawMinZ = Number.POSITIVE_INFINITY
        let rawMaxZ = Number.NEGATIVE_INFINITY

        function includeRawPosition(position) {
            rawMinX = Math.min(rawMinX, position.x)
            rawMaxX = Math.max(rawMaxX, position.x)
            rawMinY = Math.min(rawMinY, position.y)
            rawMaxY = Math.max(rawMaxY, position.y)
            rawMinZ = Math.min(rawMinZ, position.z)
            rawMaxZ = Math.max(rawMaxZ, position.z)
        }

        for (let index = 0; index < rawWaypointMarkers.length; index++) {
            includeRawPosition(rawWaypointMarkers[index].position)
        }
        for (let index = 0; index < rawRouteSegments.length; index++) {
            includeRawPosition(rawRouteSegments[index].point1)
            includeRawPosition(rawRouteSegments[index].point2)
        }

        const rawRangeX = Math.max(0, rawMaxX - rawMinX)
        const rawRangeY = Math.max(0, rawMaxY - rawMinY)
        const rawRangeZ = Math.max(0, rawMaxZ - rawMinZ)
        const rawHorizontalExtent = Math.max(rawRangeX, rawRangeZ)
        const safeHorizontalExtent = Math.max(_minimumRouteExtent,
                                              rawHorizontalExtent)
        const sceneScale = _targetSceneExtent / safeHorizontalExtent

        const waypointMarkers = []
        for (let index = 0; index < rawWaypointMarkers.length; index++) {
            const rawMarker = rawWaypointMarkers[index]
            waypointMarkers.push({
                position: _scaledPosition(rawMarker.position, sceneScale),
                rawAltitude: rawMarker.rawAltitude,
                markerIndex: rawMarker.markerIndex,
                sequenceNumber: rawMarker.sequenceNumber,
                physicalWaypointIndex: rawMarker.physicalWaypointIndex
            })
        }

        const routeSegments = []
        for (let index = 0; index < rawRouteSegments.length; index++) {
            const rawSegment = rawRouteSegments[index]
            const point1 = _scaledPosition(rawSegment.point1, sceneScale)
            const point2 = _scaledPosition(rawSegment.point2, sceneScale)
            const traversal = rawSegment.traversalIndex >= 0
                    && rawSegment.traversalIndex < traversalOccurrences.length
                    ? traversalOccurrences[rawSegment.traversalIndex]
                    : null
            const laneOffset = traversal
                    ? traversal.laneOffsetFactor * 0.8
                    : 0
            const renderPoint1 = Qt.vector3d(point1.x,
                                              point1.y + laneOffset,
                                              point1.z)
            const renderPoint2 = Qt.vector3d(point2.x,
                                              point2.y + laneOffset,
                                              point2.z)
            routeSegments.push({
                point1: point1,
                point2: point2,
                rawAltitude1: rawSegment.rawAltitude1,
                rawAltitude2: rawSegment.rawAltitude2,
                midpoint: _midpoint(renderPoint1, renderPoint2),
                renderPoint2: renderPoint2,
                length: _distance(point1, point2),
                destinationMarkerIndex: rawSegment.destinationMarkerIndex,
                traversalIndex: rawSegment.traversalIndex
            })
        }

        const minX = rawMinX * sceneScale
        const maxX = rawMaxX * sceneScale
        const minY = rawMinY * sceneScale
        const maxY = rawMaxY * sceneScale
        const minZ = rawMinZ * sceneScale
        const maxZ = rawMaxZ * sceneScale
        const rangeX = rawRangeX * sceneScale
        const rangeY = rawRangeY * sceneScale
        const rangeZ = rawRangeZ * sceneScale
        const horizontalExtent = Math.max(rangeX, rangeZ)
        const sceneExtent = Math.max(10, rangeX, rangeY, rangeZ)
        const markerRadius = Math.max(1.8, Math.min(3.2, sceneExtent * 0.025))
        const segmentRadius = Math.max(0.55, Math.min(0.95, sceneExtent * 0.0065))
        const sceneCenter = Qt.vector3d((minX + maxX) / 2,
                                        (minY + maxY) / 2,
                                        (minZ + maxZ) / 2)

        const cameraOffsetLength = Math.sqrt((_cameraOffsetX * _cameraOffsetX)
                                             + (_cameraOffsetY * _cameraOffsetY)
                                             + (_cameraOffsetZ * _cameraOffsetZ))
        const forwardX = -_cameraOffsetX / cameraOffsetLength
        const forwardY = -_cameraOffsetY / cameraOffsetLength
        const forwardZ = -_cameraOffsetZ / cameraOffsetLength
        const rightLength = Math.max(0.001,
                                     Math.sqrt((forwardZ * forwardZ)
                                               + (forwardX * forwardX)))
        const rightX = -forwardZ / rightLength
        const rightZ = forwardX / rightLength
        const upX = -rightZ * forwardY
        const upY = (rightZ * forwardX) - (rightX * forwardZ)
        const upZ = rightX * forwardY
        let projectedHalfWidth = 0
        let projectedHalfHeight = 0

        function includeProjectedPosition(position) {
            const relativeX = position.x - sceneCenter.x
            const relativeY = position.y - sceneCenter.y
            const relativeZ = position.z - sceneCenter.z
            const projectedX = (relativeX * rightX) + (relativeZ * rightZ)
            const projectedY = (relativeX * upX)
                    + (relativeY * upY)
                    + (relativeZ * upZ)
            projectedHalfWidth = Math.max(projectedHalfWidth, Math.abs(projectedX))
            projectedHalfHeight = Math.max(projectedHalfHeight, Math.abs(projectedY))
        }

        for (let index = 0; index < waypointMarkers.length; index++) {
            includeProjectedPosition(waypointMarkers[index].position)
        }
        for (let index = 0; index < routeSegments.length; index++) {
            includeProjectedPosition(routeSegments[index].point1)
            includeProjectedPosition(routeSegments[index].point2)
        }

        _sceneCenter = sceneCenter
        _sceneExtent = sceneExtent
        _sceneScale = sceneScale
        _horizontalSpan = horizontalExtent
        _verticalSpan = rangeY
        _projectedRouteWidth = Math.max(_minimumRouteExtent, projectedHalfWidth * 2)
        _projectedRouteHeight = Math.max(_minimumRouteExtent, projectedHalfHeight * 2)
        _markerRadius = markerRadius
        _segmentRadius = segmentRadius
        _groundWidth = Math.max(rangeX * groundPlaneExtentScale,
                                horizontalExtent * 0.15,
                                markerRadius * 5)
        _groundDepth = Math.max(rangeZ * groundPlaneExtentScale,
                                horizontalExtent * 0.15,
                                markerRadius * 5)
        _groundY = minY - Math.max(markerRadius * 1.35, segmentRadius * 2)
        _routeSegments = routeSegments
        _waypointMarkers = waypointMarkers
        _referenceCoordinate = referenceCoordinate
        _referenceAltitudeAMSL = referenceAltitude
        _routeTransformValid = true

        let initialAircraftPosition = routeSegments.length > 0
                ? routeSegments[0].point1
                : waypointMarkers[0].position
        let minimumRawAltitude = Number.POSITIVE_INFINITY
        for (let index = 0; index < waypointMarkers.length; index++) {
            minimumRawAltitude = Math.min(minimumRawAltitude,
                                          waypointMarkers[index].rawAltitude)
        }
        for (let index = 0; index < routeSegments.length; index++) {
            const segment = routeSegments[index]
            if (segment.rawAltitude1 > minimumRawAltitude + _altitudeTolerance) {
                initialAircraftPosition = segment.point1
                break
            }
            if (segment.rawAltitude2 > minimumRawAltitude + _altitudeTolerance) {
                initialAircraftPosition = segment.point2
                break
            }
        }

        _minimumRawAltitudeMeters = minimumRawAltitude
        _fallbackAircraftPosition = initialAircraftPosition
        aircraftPosition = initialAircraftPosition
        routeDataValid = true
        _updateVehiclePosition()
    }

    View3D {
        id: routeView3D

        anchors.fill: parent
        camera: root._attitudeView
                ? attitudeCamera
                : (root.cameraMode === root.cameraFollow && root.followCameraEnabled
                   ? followCamera
                   : overviewCamera)

        environment: SceneEnvironment {
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.Medium
            // Transparent so the route floats on the panel's own deep-navy
            // background instead of the stock white clear colour, which clashed
            // with the surrounding Falcon theme. The panel Rectangle behind this
            // View3D supplies the colour, border and rounding.
            backgroundMode: SceneEnvironment.Transparent
        }

        OrthographicCamera {
            id: overviewCamera

            readonly property real fitMagnification: Math.max(0.01,
                                                              Math.min(
                                                                  (routeView3D.width
                                                                   * root._cameraFillRatio)
                                                                  / root._projectedRouteWidth,
                                                                  (routeView3D.height
                                                                   * root._cameraFillRatio)
                                                                  / root._projectedRouteHeight))
            readonly property real cameraDistance: Math.max(60, root._sceneExtent * 1.8)

            position: Qt.vector3d(root._sceneCenter.x
                                  + (cameraDistance * root._cameraOffsetX),
                                  root._sceneCenter.y
                                  + (cameraDistance * root._cameraOffsetY),
                                  root._sceneCenter.z
                                  + (cameraDistance * root._cameraOffsetZ))
            rotation: Quaternion.lookAt(position, root._sceneCenter)
            horizontalMagnification: fitMagnification
            verticalMagnification: fitMagnification
            clipNear: 1
            clipFar: cameraDistance * 5
        }

        Node {
            id: followCameraTarget

            position: root._desiredFollowTarget

            Behavior on position {
                Vector3dAnimation {
                    duration: 250
                    easing.type: Easing.OutQuad
                }
            }
        }

        PerspectiveCamera {
            id: followCamera

            position: root._desiredFollowCameraPosition
            rotation: root._stableFollowCameraRotation(position,
                                                       followCameraTarget.position)
            fieldOfView: 55
            clipNear: 0.5
            clipFar: Math.max(500, root._sceneExtent * 10)

            Behavior on position {
                Vector3dAnimation {
                    duration: 250
                    easing.type: Easing.OutQuad
                }
            }
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-45, -35, 0)
            brightness: 0.65
            castsShadow: true
            shadowFactor: 20
            shadowMapQuality: Light.ShadowMapQualityMedium
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(35, 145, 0)
            brightness: 0.18
        }

        Model {
            position: Qt.vector3d(root._sceneCenter.x,
                                  root._groundY,
                                  root._sceneCenter.z)
            eulerRotation.x: -90
            source: "#Rectangle"
            scale: Qt.vector3d(root._groundWidth / 100,
                               root._groundDepth / 100,
                               1)
            opacity: 0.72

            materials: DefaultMaterial {
                diffuseColor: "#262626"
                specularRoughness: 1
            }
        }

        // Waypoints, rendered as holographic AR beacons rather than a solid
        // route line: a ground pad, a light beam up to the commanded altitude and
        // a glowing core. Every material is unlit and alpha-blended so the marks
        // read as a projected overlay on the transparent scene instead of solid
        // geometry, and so they stay legible against both the dark panel and the
        // ground plane. The connecting route line is deliberately gone -- the
        // waypoints themselves are the information.
        Repeater3D {
            model: root._waypointMarkers

            delegate: Node {
                id: waypointBeacon

                required property var modelData

                readonly property color beaconColor: root._waypointColor(
                                                         modelData.markerIndex,
                                                         modelData.physicalWaypointIndex)
                readonly property bool isActive: root._waypointState(
                                                     modelData.markerIndex,
                                                     modelData.physicalWaypointIndex) === root.waypointActive
                // Height of the beam: from the ground plane up to the waypoint.
                readonly property real beamHeight: Math.max(0.1, modelData.position.y - root._groundY)
                readonly property real markerScale: root._markerRadius
                                                    * root._waypointSizeStyleScale
                                                    * root.waypointSizeScale

                position: modelData.position

                // Pulsing halo marks the waypoint the vehicle is flying to.
                SequentialAnimation on scale {
                    running: waypointBeacon.isActive
                    loops: Animation.Infinite
                    NumberAnimation { to: Qt.vector3d(1.12, 1.12, 1.12); duration: 700; easing.type: Easing.InOutQuad }
                    NumberAnimation { to: Qt.vector3d(1.0, 1.0, 1.0);    duration: 700; easing.type: Easing.InOutQuad }
                }

                // --- glowing core ---
                Model {
                    source: "#Sphere"
                    scale: Qt.vector3d(waypointBeacon.markerScale / 100,
                                       waypointBeacon.markerScale / 100,
                                       waypointBeacon.markerScale / 100)
                    materials: PrincipledMaterial {
                        baseColor: waypointBeacon.beaconColor
                        lighting: PrincipledMaterial.NoLighting
                        alphaMode: PrincipledMaterial.Blend
                        opacity: 0.95
                    }
                }

                // --- halo shell around the core ---
                Model {
                    source: "#Sphere"
                    scale: Qt.vector3d(waypointBeacon.markerScale / 45,
                                       waypointBeacon.markerScale / 45,
                                       waypointBeacon.markerScale / 45)
                    materials: PrincipledMaterial {
                        baseColor: waypointBeacon.beaconColor
                        lighting: PrincipledMaterial.NoLighting
                        alphaMode: PrincipledMaterial.Blend
                        opacity: waypointBeacon.isActive ? 0.30 : 0.16
                    }
                }

                // --- vertical beam down to the ground plane ---
                Model {
                    source: "#Cylinder"
                    position: Qt.vector3d(0, -waypointBeacon.beamHeight / 2, 0)
                    scale: Qt.vector3d(waypointBeacon.markerScale / 260,
                                       waypointBeacon.beamHeight / 100,
                                       waypointBeacon.markerScale / 260)
                    materials: PrincipledMaterial {
                        baseColor: waypointBeacon.beaconColor
                        lighting: PrincipledMaterial.NoLighting
                        alphaMode: PrincipledMaterial.Blend
                        opacity: waypointBeacon.isActive ? 0.55 : 0.28
                    }
                }

                // --- ground contact pad ---
                Model {
                    source: "#Cylinder"
                    position: Qt.vector3d(0, -waypointBeacon.beamHeight, 0)
                    scale: Qt.vector3d(waypointBeacon.markerScale / 28,
                                       0.004,
                                       waypointBeacon.markerScale / 28)
                    materials: PrincipledMaterial {
                        baseColor: waypointBeacon.beaconColor
                        lighting: PrincipledMaterial.NoLighting
                        alphaMode: PrincipledMaterial.Blend
                        opacity: waypointBeacon.isActive ? 0.42 : 0.20
                    }
                }
            }
        }

        // Rig carrying only the aircraft position and heading. Cameras parented
        // here look at the airframe from a body-relative direction, so "side" is
        // always the aircraft's side rather than a compass direction. Cameras
        // face their local -Z, hence the rotations below.
        Node {
            id: attitudeRig

            position: root.aircraftPosition
            // Side and rear are body-relative, so they carry the aircraft's
            // heading. Top deliberately does not: it stays world-aligned
            // (north up, since north is -Z and the down-looking camera puts -Z
            // at the top of the screen) so the airframe visibly points at its
            // compass heading instead of always facing up the screen.
            eulerRotation.y: root.cameraMode === root.cameraTop ? 0 : root.aircraftHeading

            PerspectiveCamera {
                id: attitudeCamera

                clipNear: 0.5
                clipFar: Math.max(400, root._sceneExtent * 6)
                fieldOfView: 40

                position: {
                    switch (root.cameraMode) {
                    case root.cameraTop:
                        return Qt.vector3d(0, root.attitudeTopViewDistance, 0)
                    case root.cameraSide:
                        return Qt.vector3d(root.attitudeViewDistance, root.attitudeViewHeight, 0)
                    default:    // rear
                        return Qt.vector3d(0, root.attitudeViewHeight, -root.attitudeViewDistance)
                    }
                }

                eulerRotation: {
                    switch (root.cameraMode) {
                    case root.cameraTop:
                        return Qt.vector3d(-90, 0, 0)   // straight down, nose up-screen
                    case root.cameraSide:
                        return Qt.vector3d(0, 90, 0)    // look -X, from the right wing
                    default:
                        return Qt.vector3d(0, 180, 0)   // look +Z, from behind the tail
                    }
                }

                Behavior on position {
                    Vector3dAnimation { duration: 260; easing.type: Easing.OutQuad }
                }
            }
        }

        Node {
            id: aircraftRoot

            visible: root.routeDataValid
            position: root.aircraftPosition
            // Body attitude from telemetry. Parent local axes are the body axes
            // (X right, Y up, Z forward), so pitch maps to X and roll to Z. Signs
            // are negated to match aircraft convention (nose up / right wing down).
            eulerRotation.x: -root.aircraftPitch
            eulerRotation.y: root.aircraftHeading
            eulerRotation.z: -root.aircraftRoll
            scale: Qt.vector3d(root.aircraftModelScale,
                               root.aircraftModelScale,
                               root.aircraftModelScale)

            Behavior on position {
                Vector3dAnimation {
                    duration: 220
                    easing.type: Easing.OutQuad
                }
            }

            // Lift rotors turn in hover/transition, the pusher turns in forward
            // flight; both idle when disarmed. Driven from the VTOL state rather
            // than from actuator feedback so it still animates without MAVROS.
            readonly property bool _liftPropsTurning: root.activeVehicle
                                                      && root.activeVehicle.armed
                                                      && !root.activeVehicle.vtolInFwdFlight
            readonly property bool _pusherTurning: root.activeVehicle
                                                   && root.activeVehicle.armed
                                                   && root.activeVehicle.vtolInFwdFlight

            FalconAircraft {
                id: falconAircraft

                eulerRotation: root.aircraftModelRotationOffset

                Component.onCompleted: {
                    root._tintAircraft(falconAircraft)
                    console.log("[MissionRoute3D] FalconAircraft component ready:",
                                falconAircraft.aircraftBodyNode.objectName,
                                falconAircraft.liftPropFlNode.objectName,
                                falconAircraft.liftPropFrNode.objectName,
                                falconAircraft.liftPropRlNode.objectName,
                                falconAircraft.liftPropRrNode.objectName,
                                falconAircraft.pusherPropNode.objectName)
                }
            }

            // Lift rotors turn about their own vertical axis; the pusher turns
            // about the model's native forward axis (+X, which the -90 deg yaw
            // offset on the parent aligns with the flight direction).
            NumberAnimation {
                target: falconAircraft.liftPropFlNode
                property: "eulerRotation.y"
                from: 0; to: 360
                duration: 90
                loops: Animation.Infinite
                running: aircraftRoot._liftPropsTurning
            }

            NumberAnimation {
                target: falconAircraft.liftPropFrNode
                property: "eulerRotation.y"
                from: 360; to: 0            // counter-rotating pair
                duration: 90
                loops: Animation.Infinite
                running: aircraftRoot._liftPropsTurning
            }

            NumberAnimation {
                target: falconAircraft.liftPropRlNode
                property: "eulerRotation.y"
                from: 360; to: 0
                duration: 90
                loops: Animation.Infinite
                running: aircraftRoot._liftPropsTurning
            }

            NumberAnimation {
                target: falconAircraft.liftPropRrNode
                property: "eulerRotation.y"
                from: 0; to: 360
                duration: 90
                loops: Animation.Infinite
                running: aircraftRoot._liftPropsTurning
            }

            NumberAnimation {
                target: falconAircraft.pusherPropNode
                property: "eulerRotation.x"
                from: 0; to: 360
                duration: 70
                loops: Animation.Infinite
                running: aircraftRoot._pusherTurning
            }
        }
    }

    // AR-style waypoint tags projected from the 3D beacons onto the 2D surface.
    // Thin outline over a translucent fill rather than the previous opaque white
    // chips, so they read as a heads-up overlay and do not mask the scene.
    Repeater {
        model: root._waypointMarkers

        delegate: Item {
            id: waypointTag

            required property var modelData

            readonly property color tagColor: root._waypointColor(
                                                  modelData.markerIndex,
                                                  modelData.physicalWaypointIndex)
            readonly property string tagText: root._physicalWaypointLabel(
                                                  modelData.physicalWaypointIndex)

            readonly property vector3d screenPosition: {
                const activeCamera = routeView3D.camera
                if (!activeCamera || routeView3D.width <= 0 || routeView3D.height <= 0) {
                    return Qt.vector3d(0, 0, -1)
                }
                if (!isFinite(activeCamera.position.x) || !isFinite(activeCamera.rotation.scalar)) {
                    return Qt.vector3d(0, 0, -1)
                }
                return routeView3D.mapFrom3DScene(modelData.position)
            }

            visible: root.routeDataValid
                     && tagText.length > 0
                     && screenPosition.z > 0
                     && screenPosition.x >= 0
                     && screenPosition.x <= routeView3D.width
                     && screenPosition.y >= 0
                     && screenPosition.y <= routeView3D.height

            x: screenPosition.x - (width / 2)
            y: screenPosition.y - height - 8
            width: tagLabel.implicitWidth + 10
            height: tagLabel.implicitHeight + 5
            z: 9

            Rectangle {
                anchors.fill: parent
                radius: 2
                color: Qt.rgba(0, 0, 0, 0.70)
                border.width: 1
                border.color: Qt.rgba(waypointTag.tagColor.r,
                                      waypointTag.tagColor.g,
                                      waypointTag.tagColor.b,
                                      0.85)
            }

            // Short tick joining the tag to its beacon, as on a HUD callout.
            Rectangle {
                anchors.top: parent.bottom
                anchors.horizontalCenter: parent.horizontalCenter
                width: 1
                height: 6
                color: Qt.rgba(waypointTag.tagColor.r,
                               waypointTag.tagColor.g,
                               waypointTag.tagColor.b,
                               0.7)
            }

            Text {
                id: tagLabel
                anchors.centerIn: parent
                text: waypointTag.tagText
                color: waypointTag.tagColor
                font.bold: true
                font.pixelSize: 10
                font.letterSpacing: 0.5
            }
        }
    }

    // --- flight state, relocated from the removed bottom toolbar --------------
    // MR/FW is the VTOL transition state, the one piece of the stock strip that
    // has no equivalent in the other custom panels.
    Row {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 6
        spacing: 4
        z: 10

        Rectangle {
            visible: root.activeVehicle && root.activeVehicle.vtol
            width: vtolLabel.implicitWidth + 12
            height: 20
            color: root.activeVehicle && root.activeVehicle.vtolInFwdFlight
                       ? Qt.rgba(0.26, 0.75, 0.40, 0.85)
                       : Qt.rgba(0.51, 0.81, 1.0, 0.90)

            Text {
                id: vtolLabel
                anchors.centerIn: parent
                text: root.activeVehicle && root.activeVehicle.vtolInFwdFlight
                          ? qsTr("FW") : qsTr("MR")
                color: "#000000"
                font.bold: true
                font.pixelSize: 10
            }
        }

        Rectangle {
            visible: root.activeVehicle
            width: armLabel.implicitWidth + 12
            height: 20
            color: root.activeVehicle && root.activeVehicle.armed
                       ? Qt.rgba(0.95, 0.76, 0.10, 0.85)
                       : Qt.rgba(0.15, 0.15, 0.15, 0.92)

            Text {
                id: armLabel
                anchors.centerIn: parent
                text: root.activeVehicle && root.activeVehicle.armed
                          ? qsTr("ARMED") : qsTr("DISARMED")
                color: root.activeVehicle && root.activeVehicle.armed ? "#000000" : "#C6C6C6"
                font.bold: true
                font.pixelSize: 10
            }
        }

        Rectangle {
            visible: root.activeVehicle
            width: modeLabel.implicitWidth + 12
            height: 20
            color: Qt.rgba(0.15, 0.15, 0.15, 0.92)

            Text {
                id: modeLabel
                anchors.centerIn: parent
                text: root.activeVehicle ? root.activeVehicle.flightMode : ""
                color: "#F4F4F4"
                font.pixelSize: 10
            }
        }
    }

    // --- camera / attitude view selection -------------------------------------
    Column {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 6
        spacing: 3
        z: 10

        Row {
            anchors.right: parent.right
            spacing: 3

            Repeater {
                model: [
                    { label: qsTr("TOP"),  mode: root.cameraTop },
                    { label: qsTr("SIDE"), mode: root.cameraSide },
                    { label: qsTr("REAR"), mode: root.cameraRear }
                ]

                delegate: Rectangle {
                    id: attitudeViewButton

                    required property var modelData

                    readonly property bool selected: !root.automaticCameraMode
                                                     && root.cameraMode === modelData.mode

                    width: attitudeViewLabel.implicitWidth + 12
                    height: 20
                    color: selected
                           ? Qt.rgba(0.51, 0.81, 1.0, 0.90)
                           : Qt.rgba(0.15, 0.15, 0.15, 0.92)
                    border.width: 1
                    border.color: "#393939"

                    Text {
                        id: attitudeViewLabel

                        anchors.centerIn: parent
                        text: attitudeViewButton.modelData.label
                        color: attitudeViewButton.selected ? "#000000" : "#C6C6C6"
                        font.bold: attitudeViewButton.selected
                        font.pixelSize: 9
                    }

                    MouseArea {
                        anchors.fill: parent
                        cursorShape: Qt.PointingHandCursor
                        onClicked: {
                            root.automaticCameraMode = false
                            root.cameraMode = attitudeViewButton.modelData.mode
                        }
                    }
                }
            }
        }

    Row {
        anchors.right: parent.right
        spacing: 3

        Repeater {
            model: [qsTr("OVERVIEW"), qsTr("FOLLOW"), qsTr("AUTO")]

            delegate: Rectangle {
                id: cameraModeButton

                required property int index
                required property var modelData

                readonly property bool selected: index === 2
                                                 ? root.automaticCameraMode
                                                 : !root.automaticCameraMode
                                                   && root.cameraMode === index

                width: cameraModeLabel.implicitWidth + 12
                height: 20
                radius: 3
                color: selected
                       ? Qt.rgba(0.51, 0.81, 1.0, 0.90)
                       : Qt.rgba(0.15, 0.15, 0.15, 0.92)
                border.width: 1
                border.color: "#393939"
                opacity: index !== 1 || root.followCameraEnabled ? 1 : 0.45

                Text {
                    id: cameraModeLabel

                    anchors.centerIn: parent
                    text: cameraModeButton.modelData
                    color: cameraModeButton.selected ? "#000000" : "#C6C6C6"
                    font.bold: cameraModeButton.selected
                    font.pixelSize: 9
                }

                MouseArea {
                    anchors.fill: parent
                    enabled: cameraModeButton.index !== 1 || root.followCameraEnabled

                    onClicked: {
                        if (cameraModeButton.index === 2) {
                            root.automaticCameraMode = true
                            root._updateAutomaticCameraMode()
                        } else {
                            root.automaticCameraMode = false
                            root.cameraMode = cameraModeButton.index
                        }
                    }
                }
            }
        }
        }
    }

    Repeater {
        model: root.missionController ? root.missionController.visualItems : null

        delegate: Item {
            id: visualItemWatcher

            required property var object

            visible: false
            width: 0
            height: 0

            Connections {
                target: visualItemWatcher.object
                ignoreUnknownSignals: true

                function onCoordinateChanged() {
                    root.rebuildScene()
                }

                function onCommandChanged() {
                    root.rebuildScene()
                }

                function onSequenceNumberChanged() {
                    root.rebuildScene()
                }

                function onIsTakeoffItemChanged() {
                    root.rebuildScene()
                }

                function onIsLandCommandChanged() {
                    root.rebuildScene()
                }

                function onAmslEntryAltChanged() {
                    root.rebuildScene()
                }

                function onAmslExitAltChanged() {
                    root.rebuildScene()
                }

                function onSpecifiesCoordinateChanged() {
                    root.rebuildScene()
                }

                function onIsStandaloneCoordinateChanged() {
                    root.rebuildScene()
                }
            }

            Connections {
                target: visualItemWatcher.object && visualItemWatcher.object.isSimpleItem
                        ? visualItemWatcher.object.altitude
                        : null
                ignoreUnknownSignals: true

                function onRawValueChanged() {
                    root.rebuildScene()
                }
            }
        }
    }

    Repeater {
        model: root.missionController
               ? root.missionController.simpleFlightPathSegments
               : null

        delegate: Item {
            id: segmentWatcher

            required property var object

            visible: false
            width: 0
            height: 0

            Connections {
                target: segmentWatcher.object
                ignoreUnknownSignals: true

                function onCoordinate1Changed() {
                    root.rebuildScene()
                }

                function onCoordinate2Changed() {
                    root.rebuildScene()
                }

                function onCoord1AMSLAltChanged() {
                    root.rebuildScene()
                }

                function onCoord2AMSLAltChanged() {
                    root.rebuildScene()
                }
            }
        }
    }

    Connections {
        target: root.activeVehicle
        ignoreUnknownSignals: true

        function onCoordinateChanged() {
            root._updateVehiclePosition()
        }

        function onArmedChanged() {
            root._updateAutomaticCameraMode()
        }

        function onFlyingChanged() {
            root._updateAutomaticCameraMode()
        }

        function onLandingChanged() {
            root._updateAutomaticCameraMode()
        }
    }

    Connections {
        target: root.activeVehicle ? root.activeVehicle.altitudeAMSL : null
        ignoreUnknownSignals: true

        function onRawValueChanged() {
            root._updateVehiclePosition()
        }
    }

    Connections {
        target: root.activeVehicle ? root.activeVehicle.heading : null
        ignoreUnknownSignals: true

        function onRawValueChanged() {
            root._updateFollowCameraPose()
        }
    }

    Connections {
        target: root.missionController
        ignoreUnknownSignals: true

        function onVisualItemsReset() {
            root.rebuildScene()
        }

        function onNewItemsFromVehicle() {
            root.rebuildScene()
        }
    }

    Connections {
        target: root.missionController ? root.missionController.visualItems : null
        ignoreUnknownSignals: true

        function onCountChanged() {
            root.rebuildScene()
        }

        function onModelReset() {
            root.rebuildScene()
        }
    }

    Connections {
        target: root.missionController
                ? root.missionController.simpleFlightPathSegments
                : null
        ignoreUnknownSignals: true

        function onCountChanged() {
            root.rebuildScene()
        }

        function onModelReset() {
            root.rebuildScene()
        }
    }

    onMissionControllerChanged: {
        rebuildScene()
        Qt.callLater(function() {
            root.logDebugState("missionController changed")
        })
    }
    onMissionAvailableChanged: {
        rebuildScene()
        Qt.callLater(function() {
            root.logDebugState("missionAvailable changed")
        })
    }
    onActiveVehicleChanged: {
        _lastVehiclePositionValid = false
        _updateVehiclePosition()
    }
    onCurrentLegIndexChanged: _updateFollowCameraPose()
    onOverviewPitchDegreesChanged: rebuildScene()
    onFollowPitchDegreesChanged: _updateFollowCameraPose()
    onFollowDistanceChanged: _updateFollowCameraPose()
    onFollowHeightOffsetChanged: _updateFollowCameraPose()
    onFollowLookAheadDistanceChanged: _updateFollowCameraPose()
    onAutomaticCameraModeChanged: _updateAutomaticCameraMode()
    onFollowCameraEnabledChanged: _updateAutomaticCameraMode()
    onAltitudeVisualScaleChanged: rebuildScene()
    onMissionProgressRevisionChanged: rebuildScene()
    onRouteDataValidChanged: {
        Qt.callLater(function() {
            root.logDebugState("routeDataValid changed")
        })
    }

    Component.onCompleted: {
        rebuildScene()
        logDebugState("Component.onCompleted")
    }
}
