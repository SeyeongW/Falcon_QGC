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
        const zMeters = (coordinate.latitude - referenceCoordinate.latitude)
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

    function _currentForwardDirection() {
        if (activeVehicle && activeVehicle.heading) {
            const headingDegrees = Number(activeVehicle.heading.rawValue)
            if (isFinite(headingDegrees)
                    && headingDegrees >= 0
                    && headingDegrees < 360) {
                const headingRadians = headingDegrees * Math.PI / 180
                return Qt.vector3d(Math.sin(headingRadians),
                                   0,
                                   Math.cos(headingRadians))
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
        camera: root.cameraMode === root.cameraFollow && root.followCameraEnabled
                ? followCamera
                : overviewCamera

        environment: SceneEnvironment {
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.Medium
            backgroundMode: SceneEnvironment.Color
            clearColor: "#FFFFFF"
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
                diffuseColor: "#EEF1F4"
                specularRoughness: 1
            }
        }

        Repeater3D {
            model: root._routeSegments

            delegate: Node {
                id: routeSegment

                required property int index
                required property var modelData

                position: modelData.midpoint
                rotation: Quaternion.lookAt(position, modelData.renderPoint2)

                Model {
                    eulerRotation.x: 90
                    source: "#Cylinder"
                    scale: Qt.vector3d((root._segmentRadius
                                        * root._routeThicknessStyleScale
                                        * root.routeThicknessScale) / 50,
                                       routeSegment.modelData.length / 100,
                                       (root._segmentRadius
                                        * root._routeThicknessStyleScale
                                        * root.routeThicknessScale) / 50)

                    materials: DefaultMaterial {
                        diffuseColor: root._segmentColor(
                                          routeSegment.index,
                                          routeSegment.modelData.destinationMarkerIndex,
                                          routeSegment.modelData.traversalIndex)
                        specularRoughness: 0.75
                    }
                }
            }
        }

        Repeater3D {
            model: root._waypointMarkers

            delegate: Model {
                id: waypointMarker

                required property var modelData

                position: modelData.position
                source: "#Sphere"
                scale: Qt.vector3d((root._markerRadius
                                    * root._waypointSizeStyleScale
                                    * root.waypointSizeScale * 2) / 100,
                                   (root._markerRadius
                                    * root._waypointSizeStyleScale
                                    * root.waypointSizeScale * 2) / 100,
                                   (root._markerRadius
                                    * root._waypointSizeStyleScale
                                    * root.waypointSizeScale * 2) / 100)

                materials: PrincipledMaterial {
                    baseColor: root._waypointColor(
                                   waypointMarker.modelData.markerIndex,
                                   waypointMarker.modelData.physicalWaypointIndex)
                    roughness: 0.65
                    metalness: 0.05
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

            FalconAircraft {
                id: falconAircraft

                eulerRotation: root.aircraftModelRotationOffset

                Component.onCompleted: {
                    console.log("[MissionRoute3D] FalconAircraft component ready:",
                                falconAircraft.aircraftBodyNode.objectName,
                                falconAircraft.liftPropFlNode.objectName,
                                falconAircraft.liftPropFrNode.objectName,
                                falconAircraft.liftPropRlNode.objectName,
                                falconAircraft.liftPropRrNode.objectName,
                                falconAircraft.pusherPropNode.objectName)
                }
            }
        }
    }

    Repeater {
        model: root._waypointMarkers

        delegate: Item {
            id: waypointSequenceLabel

            required property var modelData

            readonly property vector3d screenPosition: {
                const activeCamera = routeView3D.camera
                if (!activeCamera
                        || routeView3D.width <= 0
                        || routeView3D.height <= 0) {
                    return Qt.vector3d(0, 0, -1)
                }

                const cameraPosition = activeCamera.position
                const cameraRotation = activeCamera.rotation
                if (!isFinite(cameraPosition.x)
                        || !isFinite(cameraRotation.scalar)) {
                    return Qt.vector3d(0, 0, -1)
                }

                return routeView3D.mapFrom3DScene(modelData.position)
            }

            visible: root.routeDataValid
                     && root._physicalWaypointLabel(
                         modelData.physicalWaypointIndex).length > 0
                     && screenPosition.z > 0
                     && screenPosition.x >= 0
                     && screenPosition.x <= routeView3D.width
                     && screenPosition.y >= 0
                     && screenPosition.y <= routeView3D.height
            x: screenPosition.x - (width / 2)
            y: screenPosition.y - height - 3
            width: waypointLabelColumn.implicitWidth + 6
            height: waypointLabelColumn.implicitHeight + 4
            z: 9

            Rectangle {
                anchors.fill: parent
                radius: 3
                color: Qt.rgba(1, 1, 1, 0.82)
                border.width: 1
                border.color: Qt.rgba(0.31, 0.35, 0.41, 0.45)
            }

            Column {
                id: waypointLabelColumn

                anchors.centerIn: parent
                spacing: 0

                Text {
                    text: root._physicalWaypointLabel(
                              waypointSequenceLabel.modelData.physicalWaypointIndex)
                    color: "#1F2937"
                    font.bold: true
                    font.pixelSize: 10
                    horizontalAlignment: Text.AlignHCenter
                }

                Text {
                    visible: root.showDebugSequenceNumbers
                             && text.length > 0
                    text: root._physicalWaypointDebugLabel(
                              waypointSequenceLabel.modelData.physicalWaypointIndex)
                    color: "#64748B"
                    font.pixelSize: 7
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }
    }

    Row {
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 6
        spacing: 3
        z: 10

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
                       ? Qt.rgba(0.22, 0.74, 0.97, 0.82)
                       : Qt.rgba(0.03, 0.08, 0.14, 0.78)
                border.width: 1
                border.color: Qt.rgba(0.55, 0.70, 0.78, 0.55)
                opacity: index !== 1 || root.followCameraEnabled ? 1 : 0.45

                Text {
                    id: cameraModeLabel

                    anchors.centerIn: parent
                    text: cameraModeButton.modelData
                    color: cameraModeButton.selected ? "#071526" : "#D6E2E8"
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
