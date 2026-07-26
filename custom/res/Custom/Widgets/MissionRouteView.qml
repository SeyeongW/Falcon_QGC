pragma ComponentBehavior: Bound

import QtQuick

import QGroundControl.Controls

Item {
    id: root

    property var missionController
    property var activeVehicle
    property bool missionAvailable: false
    property bool showDebugSequenceNumbers: false

    property int  activeWaypointIndex: -1
    property int  currentLegIndex:     -1
    property real currentLegProgress:  0.0
    property var  waypointStates:      []
    property bool routeDataValid:      false
    property var  missionOccurrences:       []
    property var  physicalWaypointGroups:   []
    property var  traversalOccurrences:     []
    property int  activeOccurrenceIndex:    -1
    property int  activeTraversalIndex:     -1
    property int  missionProgressRevision:  0

    property string routeText:              "-- >>> --"
    property string routeDetailText:        ""
    property real   horizontalError:        Number.NaN
    property real   verticalError:          Number.NaN
    property real   altitudeDrop:           Number.NaN
    property real   maximumAccelerationG:   Number.NaN
    property string missionTimeText:        "--"
    property int    estimatedScore:         -1

    readonly property real routeAreaRatio: 0.70
    readonly property real routeAreaWidth: routeArea.width

    readonly property int waypointPending: 0
    readonly property int waypointActive:  1
    readonly property int waypointPassed:  2
    readonly property int waypointFailed:  3

    readonly property color _baseRouteColor:      "#374151"
    readonly property color _standbyColor:       "#64748B"
    readonly property color _standbyBorderColor: "#CBD5E1"
    readonly property color _activeColor:        "#D6B84C"
    readonly property color _passedColor:        "#4E9F75"
    readonly property color _failedColor:        "#C75C5C"
    readonly property color _takeoffColor:       "#6F9BA8"
    readonly property color _landingColor:       "#91889C"
    readonly property color _vertiportColor:     "#183C48"
    readonly property real  _markerDiameter:     Math.max(14, Math.min(18, Math.min(routeArea.width,
                                                                                   routeArea.height) * 0.05))
    readonly property real  _lineWidth:          Math.max(1, Math.min(routeArea.width,
                                                                     routeArea.height) * 0.005)
    readonly property real  _verticalLineWidth:  Math.max(_lineWidth + 0.5, Math.min(routeArea.width,
                                                                                    routeArea.height) * 0.007)
    readonly property real  _labelPixelSize:     Math.max(8, Math.min(10, Math.min(routeArea.width,
                                                                                  routeArea.height) * 0.032))
    readonly property real  _coordinateTolerance: 0.0000001
    readonly property real  _rangeEpsilon:         0.000000000001
    readonly property int   _mavCmdNavLand:        21
    readonly property int   _mavCmdNavTakeoff:     22
    readonly property int   _mavCmdNavVtolTakeoff: 84
    readonly property int   _mavCmdNavVtolLand:    85

    property bool  hasMissionData: false
    property point startPoint:     Qt.point(routeArea.width / 2, routeArea.height / 2)
    property point aircraftPoint:  startPoint
    property var   _routeSegments: []
    property var   _routeMarkers:  []
    property var   _takeoffVisual: null
    property var   _landingVisual: null
    property var   _homePadVisual: null
    property var   _targetMissionItem: null
    property int   _targetSequenceNumber: -1
    property real  _missionStartTimestampMs: Number.NaN
    property real  _missionStopTimestampMs: Number.NaN
    property bool  _missionTimerRunning: false
    property int   _lastMissionSequence: -1
    property string _homePhysicalWaypointKey: ""

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

    function _findCoordinateIndex(coordinates, coordinate) {
        for (let index = 0; index < coordinates.length; index++) {
            if (_coordinatesMatch(coordinates[index], coordinate)) {
                return index
            }
        }
        return -1
    }

    function _groupForCoordinate(coordinate) {
        const coordinateKey = _physicalWaypointKey(coordinate)
        for (let groupIndex = 0;
             groupIndex < physicalWaypointGroups.length;
             groupIndex++) {
            const group = physicalWaypointGroups[groupIndex]
            if (group.coordinateKeys
                    && group.coordinateKeys.indexOf(coordinateKey) >= 0) {
                return group
            }
        }
        return null
    }

    function _labelForCoordinate(coordinate, homeCoordinate) {
        if (homeCoordinate && _coordinatesMatch(coordinate, homeCoordinate)) {
            return qsTr("HOME")
        }

        const group = _groupForCoordinate(coordinate)
        return group ? group.label : ""
    }

    function _debugLabelForCoordinate(coordinate) {
        const group = _groupForCoordinate(coordinate)
        return group && group.sequenceNumbers.length > 0
                ? qsTr("SEQ %1").arg(group.sequenceNumbers.join("·"))
                : ""
    }

    function _waypointState(markerIndex) {
        if (markerIndex === activeWaypointIndex) {
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

    function _waypointColor(markerIndex) {
        switch (_waypointState(markerIndex)) {
        case waypointActive:
            return _activeColor
        case waypointPassed:
            return _passedColor
        case waypointFailed:
            return _failedColor
        default:
            return _standbyColor
        }
    }

    function _segmentState(segment, segmentIndex) {
        const destinationState = _waypointState(segment.destinationMarkerIndex)
        if (destinationState === waypointFailed) {
            return waypointFailed
        }
        if (segmentIndex === currentLegIndex) {
            return waypointActive
        }
        if (destinationState === waypointPassed) {
            return waypointPassed
        }
        return waypointPending
    }

    function _segmentColor(segment, segmentIndex) {
        switch (_segmentState(segment, segmentIndex)) {
        case waypointActive:
            return _activeColor
        case waypointPassed:
            return _passedColor
        case waypointFailed:
            return _failedColor
        default:
            return _baseRouteColor
        }
    }

    function _isTakeoffCommand(command) {
        return command === _mavCmdNavTakeoff || command === _mavCmdNavVtolTakeoff
    }

    function _isLandingCommand(command) {
        return command === _mavCmdNavLand || command === _mavCmdNavVtolLand
    }

    function _verticalVisualHeight(item) {
        let altitude = Number.NaN
        if (item && item.isSimpleItem && item.altitude) {
            altitude = Number(item.altitude.rawValue)
        }

        if (!isFinite(altitude) || Math.abs(altitude) < 0.01) {
            return 46
        }

        // This is a screen-only 2.5D offset. Mission coordinates are not changed.
        const altitudeBasedHeight = 40 + (Math.sqrt(Math.abs(altitude)) * 1.5)
        return Math.max(40, Math.min(55, altitudeBasedHeight))
    }

    function _projectCoordinate(coordinate, projection) {
        const localX = (coordinate.longitude - projection.referenceLongitude) * projection.cosLatitude
        const localY = coordinate.latitude - projection.referenceLatitude
        return Qt.point(projection.offsetX + ((localX - projection.minX) * projection.scale),
                        projection.offsetY + ((projection.maxY - localY) * projection.scale))
    }

    function pointForLegProgress(legIndex, legProgress) {
        if (legIndex < 0 || legIndex >= _routeSegments.length) {
            return startPoint
        }

        const segment = _routeSegments[legIndex]
        const numericProgress = Number(legProgress)
        const progress = isFinite(numericProgress)
                ? Math.max(0, Math.min(1, numericProgress))
                : 0
        return Qt.point(segment.point1.x + ((segment.point2.x - segment.point1.x) * progress),
                        segment.point1.y + ((segment.point2.y - segment.point1.y) * progress))
    }

    function updateAircraftPoint() {
        aircraftPoint = currentLegIndex >= 0
                ? pointForLegProgress(currentLegIndex, currentLegProgress)
                : startPoint
    }

    function _infoLabel(rowIndex) {
        const labels = [
            qsTr("ROUTE"),
            qsTr("H ERROR"),
            qsTr("V ERROR"),
            qsTr("ALT DROP"),
            qsTr("MAX ACC"),
            qsTr("TIME"),
            qsTr("SCORE EST.")
        ]
        return labels[rowIndex]
    }

    function _validText(value, fallback) {
        return value && value.length > 0 ? value : fallback
    }

    function _infoValue(rowIndex) {
        switch (rowIndex) {
        case 0:
            return _validText(routeText, "-- >>> --")
        case 1:
            return isFinite(horizontalError) ? Number(horizontalError).toFixed(1) : "--"
        case 2:
            return isFinite(verticalError) ? Number(verticalError).toFixed(1) : "--"
        case 3:
            return isFinite(altitudeDrop) ? Number(altitudeDrop).toFixed(1) : "--"
        case 4:
            return isFinite(maximumAccelerationG) ? Number(maximumAccelerationG).toFixed(2) : "--"
        case 5:
            return _validText(missionTimeText, "--")
        case 6:
            return estimatedScore >= 0 ? String(estimatedScore) : "--"
        default:
            return "--"
        }
    }

    function _infoUnit(rowIndex) {
        switch (rowIndex) {
        case 1:
        case 2:
        case 3:
            return qsTr("m")
        case 4:
            return qsTr("G")
        default:
            return ""
        }
    }

    function _infoDisplayValue(rowIndex) {
        const value = _infoValue(rowIndex)
        const unit = _infoUnit(rowIndex)
        return value === "--" || unit.length === 0 ? value : value + " " + unit
    }

    function _physicalWaypointKey(coordinate) {
        if (!_isValidCoordinate(coordinate)) {
            return ""
        }
        return Number(coordinate.latitude).toFixed(7)
                + ","
                + Number(coordinate.longitude).toFixed(7)
    }

    function _semanticWaypointKey(item) {
        const semanticPropertyNames = [
            "physicalWaypointId",
            "waypointId",
            "semanticName"
        ]
        for (let index = 0; index < semanticPropertyNames.length; index++) {
            let value
            try {
                value = item[semanticPropertyNames[index]]
            } catch (error) {
                value = undefined
            }
            if (value && typeof value === "object" && value.rawValue !== undefined) {
                value = value.rawValue
            }
            if (value === undefined || value === null) {
                continue
            }
            const normalizedValue = String(value).trim()
            if (normalizedValue.length > 0) {
                return "semantic:" + normalizedValue
            }
        }
        return ""
    }

    function _waypointGroupingKey(item) {
        const semanticKey = _semanticWaypointKey(item)
        return semanticKey.length > 0
                ? semanticKey
                : "coordinate:" + _physicalWaypointKey(item.coordinate)
    }

    function _missionHomeCoordinate(visualItems) {
        if (!visualItems) {
            return null
        }

        let takeoffCoordinate = null
        let landingCoordinate = null
        for (let index = 0; index < visualItems.count; index++) {
            const item = visualItems.get(index)
            if (!item || !_isValidCoordinate(item.coordinate)) {
                continue
            }
            if (item.homePosition) {
                return item.coordinate
            }
            if (!takeoffCoordinate && item.isTakeoffItem) {
                takeoffCoordinate = item.coordinate
            }
            if (item.isLandCommand) {
                landingCoordinate = item.coordinate
            }
        }
        return takeoffCoordinate || landingCoordinate
    }

    function _isMissionOccurrenceItem(item) {
        return item
                && !item.homePosition
                && item.specifiesCoordinate
                && !item.isStandaloneCoordinate
                && _isValidCoordinate(item.coordinate)
    }

    function _occurrenceAltitudeAMSL(item) {
        if (!item) {
            return Number.NaN
        }

        const entryAltitude = Number(item.amslEntryAlt)
        if (isFinite(entryAltitude)) {
            return entryAltitude
        }

        const exitAltitude = Number(item.amslExitAlt)
        return isFinite(exitAltitude) ? exitAltitude : Number.NaN
    }

    function _copyOccurrence(occurrence) {
        return {
            occurrenceIndex: occurrence.occurrenceIndex,
            sequenceNumber: occurrence.sequenceNumber,
            lastSequenceNumber: occurrence.lastSequenceNumber,
            coordinate: occurrence.coordinate,
            command: occurrence.command,
            physicalWaypointKey: occurrence.physicalWaypointKey,
            physicalWaypointIndex: occurrence.physicalWaypointIndex,
            isWaypoint: occurrence.isWaypoint,
            isTakeoff: occurrence.isTakeoff,
            isLand: occurrence.isLand,
            visitNumber: occurrence.visitNumber,
            totalVisitCount: occurrence.totalVisitCount,
            state: occurrence.state,
            minimumHorizontalError: occurrence.minimumHorizontalError,
            minimumVerticalError: occurrence.minimumVerticalError,
            passedTimestamp: occurrence.passedTimestamp,
            altitudeAMSL: occurrence.altitudeAMSL,
            missionItem: occurrence.missionItem
        }
    }

    function _copyTraversal(traversal) {
        return {
            legIndex: traversal.legIndex,
            startOccurrenceIndex: traversal.startOccurrenceIndex,
            endOccurrenceIndex: traversal.endOccurrenceIndex,
            startSequence: traversal.startSequence,
            endSequence: traversal.endSequence,
            startCoordinate: traversal.startCoordinate,
            endCoordinate: traversal.endCoordinate,
            startPhysicalWaypointKey: traversal.startPhysicalWaypointKey,
            endPhysicalWaypointKey: traversal.endPhysicalWaypointKey,
            traversalDirection: traversal.traversalDirection,
            physicalEdgeKey: traversal.physicalEdgeKey,
            laneIndex: traversal.laneIndex,
            laneCount: traversal.laneCount,
            laneOffsetFactor: traversal.laneOffsetFactor,
            state: traversal.state
        }
    }

    function rebuildMissionProgressModel() {
        const visualItems = missionController ? missionController.visualItems : null
        const occurrences = []
        const groups = []
        const groupIndexByKey = ({})
        const homeCoordinate = _missionHomeCoordinate(visualItems)
        _homePhysicalWaypointKey = homeCoordinate
                ? _physicalWaypointKey(homeCoordinate)
                : ""
        let nextWaypointNumber = 1

        if (visualItems) {
            for (let modelIndex = 0; modelIndex < visualItems.count; modelIndex++) {
                const item = visualItems.get(modelIndex)
                if (!_isMissionOccurrenceItem(item)) {
                    continue
                }

                const sequenceNumber = Number(item.sequenceNumber)
                if (!isFinite(sequenceNumber)) {
                    continue
                }

                const lastSequenceNumber = Number(item.lastSequenceNumber)
                const physicalKey = _physicalWaypointKey(item.coordinate)
                const groupingKey = _waypointGroupingKey(item)
                let groupIndex = groupIndexByKey[groupingKey]
                if (groupIndex === undefined) {
                    groupIndex = groups.length
                    groupIndexByKey[groupingKey] = groupIndex
                    groups.push({
                        physicalWaypointIndex: groupIndex,
                        physicalWaypointKey: physicalKey,
                        groupingKey: groupingKey,
                        coordinateKeys: [physicalKey],
                        coordinate: item.coordinate,
                        occurrenceIndices: [],
                        waypointOccurrenceIndices: [],
                        sequenceNumbers: [],
                        completedVisitCount: 0,
                        totalVisitCount: 0,
                        hasWaypointOccurrence: false,
                        isDedicated: true,
                        isHome: physicalKey === _homePhysicalWaypointKey,
                        hasTakeoffOccurrence: false,
                        hasLandOccurrence: false,
                        waypointNumber: 0,
                        state: waypointPending,
                        label: ""
                    })
                } else if (groups[groupIndex].coordinateKeys.indexOf(physicalKey) < 0) {
                    groups[groupIndex].coordinateKeys.push(physicalKey)
                }

                const group = groups[groupIndex]
                const isTakeoff = Boolean(item.isTakeoffItem)
                const isLand = Boolean(item.isLandCommand)
                const isWaypoint = !isTakeoff && !isLand && !group.isHome
                if (isWaypoint && !group.hasWaypointOccurrence) {
                    group.hasWaypointOccurrence = true
                    group.isDedicated = false
                    group.waypointNumber = nextWaypointNumber++
                }
                const occurrenceIndex = occurrences.length
                group.occurrenceIndices.push(occurrenceIndex)
                group.hasTakeoffOccurrence = group.hasTakeoffOccurrence || isTakeoff
                group.hasLandOccurrence = group.hasLandOccurrence || isLand
                if (isWaypoint) {
                    group.waypointOccurrenceIndices.push(occurrenceIndex)
                    group.sequenceNumbers.push(Math.floor(sequenceNumber))
                }
                occurrences.push({
                    occurrenceIndex: occurrenceIndex,
                    sequenceNumber: Math.floor(sequenceNumber),
                    lastSequenceNumber: isFinite(lastSequenceNumber)
                            ? Math.max(Math.floor(sequenceNumber),
                                       Math.floor(lastSequenceNumber))
                            : Math.floor(sequenceNumber),
                    coordinate: item.coordinate,
                    command: item.isSimpleItem ? Number(item.command) : -1,
                    physicalWaypointKey: physicalKey,
                    physicalWaypointIndex: groupIndex,
                    isWaypoint: isWaypoint,
                    isTakeoff: isTakeoff,
                    isLand: isLand,
                    visitNumber: isWaypoint
                            ? group.waypointOccurrenceIndices.length
                            : 0,
                    totalVisitCount: 0,
                    state: waypointPending,
                    minimumHorizontalError: Number.NaN,
                    minimumVerticalError: Number.NaN,
                    passedTimestamp: Number.NaN,
                    altitudeAMSL: _occurrenceAltitudeAMSL(item),
                    missionItem: item
                })
            }
        }

        for (let groupIndex = 0; groupIndex < groups.length; groupIndex++) {
            const totalVisitCount = groups[groupIndex].waypointOccurrenceIndices.length
            groups[groupIndex].totalVisitCount = totalVisitCount
            for (let visitIndex = 0; visitIndex < totalVisitCount; visitIndex++) {
                const occurrenceIndex = groups[groupIndex].waypointOccurrenceIndices[visitIndex]
                occurrences[occurrenceIndex].totalVisitCount = totalVisitCount
            }
        }

        const traversals = []
        const traversalIndicesByEdge = ({})
        for (let occurrenceIndex = 1;
             occurrenceIndex < occurrences.length;
             occurrenceIndex++) {
            const startOccurrence = occurrences[occurrenceIndex - 1]
            const endOccurrence = occurrences[occurrenceIndex]
            const firstKey = startOccurrence.physicalWaypointKey
                    < endOccurrence.physicalWaypointKey
                    ? startOccurrence.physicalWaypointKey
                    : endOccurrence.physicalWaypointKey
            const secondKey = firstKey === startOccurrence.physicalWaypointKey
                    ? endOccurrence.physicalWaypointKey
                    : startOccurrence.physicalWaypointKey
            const edgeKey = firstKey + "|" + secondKey
            const traversalIndex = traversals.length
            if (traversalIndicesByEdge[edgeKey] === undefined) {
                traversalIndicesByEdge[edgeKey] = []
            }
            traversalIndicesByEdge[edgeKey].push(traversalIndex)
            traversals.push({
                legIndex: traversalIndex,
                startOccurrenceIndex: startOccurrence.occurrenceIndex,
                endOccurrenceIndex: endOccurrence.occurrenceIndex,
                startSequence: startOccurrence.sequenceNumber,
                endSequence: endOccurrence.sequenceNumber,
                startCoordinate: startOccurrence.coordinate,
                endCoordinate: endOccurrence.coordinate,
                startPhysicalWaypointKey: startOccurrence.physicalWaypointKey,
                endPhysicalWaypointKey: endOccurrence.physicalWaypointKey,
                traversalDirection: startOccurrence.physicalWaypointKey
                        + ">"
                        + endOccurrence.physicalWaypointKey,
                physicalEdgeKey: edgeKey,
                laneIndex: 0,
                laneCount: 1,
                laneOffsetFactor: 0,
                state: waypointPending
            })
        }

        const edgeKeys = Object.keys(traversalIndicesByEdge)
        for (let edgeIndex = 0; edgeIndex < edgeKeys.length; edgeIndex++) {
            const traversalIndices = traversalIndicesByEdge[edgeKeys[edgeIndex]]
            const laneCount = traversalIndices.length
            for (let laneIndex = 0; laneIndex < laneCount; laneIndex++) {
                const traversal = traversals[traversalIndices[laneIndex]]
                traversal.laneIndex = laneIndex
                traversal.laneCount = laneCount
                traversal.laneOffsetFactor = laneIndex - ((laneCount - 1) / 2)
            }
        }

        missionOccurrences = occurrences
        physicalWaypointGroups = groups
        traversalOccurrences = traversals
        activeOccurrenceIndex = -1
        activeTraversalIndex = -1
        _lastMissionSequence = -1
        _applyMissionSequence(_currentMissionSequence())
        missionProgressRevision++
    }

    function _resetOccurrenceProgress() {
        const occurrences = []
        for (let index = 0; index < missionOccurrences.length; index++) {
            const occurrence = _copyOccurrence(missionOccurrences[index])
            occurrence.state = waypointPending
            occurrence.minimumHorizontalError = Number.NaN
            occurrence.minimumVerticalError = Number.NaN
            occurrence.passedTimestamp = Number.NaN
            occurrences.push(occurrence)
        }
        missionOccurrences = occurrences
        _lastMissionSequence = -1
        _publishMissionProgress()
    }

    function _applyMissionSequence(sequenceNumber) {
        if (sequenceNumber < 0 || missionOccurrences.length === 0) {
            activeOccurrenceIndex = -1
            activeTraversalIndex = -1
            _publishMissionProgress()
            return
        }

        if (_lastMissionSequence >= 0 && sequenceNumber < _lastMissionSequence) {
            _resetOccurrenceProgress()
        }

        const now = Date.now()
        let activeTargetIndex = -1
        for (let index = 0; index < missionOccurrences.length; index++) {
            const occurrence = missionOccurrences[index]
            if ((sequenceNumber >= occurrence.sequenceNumber
                 && sequenceNumber <= occurrence.lastSequenceNumber)
                    || sequenceNumber < occurrence.sequenceNumber) {
                activeTargetIndex = index
                break
            }
        }
        const occurrences = []
        for (let index = 0; index < missionOccurrences.length; index++) {
            const occurrence = _copyOccurrence(missionOccurrences[index])
            const wasPassed = occurrence.state === waypointPassed
            if (activeTargetIndex < 0 || index < activeTargetIndex) {
                occurrence.state = waypointPassed
                if (!wasPassed && !isFinite(occurrence.passedTimestamp)) {
                    occurrence.passedTimestamp = now
                }
            } else if (index === activeTargetIndex) {
                occurrence.state = waypointActive
            } else if (occurrence.state !== waypointFailed) {
                occurrence.state = waypointPending
            }
            occurrences.push(occurrence)
        }

        missionOccurrences = occurrences
        _lastMissionSequence = sequenceNumber
        _publishMissionProgress()
    }

    function _completeActiveOccurrence() {
        if (activeOccurrenceIndex < 0
                || activeOccurrenceIndex >= missionOccurrences.length) {
            return
        }

        const occurrences = missionOccurrences.slice()
        const occurrence = _copyOccurrence(occurrences[activeOccurrenceIndex])
        occurrence.state = waypointPassed
        if (!isFinite(occurrence.passedTimestamp)) {
            occurrence.passedTimestamp = Date.now()
        }
        occurrences[activeOccurrenceIndex] = occurrence
        missionOccurrences = occurrences
        _publishMissionProgress()
    }

    function _publishMissionProgress() {
        let activeIndex = -1
        for (let index = 0; index < missionOccurrences.length; index++) {
            if (missionOccurrences[index].state === waypointActive) {
                activeIndex = index
                break
            }
        }
        activeOccurrenceIndex = activeIndex

        const groups = []
        const states = []
        for (let groupIndex = 0;
             groupIndex < physicalWaypointGroups.length;
             groupIndex++) {
            const sourceGroup = physicalWaypointGroups[groupIndex]
            let completedVisitCount = 0
            let groupState = waypointPending
            for (let visitIndex = 0;
                 visitIndex < sourceGroup.occurrenceIndices.length;
                 visitIndex++) {
                const occurrence = missionOccurrences[
                            sourceGroup.occurrenceIndices[visitIndex]]
                if (occurrence.state === waypointPassed) {
                    if (occurrence.isWaypoint) {
                        completedVisitCount++
                    }
                    groupState = waypointPassed
                } else if (occurrence.state === waypointActive) {
                    groupState = waypointActive
                } else if (occurrence.state === waypointFailed
                           && groupState !== waypointActive) {
                    groupState = waypointFailed
                }
            }

            let label = ""
            if (sourceGroup.isHome) {
                label = qsTr("HOME")
            } else if (sourceGroup.waypointNumber > 0) {
                label = qsTr("WP%1").arg(sourceGroup.waypointNumber)
                if (sourceGroup.totalVisitCount > 1) {
                    label += "\n"
                            + completedVisitCount
                            + "/"
                            + sourceGroup.totalVisitCount
                }
            } else if (sourceGroup.hasTakeoffOccurrence) {
                label = qsTr("TO")
            } else if (sourceGroup.hasLandOccurrence) {
                label = qsTr("LAND")
            }
            groups.push({
                physicalWaypointIndex: sourceGroup.physicalWaypointIndex,
                physicalWaypointKey: sourceGroup.physicalWaypointKey,
                groupingKey: sourceGroup.groupingKey,
                coordinateKeys: sourceGroup.coordinateKeys,
                coordinate: sourceGroup.coordinate,
                occurrenceIndices: sourceGroup.occurrenceIndices,
                waypointOccurrenceIndices: sourceGroup.waypointOccurrenceIndices,
                sequenceNumbers: sourceGroup.sequenceNumbers,
                completedVisitCount: completedVisitCount,
                totalVisitCount: sourceGroup.totalVisitCount,
                hasWaypointOccurrence: sourceGroup.hasWaypointOccurrence,
                isDedicated: sourceGroup.isDedicated,
                isHome: sourceGroup.isHome,
                hasTakeoffOccurrence: sourceGroup.hasTakeoffOccurrence,
                hasLandOccurrence: sourceGroup.hasLandOccurrence,
                waypointNumber: sourceGroup.waypointNumber,
                state: groupState,
                label: label
            })
            states.push(groupState)
        }
        physicalWaypointGroups = groups
        waypointStates = states
        activeWaypointIndex = activeIndex >= 0
                ? missionOccurrences[activeIndex].physicalWaypointIndex
                : -1

        let activeLegIndex = -1
        const traversals = []
        for (let index = 0; index < traversalOccurrences.length; index++) {
            const traversal = _copyTraversal(traversalOccurrences[index])
            const destinationOccurrence = missionOccurrences[
                        traversal.endOccurrenceIndex]
            traversal.state = destinationOccurrence
                    ? destinationOccurrence.state
                    : waypointPending
            if (traversal.state === waypointActive) {
                activeLegIndex = index
            }
            traversals.push(traversal)
        }
        traversalOccurrences = traversals
        activeTraversalIndex = activeLegIndex
        _updateRouteText()
    }

    function _routeEndpointLabel(occurrence) {
        if (!occurrence
                || occurrence.physicalWaypointIndex < 0
                || occurrence.physicalWaypointIndex >= physicalWaypointGroups.length) {
            return "--"
        }

        const group = physicalWaypointGroups[occurrence.physicalWaypointIndex]
        if (group.isHome) {
            return qsTr("HOME")
        }
        if (occurrence.isTakeoff) {
            return qsTr("TO")
        }
        if (occurrence.isLand) {
            return qsTr("LAND")
        }
        if (group.waypointNumber > 0) {
            return qsTr("WP%1").arg(group.waypointNumber)
        }
        return "--"
    }

    function _updateRouteText() {
        routeText = "-- >>> --"
        routeDetailText = ""
        if (activeOccurrenceIndex < 0
                || activeOccurrenceIndex >= missionOccurrences.length) {
            return
        }

        const targetOccurrence = missionOccurrences[activeOccurrenceIndex]
        const previousOccurrence = activeOccurrenceIndex > 0
                ? missionOccurrences[activeOccurrenceIndex - 1]
                : null
        const currentLabel = previousOccurrence
                ? _routeEndpointLabel(previousOccurrence)
                : (_homePhysicalWaypointKey.length > 0 ? qsTr("HOME") : "--")
        const targetLabel = _routeEndpointLabel(targetOccurrence)
        routeText = currentLabel + " >>> " + targetLabel

        if (targetOccurrence.isWaypoint
                && targetOccurrence.totalVisitCount > 1) {
            routeDetailText = (targetOccurrence.visitNumber > 1
                               ? qsTr("RETURN · ")
                               : "")
                    + qsTr("VISIT %1/%2")
                        .arg(targetOccurrence.visitNumber)
                        .arg(targetOccurrence.totalVisitCount)
        }
    }

    function _recordActiveOccurrenceErrors() {
        if (activeOccurrenceIndex < 0
                || activeOccurrenceIndex >= missionOccurrences.length) {
            return
        }

        const occurrences = missionOccurrences.slice()
        const occurrence = _copyOccurrence(occurrences[activeOccurrenceIndex])
        let changed = false
        if (isFinite(horizontalError)
                && (!isFinite(occurrence.minimumHorizontalError)
                    || horizontalError < occurrence.minimumHorizontalError)) {
            occurrence.minimumHorizontalError = horizontalError
            changed = true
        }
        if (isFinite(verticalError)
                && (!isFinite(occurrence.minimumVerticalError)
                    || verticalError < occurrence.minimumVerticalError)) {
            occurrence.minimumVerticalError = verticalError
            changed = true
        }
        if (changed) {
            occurrences[activeOccurrenceIndex] = occurrence
            missionOccurrences = occurrences
        }
    }

    function _currentMissionSequence() {
        if (!activeVehicle
                || !activeVehicle.armed
                || !activeVehicle.missionItemIndex) {
            return -1
        }

        const sequenceNumber = Number(activeVehicle.missionItemIndex.rawValue)
        return isFinite(sequenceNumber)
                && sequenceNumber >= 0
                && sequenceNumber < 65535
                ? Math.floor(sequenceNumber)
                : -1
    }

    function refreshTargetItem() {
        const sequenceNumber = _currentMissionSequence()
        _targetSequenceNumber = sequenceNumber
        _targetMissionItem = null
        routeText = "-- >>> --"
        routeDetailText = ""

        if (sequenceNumber < 0) {
            updateTargetMetrics()
            _updateMissionTimerState()
            return
        }

        _applyMissionSequence(sequenceNumber)
        if (activeOccurrenceIndex >= 0
                && activeOccurrenceIndex < missionOccurrences.length) {
            const occurrence = missionOccurrences[activeOccurrenceIndex]
            _targetMissionItem = occurrence.missionItem
            _updateRouteText()
        }

        updateTargetMetrics()
        _updateMissionTimerState()
    }

    function updateTargetMetrics() {
        horizontalError = Number.NaN
        verticalError = Number.NaN

        if (!activeVehicle
                || !_targetMissionItem
                || !_isValidCoordinate(activeVehicle.coordinate)
                || !_isValidCoordinate(_targetMissionItem.coordinate)) {
            return
        }

        const distanceMeters = activeVehicle.coordinate.distanceTo(
                    _targetMissionItem.coordinate)
        if (isFinite(distanceMeters)) {
            horizontalError = Math.max(0, distanceMeters)
        }

        const vehicleAltitudeAMSL = activeVehicle.altitudeAMSL
                ? Number(activeVehicle.altitudeAMSL.rawValue)
                : Number.NaN
        const targetAltitudeAMSL = Number(_targetMissionItem.amslEntryAlt)
        if (isFinite(vehicleAltitudeAMSL) && isFinite(targetAltitudeAMSL)) {
            verticalError = Math.abs(targetAltitudeAMSL - vehicleAltitudeAMSL)
        }
        _recordActiveOccurrenceErrors()
    }

    function _formatMissionElapsedTime(elapsedMilliseconds) {
        const totalSeconds = Math.max(0, Math.floor(elapsedMilliseconds / 1000))
        const hours = Math.floor(totalSeconds / 3600)
        const minutes = Math.floor((totalSeconds % 3600) / 60)
        const seconds = totalSeconds % 60
        const minuteText = String(minutes).padStart(2, "0")
        const secondText = String(seconds).padStart(2, "0")
        return hours > 0
                ? String(hours).padStart(2, "0") + ":" + minuteText + ":" + secondText
                : minuteText + ":" + secondText
    }

    function _updateMissionTimeText() {
        if (!isFinite(_missionStartTimestampMs)) {
            missionTimeText = "--"
            return
        }

        const endTimestamp = _missionTimerRunning
                ? Date.now()
                : _missionStopTimestampMs
        missionTimeText = isFinite(endTimestamp)
                ? _formatMissionElapsedTime(endTimestamp - _missionStartTimestampMs)
                : "--"
    }

    function _resetMissionTimer() {
        _missionStartTimestampMs = Number.NaN
        _missionStopTimestampMs = Number.NaN
        _missionTimerRunning = false
        missionTimeText = "--"
    }

    function _updateMissionTimerState() {
        const missionExecuting = activeVehicle
                && activeVehicle.armed
                && (activeVehicle.flying || activeVehicle.landing)
                && _targetSequenceNumber >= 0

        if (missionExecuting) {
            if (!isFinite(_missionStartTimestampMs)) {
                _missionStartTimestampMs = Date.now()
                _missionStopTimestampMs = Number.NaN
            }
            _missionTimerRunning = true
            _updateMissionTimeText()
        } else if (_missionTimerRunning) {
            _missionStopTimestampMs = Date.now()
            _missionTimerRunning = false
            _updateMissionTimeText()
        }
    }

    function _handleMissionExecutionStateChange() {
        if (activeVehicle
                && !activeVehicle.flying
                && !activeVehicle.landing
                && _missionTimerRunning) {
            _completeActiveOccurrence()
        }
        _updateMissionTimerState()
    }

    function rebuildRoute() {
        if (!missionAvailable || !missionController) {
            hasMissionData = false
            routeDataValid = false
            _routeSegments = []
            _routeMarkers = []
            _takeoffVisual = null
            _landingVisual = null
            _homePadVisual = null
            startPoint = Qt.point(routeArea.width / 2, routeArea.height / 2)
            updateAircraftPoint()
            routeCanvas.requestPaint()
            return
        }

        const visualItems = missionController ? missionController.visualItems : null
        const flightSegments = missionController ? missionController.simpleFlightPathSegments : null
        const geoSegments = []
        const uniqueCoordinates = []
        let takeoffItem = null
        let landingItem = null

        if (flightSegments) {
            for (let index = 0; index < flightSegments.count; index++) {
                const segment = flightSegments.get(index)
                if (!segment
                        || !_isValidCoordinate(segment.coordinate1)
                        || !_isValidCoordinate(segment.coordinate2)
                        || _coordinatesMatch(segment.coordinate1, segment.coordinate2)) {
                    continue
                }

                geoSegments.push({
                    coordinate1: segment.coordinate1,
                    coordinate2: segment.coordinate2
                })

                if (_findCoordinateIndex(uniqueCoordinates, segment.coordinate1) < 0) {
                    uniqueCoordinates.push(segment.coordinate1)
                }
                if (_findCoordinateIndex(uniqueCoordinates, segment.coordinate2) < 0) {
                    uniqueCoordinates.push(segment.coordinate2)
                }
            }
        }

        if (visualItems) {
            for (let index = 0; index < visualItems.count; index++) {
                const item = visualItems.get(index)

                if (item && item.isSimpleItem && _isValidCoordinate(item.coordinate)) {
                    const command = Number(item.command)
                    if (!takeoffItem && _isTakeoffCommand(command)) {
                        takeoffItem = item
                    }
                    if (_isLandingCommand(command)) {
                        landingItem = item
                    }
                }

                if (!item
                        || !_isValidCoordinate(item.coordinate)
                        || (!item.homePosition && (!item.specifiesCoordinate || item.isStandaloneCoordinate))) {
                    continue
                }

                if (_findCoordinateIndex(uniqueCoordinates, item.coordinate) < 0) {
                    uniqueCoordinates.push(item.coordinate)
                }
            }
        }

        if (uniqueCoordinates.length === 0 || routeArea.width <= 0 || routeArea.height <= 0) {
            hasMissionData = false
            routeDataValid = false
            _routeSegments = []
            _routeMarkers = []
            _takeoffVisual = null
            _landingVisual = null
            _homePadVisual = null
            startPoint = Qt.point(routeArea.width / 2, routeArea.height / 2)
            updateAircraftPoint()
            routeCanvas.requestPaint()
            return
        }

        const referenceLatitude = uniqueCoordinates[0].latitude
        const referenceLongitude = uniqueCoordinates[0].longitude
        const cosLatitude = Math.cos(referenceLatitude * Math.PI / 180)
        let minX = Number.POSITIVE_INFINITY
        let maxX = Number.NEGATIVE_INFINITY
        let minY = Number.POSITIVE_INFINITY
        let maxY = Number.NEGATIVE_INFINITY

        for (let index = 0; index < uniqueCoordinates.length; index++) {
            const coordinate = uniqueCoordinates[index]
            const localX = (coordinate.longitude - referenceLongitude) * cosLatitude
            const localY = coordinate.latitude - referenceLatitude
            minX = Math.min(minX, localX)
            maxX = Math.max(maxX, localX)
            minY = Math.min(minY, localY)
            maxY = Math.max(maxY, localY)
        }

        const padding = Math.max(_markerDiameter * 2.4, Math.min(routeArea.width,
                                                                routeArea.height) * 0.08)
        const availableWidth = Math.max(1, routeArea.width - (padding * 2))
        const availableHeight = Math.max(1, routeArea.height - (padding * 2))
        const rangeX = maxX - minX
        const rangeY = maxY - minY
        const scaleX = rangeX > _rangeEpsilon ? availableWidth / rangeX : Number.POSITIVE_INFINITY
        const scaleY = rangeY > _rangeEpsilon ? availableHeight / rangeY : Number.POSITIVE_INFINITY
        let scale = Math.min(scaleX, scaleY)
        if (!isFinite(scale)) {
            scale = 1
        }

        const projectedWidth = rangeX * scale
        const projectedHeight = rangeY * scale
        const projection = {
            referenceLatitude: referenceLatitude,
            referenceLongitude: referenceLongitude,
            cosLatitude: cosLatitude,
            minX: minX,
            maxY: maxY,
            scale: scale,
            offsetX: padding + ((availableWidth - projectedWidth) / 2),
            offsetY: padding + ((availableHeight - projectedHeight) / 2)
        }
        const projectedSegments = []
        const projectedMarkers = []

        for (let index = 0; index < geoSegments.length; index++) {
            projectedSegments.push({
                point1: _projectCoordinate(geoSegments[index].coordinate1, projection),
                point2: _projectCoordinate(geoSegments[index].coordinate2, projection),
                destinationMarkerIndex: _findCoordinateIndex(uniqueCoordinates,
                                                              geoSegments[index].coordinate2)
            })
        }

        const homeCoordinate = _missionHomeCoordinate(visualItems)

        for (let index = 0; index < uniqueCoordinates.length; index++) {
            const coordinate = uniqueCoordinates[index]
            projectedMarkers.push({
                point: _projectCoordinate(coordinate, projection),
                label: _labelForCoordinate(coordinate, homeCoordinate),
                debugLabel: _debugLabelForCoordinate(coordinate),
                markerIndex: index,
                labelAbove: index % 2 === 1
            })
        }

        const homeGroundPoint = homeCoordinate
                ? _projectCoordinate(homeCoordinate, projection)
                : null
        const verticalLineOffset = Math.max(7, Math.min(10, _markerDiameter * 0.55))
        let takeoffVisual = null
        let landingVisual = null

        if (takeoffItem && homeGroundPoint) {
            const lineX = homeGroundPoint.x - verticalLineOffset
            const visualHeight = _verticalVisualHeight(takeoffItem)
            takeoffVisual = {
                groundPoint: homeGroundPoint,
                airbornePoint: Qt.point(lineX, homeGroundPoint.y - visualHeight),
                lineX: lineX,
                visualHeight: visualHeight,
                label: qsTr("TO")
            }
        }

        if (landingItem && homeGroundPoint) {
            const lineX = homeGroundPoint.x + verticalLineOffset
            const visualHeight = _verticalVisualHeight(landingItem)
            landingVisual = {
                groundPoint: homeGroundPoint,
                airbornePoint: Qt.point(lineX, homeGroundPoint.y - visualHeight),
                lineX: lineX,
                visualHeight: visualHeight,
                label: qsTr("LAND")
            }
        }

        const startCoordinate = geoSegments.length > 0
                ? geoSegments[0].coordinate1
                : uniqueCoordinates[0]

        _routeSegments = projectedSegments
        _routeMarkers = projectedMarkers
        _takeoffVisual = takeoffVisual
        _landingVisual = landingVisual
        _homePadVisual = homeGroundPoint ? { point: homeGroundPoint } : null
        startPoint = takeoffVisual
                ? takeoffVisual.airbornePoint
                : _projectCoordinate(startCoordinate, projection)
        hasMissionData = true
        routeDataValid = true
        updateAircraftPoint()
        routeCanvas.requestPaint()
    }

    Timer {
        interval: 200
        repeat: true
        running: root.activeVehicle !== null
                 && root.activeVehicle !== undefined
                 && root._targetMissionItem !== null

        onTriggered: root.updateTargetMetrics()
    }

    Timer {
        interval: 1000
        repeat: true
        running: root._missionTimerRunning

        onTriggered: root._updateMissionTimeText()
    }

    Item {
        id: routeArea

        anchors.left:   parent.left
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        width:          parent.width * root.routeAreaRatio
        clip:           true
    }

    Rectangle {
        id: routeInfoDivider

        anchors.left:   routeArea.right
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        width:          1
        color:          Qt.rgba(0.32, 0.42, 0.48, 0.35)
    }

    Rectangle {
        id: infoArea

        anchors.left:   routeInfoDivider.right
        anchors.right:  parent.right
        anchors.top:    parent.top
        anchors.bottom: parent.bottom
        color:          Qt.rgba(0.01, 0.04, 0.07, 0.28)

        readonly property real infoLabelFontSize:
            Math.max(11, Math.min(15, width * 0.055))
        readonly property real infoValueFontSize:
            Math.max(15, Math.min(20, width * 0.080))
        readonly property real routeValueFontSize:
            Math.max(18, Math.min(24, width * 0.095))
        readonly property real routeDetailFontSize:
            Math.max(10, Math.min(13, width * 0.050))

        Column {
            id: infoColumn

            readonly property real rowSpacing:     Math.max(2, height * 0.014)
            readonly property real routeRowHeight: Math.max(1, height * 0.20)
            readonly property real rowHeight:
                Math.max(1, (height - routeRowHeight - (rowSpacing * 6)) / 6)

            anchors.fill:    parent
            anchors.margins: Math.max(5, Math.min(parent.width, parent.height) * 0.055)
            spacing:         rowSpacing

            Repeater {
                model: 7

                delegate: Item {
                    id: infoRow

                    required property int index

                    width:  infoColumn.width
                    height: infoRow.index === 0
                            ? infoColumn.routeRowHeight
                            : infoColumn.rowHeight

                    QGCLabel {
                        anchors.left:           parent.left
                        anchors.verticalCenter: parent.verticalCenter
                        width:                  parent.width * 0.52
                        text:                   root._infoLabel(infoRow.index)
                        color:                  root._standbyColor
                        elide:                  Text.ElideRight
                        font.pixelSize:         infoArea.infoLabelFontSize
                    }

                    QGCLabel {
                        anchors.right:          parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        width:                  infoRow.index === 0 ? parent.width : parent.width * 0.47
                        text:                   root._infoDisplayValue(infoRow.index)
                        color:                  root._standbyBorderColor
                        elide:                  Text.ElideRight
                        horizontalAlignment:    Text.AlignRight
                        font.bold:              true
                        font.pixelSize:         infoRow.index === 0
                                                ? infoArea.routeValueFontSize
                                                : infoArea.infoValueFontSize
                    }

                    QGCLabel {
                        anchors.right:  parent.right
                        anchors.bottom: parent.bottom
                        visible:        infoRow.index === 0 && root.routeDetailText.length > 0
                        width:          parent.width
                        text:           root.routeDetailText
                        color:          root._standbyColor
                        elide:          Text.ElideRight
                        horizontalAlignment: Text.AlignRight
                        font.bold:      true
                        font.pixelSize: infoArea.routeDetailFontSize
                    }
                }
            }
        }
    }

    Canvas {
        id: routeCanvas
        parent: routeArea
        anchors.fill: parent

        onPaint: {
            const context = getContext("2d")
            context.reset()

            context.lineWidth = root._lineWidth
            context.strokeStyle = root._baseRouteColor
            context.lineCap = "round"
            context.lineJoin = "round"

            for (let index = 0; index < root._routeSegments.length; index++) {
                const segment = root._routeSegments[index]
                context.beginPath()
                context.moveTo(segment.point1.x, segment.point1.y)
                context.lineTo(segment.point2.x, segment.point2.y)
                context.stroke()
            }

            for (let index = 0; index < root._routeSegments.length; index++) {
                const segment = root._routeSegments[index]
                if (root._segmentState(segment, index) === root.waypointPending) {
                    continue
                }

                context.lineWidth = root._lineWidth * 1.5
                context.strokeStyle = root._segmentColor(segment, index)
                context.beginPath()
                context.moveTo(segment.point1.x, segment.point1.y)
                context.lineTo(segment.point2.x, segment.point2.y)
                context.stroke()
            }

            if (root._takeoffVisual) {
                const takeoff = root._takeoffVisual
                const arrowSize = Math.max(4, root._markerDiameter * 0.42)

                context.lineWidth = root._verticalLineWidth
                context.strokeStyle = root._takeoffColor
                context.beginPath()
                context.moveTo(takeoff.lineX, takeoff.groundPoint.y)
                context.lineTo(takeoff.lineX, takeoff.airbornePoint.y)
                context.stroke()

                context.fillStyle = root._takeoffColor
                context.beginPath()
                context.moveTo(takeoff.lineX, takeoff.airbornePoint.y)
                context.lineTo(takeoff.lineX - arrowSize, takeoff.airbornePoint.y + arrowSize)
                context.lineTo(takeoff.lineX + arrowSize, takeoff.airbornePoint.y + arrowSize)
                context.closePath()
                context.fill()
            }

            if (root._landingVisual) {
                const landing = root._landingVisual
                const arrowSize = Math.max(4, root._markerDiameter * 0.42)
                const arrowTipY = landing.groundPoint.y - (root._markerDiameter * 0.35)

                context.lineWidth = root._verticalLineWidth
                context.strokeStyle = root._landingColor
                context.beginPath()
                context.moveTo(landing.lineX, landing.airbornePoint.y)
                context.lineTo(landing.lineX, arrowTipY)
                context.stroke()

                context.fillStyle = root._landingColor
                context.beginPath()
                context.moveTo(landing.lineX, arrowTipY)
                context.lineTo(landing.lineX - arrowSize, arrowTipY - arrowSize)
                context.lineTo(landing.lineX + arrowSize, arrowTipY - arrowSize)
                context.closePath()
                context.fill()
            }
        }

        onWidthChanged: root.rebuildRoute()
        onHeightChanged: root.rebuildRoute()
        Component.onCompleted: root.rebuildRoute()
    }

    Repeater {
        parent: routeArea
        model: root._routeMarkers

        delegate: Item {
            id: waypoint

            required property var modelData

            width:  root._markerDiameter
            height: width
            x:      modelData.point.x - (width / 2)
            y:      modelData.point.y - (height / 2)

            Rectangle {
                anchors.fill: parent
                radius:       width / 2
                color:        root._waypointColor(waypoint.modelData.markerIndex)
                border.color: root._standbyBorderColor
                border.width: Math.max(1, root._lineWidth * 0.5)
            }

            QGCLabel {
                id: waypointLabel

                x:              (parent.width - width) / 2
                y:              waypoint.modelData.labelAbove
                                ? -height - (root._markerDiameter * 0.22)
                                : parent.height + (root._markerDiameter * 0.18)
                text:           waypoint.modelData.label
                color:          root._waypointColor(waypoint.modelData.markerIndex)
                font.bold:      false
                font.pixelSize: root._labelPixelSize
            }

            QGCLabel {
                visible:        root.showDebugSequenceNumbers
                                && waypoint.modelData.debugLabel.length > 0
                x:              (parent.width - width) / 2
                y:              waypoint.modelData.labelAbove
                                ? waypointLabel.y - height
                                : waypointLabel.y + waypointLabel.height
                text:           waypoint.modelData.debugLabel
                color:          root._standbyColor
                font.pixelSize: Math.max(7, root._labelPixelSize - 2)
            }
        }
    }

    Item {
        id: homePad
        parent: routeArea

        visible: root._homePadVisual !== null
        width:   root._markerDiameter * 2.2
        height:  root._markerDiameter * 0.9
        x:       visible ? root._homePadVisual.point.x - (width / 2) : 0
        y:       visible ? root._homePadVisual.point.y - (height / 2) : 0

        Rectangle {
            width:  parent.width
            height: parent.height
            y:      Math.max(2, root._lineWidth)
            radius: height / 2
            color:  Qt.rgba(0, 0, 0, 0.30)
        }

        Rectangle {
            anchors.fill: parent
            radius:       height / 2
            color:        root._vertiportColor
            border.color: root._standbyBorderColor
            border.width: Math.max(1, root._lineWidth * 0.7)

            Rectangle {
                anchors.centerIn: parent
                width:            parent.width * 0.48
                height:           parent.height * 0.34
                radius:           height / 2
                color:            "transparent"
                border.color:     root._standbyBorderColor
                border.width:     Math.max(1, root._lineWidth * 0.5)
            }
        }
    }

    Item {
        id: takeoffAirborneMarker
        parent: routeArea

        visible: root._takeoffVisual !== null
        width:   root._markerDiameter * 0.65
        height:  width
        x:       visible ? root._takeoffVisual.airbornePoint.x - (width / 2) : 0
        y:       visible ? root._takeoffVisual.airbornePoint.y - (height / 2) : 0

        Rectangle {
            anchors.fill: parent
            radius:       width / 2
            color:        root._takeoffColor
            border.color: root._standbyBorderColor
            border.width: Math.max(1, root._lineWidth * 0.6)
        }
    }

    QGCLabel {
        parent: routeArea

        visible:        root._takeoffVisual !== null
        x:              !visible ? 0
                        : root._takeoffVisual.lineX - implicitWidth - (root._markerDiameter * 0.65)
        y:              !visible ? 0
                        : ((root._takeoffVisual.groundPoint.y
                            + root._takeoffVisual.airbornePoint.y) / 2) - (implicitHeight / 2)
        text:           visible ? root._takeoffVisual.label : ""
        color:          root._takeoffColor
        font.bold:      true
        font.pixelSize: root._labelPixelSize
    }

    Item {
        id: landingAirborneMarker
        parent: routeArea

        visible: root._landingVisual !== null
        width:   root._markerDiameter * 0.65
        height:  width
        x:       visible ? root._landingVisual.airbornePoint.x - (width / 2) : 0
        y:       visible ? root._landingVisual.airbornePoint.y - (height / 2) : 0

        Rectangle {
            anchors.fill: parent
            radius:       width / 2
            color:        root._landingColor
            border.color: root._standbyBorderColor
            border.width: Math.max(1, root._lineWidth * 0.6)
        }
    }

    QGCLabel {
        parent: routeArea

        visible:        root._landingVisual !== null
        x:              !visible ? 0
                        : root._landingVisual.lineX + (root._markerDiameter * 0.65)
        y:              !visible ? 0
                        : ((root._landingVisual.groundPoint.y
                            + root._landingVisual.airbornePoint.y) / 2) - (implicitHeight / 2)
        text:           visible ? root._landingVisual.label : ""
        color:          root._landingColor
        font.bold:      true
        font.pixelSize: root._labelPixelSize
    }

    Repeater {
        model: missionController ? missionController.visualItems : null

        delegate: Item {
            id: visualItemWatcher

            required property var object

            visible: false
            width:   0
            height:  0

            Connections {
                target: visualItemWatcher.object
                ignoreUnknownSignals: true

                function onCoordinateChanged() {
                    root.rebuildRoute()
                    root.rebuildMissionProgressModel()
                }

                function onSequenceNumberChanged() {
                    root.rebuildRoute()
                    root.rebuildMissionProgressModel()
                    root.refreshTargetItem()
                }

                function onLastSequenceNumberChanged() {
                    root.rebuildMissionProgressModel()
                    root.refreshTargetItem()
                }

                function onCommandChanged() {
                    root.rebuildRoute()
                    root.rebuildMissionProgressModel()
                }

                function onAmslEntryAltChanged() {
                    root.rebuildRoute()
                    root.updateTargetMetrics()
                }

                function onSpecifiesCoordinateChanged() {
                    root.rebuildRoute()
                    root.rebuildMissionProgressModel()
                }

                function onIsStandaloneCoordinateChanged() {
                    root.rebuildRoute()
                    root.rebuildMissionProgressModel()
                }
            }

            Connections {
                target: visualItemWatcher.object && visualItemWatcher.object.isSimpleItem
                        ? visualItemWatcher.object.altitude
                        : null
                ignoreUnknownSignals: true

                function onRawValueChanged() {
                    root.rebuildRoute()
                }
            }
        }
    }

    Repeater {
        model: missionController ? missionController.simpleFlightPathSegments : null

        delegate: Item {
            id: segmentWatcher

            required property var object

            visible: false
            width:   0
            height:  0

            Connections {
                target: segmentWatcher.object

                function onCoordinate1Changed() {
                    root.rebuildRoute()
                }

                function onCoordinate2Changed() {
                    root.rebuildRoute()
                }
            }
        }
    }

    QGCLabel {
        parent: routeArea

        anchors.centerIn: parent
        visible:          !root.hasMissionData
        text:             qsTr("NO MISSION DATA")
        color:            root._standbyColor
        font.bold:        true
        font.pixelSize:   root._labelPixelSize
    }

    Connections {
        target: missionController
        ignoreUnknownSignals: true

        function onVisualItemsReset() {
            root.rebuildRoute()
            root.rebuildMissionProgressModel()
            root.refreshTargetItem()
        }

        function onNewItemsFromVehicle() {
            root.rebuildRoute()
            root.rebuildMissionProgressModel()
            root.refreshTargetItem()
        }

        function onCurrentMissionIndexChanged() {
            root.refreshTargetItem()
        }
    }

    Connections {
        target: missionController ? missionController.visualItems : null
        ignoreUnknownSignals: true

        function onCountChanged() {
            root.rebuildRoute()
            root.rebuildMissionProgressModel()
            root.refreshTargetItem()
        }

        function onModelReset() {
            root.rebuildRoute()
            root.rebuildMissionProgressModel()
            root.refreshTargetItem()
        }
    }

    Connections {
        target: missionController ? missionController.simpleFlightPathSegments : null
        ignoreUnknownSignals: true

        function onModelReset() {
            root.rebuildRoute()
        }
    }

    Connections {
        target: root.activeVehicle
        ignoreUnknownSignals: true

        function onArmedChanged() {
            if (root.activeVehicle && root.activeVehicle.armed) {
                root._resetMissionTimer()
                root._resetOccurrenceProgress()
            } else {
                root._completeActiveOccurrence()
            }
            root.refreshTargetItem()
            root._updateMissionTimerState()
        }

        function onFlyingChanged() {
            root._handleMissionExecutionStateChange()
        }

        function onLandingChanged() {
            root._handleMissionExecutionStateChange()
        }
    }

    Connections {
        target: root.activeVehicle ? root.activeVehicle.missionItemIndex : null
        ignoreUnknownSignals: true

        function onRawValueChanged() {
            root.refreshTargetItem()
        }
    }

    Connections {
        target: root._targetMissionItem
        ignoreUnknownSignals: true

        function onCoordinateChanged() {
            root.updateTargetMetrics()
        }

        function onAmslEntryAltChanged() {
            root.updateTargetMetrics()
        }
    }

    onMissionControllerChanged: {
        rebuildRoute()
        rebuildMissionProgressModel()
        refreshTargetItem()
    }
    onMissionAvailableChanged: {
        rebuildRoute()
        rebuildMissionProgressModel()
        refreshTargetItem()
    }
    onActiveVehicleChanged: {
        _resetMissionTimer()
        rebuildMissionProgressModel()
        refreshTargetItem()
    }
    onActiveWaypointIndexChanged: routeCanvas.requestPaint()
    onCurrentLegIndexChanged: {
        updateAircraftPoint()
        routeCanvas.requestPaint()
    }
    onCurrentLegProgressChanged: updateAircraftPoint()
    onWaypointStatesChanged: routeCanvas.requestPaint()
    onPhysicalWaypointGroupsChanged: rebuildRoute()

    Component.onCompleted: {
        rebuildMissionProgressModel()
        refreshTargetItem()
    }
}
