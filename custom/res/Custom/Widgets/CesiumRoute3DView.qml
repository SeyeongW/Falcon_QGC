import QtQuick
import QtWebChannel
import QtWebEngine

Item {
    id: root

    property var missionOccurrences: []
    property var physicalWaypointGroups: []
    property var traversalOccurrences: []
    property int missionProgressRevision: 0
    property bool showDebugSequenceNumbers: false
    property var activeVehicle
    property var missionController
    property bool missionAvailable: false
    property int activeWaypointIndex: -1
    property int currentLegIndex: -1
    property real currentLegProgress: 0
    property var waypointStates: []

    property bool routeDataValid: false
    property bool displayReady: false
    property bool loadFailed: false
    property string loadError: ""

    readonly property string ionAccessToken: (typeof customCesiumIonToken !== "undefined")
                                                    ? customCesiumIonToken : ""

    function _isValidCoordinate(coordinate) {
        return coordinate
                && coordinate.isValid
                && isFinite(Number(coordinate.latitude))
                && isFinite(Number(coordinate.longitude))
    }

    function _altitudeForOccurrence(occurrence) {
        const altitudeAMSL = Number(occurrence.altitudeAMSL)
        if (isFinite(altitudeAMSL)) {
            return altitudeAMSL
        }
        const coordinateAltitude = occurrence.coordinate
                ? Number(occurrence.coordinate.altitude) : Number.NaN
        return isFinite(coordinateAltitude) ? coordinateAltitude : 0
    }

    function _labelForOccurrence(occurrence, index) {
        const groupIndex = Number(occurrence.physicalWaypointIndex)
        if (physicalWaypointGroups
                && groupIndex >= 0
                && groupIndex < physicalWaypointGroups.length) {
            const group = physicalWaypointGroups[groupIndex]
            if (group && group.label) {
                return String(group.label)
            }
        }
        if (occurrence.isTakeoff) {
            return qsTr("TAKEOFF")
        }
        if (occurrence.isLand) {
            return qsTr("LAND")
        }
        return occurrence.isWaypoint ? qsTr("WP %1").arg(index) : ""
    }

    function _vehicleSnapshot() {
        if (!activeVehicle || !_isValidCoordinate(activeVehicle.coordinate)) {
            return null
        }

        const altitudeAMSL = activeVehicle.altitudeAMSL
                ? Number(activeVehicle.altitudeAMSL.rawValue) : Number.NaN
        const coordinateAltitude = Number(activeVehicle.coordinate.altitude)
        const heading = activeVehicle.heading
                ? Number(activeVehicle.heading.rawValue) : 0
        const pitch = activeVehicle.pitch
                ? Number(activeVehicle.pitch.rawValue) : 0
        const roll = activeVehicle.roll
                ? Number(activeVehicle.roll.rawValue) : 0

        return {
            latitude: Number(activeVehicle.coordinate.latitude),
            longitude: Number(activeVehicle.coordinate.longitude),
            altitude: isFinite(altitudeAMSL)
                      ? altitudeAMSL
                      : (isFinite(coordinateAltitude) ? coordinateAltitude : 0),
            heading: isFinite(heading) ? heading : 0,
            pitch: isFinite(pitch) ? pitch : 0,
            roll: isFinite(roll) ? roll : 0,
            armed: Boolean(activeVehicle.armed),
            flying: Boolean(activeVehicle.flying),
            landing: Boolean(activeVehicle.landing)
        }
    }

    function _snapshotJson() {
        const route = []
        for (let index = 0; index < missionOccurrences.length; index++) {
            const occurrence = missionOccurrences[index]
            if (!occurrence || !_isValidCoordinate(occurrence.coordinate)) {
                continue
            }
            route.push({
                latitude: Number(occurrence.coordinate.latitude),
                longitude: Number(occurrence.coordinate.longitude),
                altitude: _altitudeForOccurrence(occurrence),
                label: _labelForOccurrence(occurrence, index),
                state: Number(occurrence.state),
                sequenceNumber: Number(occurrence.sequenceNumber),
                isTakeoff: Boolean(occurrence.isTakeoff),
                isLand: Boolean(occurrence.isLand),
                isWaypoint: Boolean(occurrence.isWaypoint)
            })
        }

        routeDataValid = route.length > 0
        return JSON.stringify({
            revision: missionProgressRevision,
            route: route,
            vehicle: _vehicleSnapshot(),
            currentLegIndex: currentLegIndex,
            currentLegProgress: currentLegProgress,
            showDebugSequenceNumbers: showDebugSequenceNumbers
        })
    }

    function publishSnapshot() {
        bridge.snapshotJson = _snapshotJson()
    }

    function fail(message) {
        if (loadFailed) {
            return
        }
        loadError = message || qsTr("Cesium 3D map failed to load")
        loadFailed = true
        displayReady = false
        console.error("[CesiumRoute3DView]", loadError)
    }

    function logDebugState(reason) {
        console.log("[CesiumRoute3DView]", reason,
                    "routeDataValid:", routeDataValid,
                    "displayReady:", displayReady,
                    "loadFailed:", loadFailed,
                    "route points:", missionOccurrences.length)
    }

    QtObject {
        id: bridge

        WebChannel.id: "fgcBridge"
        property string snapshotJson: "{}"
        property string ionAccessToken: root.ionAccessToken

        function reportReady() {
            root.displayReady = true
            root.loadFailed = false
            root.loadError = ""
            root.publishSnapshot()
        }

        function reportError(message) {
            root.fail(String(message))
        }
    }

    WebChannel {
        id: webChannel
        registeredObjects: [bridge]
    }

    WebEngineView {
        id: webView
        anchors.fill: parent
        url: "qrc:/qml/Custom/Widgets/Cesium/cesium_route.html"
        webChannel: webChannel
        backgroundColor: "#071526"

        settings.webGLEnabled: true
        settings.localContentCanAccessRemoteUrls: true

        onLoadingChanged: function(loadRequest) {
            console.log("[CesiumRoute3DView] WebEngine loading status:",
                        loadRequest.status,
                        "url:", loadRequest.url,
                        "error:", loadRequest.errorString)
            if (loadRequest.status === WebEngineView.LoadFailedStatus) {
                root.fail(qsTr("Cesium page load failed: %1").arg(loadRequest.errorString))
            }
        }

        onRenderProcessTerminated: function(terminationStatus, exitCode) {
            root.fail(qsTr("Cesium render process stopped (%1, %2)")
                      .arg(terminationStatus).arg(exitCode))
        }

        onJavaScriptConsoleMessage: function(level, message, lineNumber, sourceId) {
            if (level === WebEngineView.ErrorMessageLevel) {
                console.error("[CesiumJS]", sourceId + ":" + lineNumber, message)
            } else {
                console.log("[CesiumJS]", message)
            }
        }
    }

    Timer {
        interval: 250
        running: root.visible && !root.loadFailed
        repeat: true
        onTriggered: root.publishSnapshot()
    }

    onMissionProgressRevisionChanged: publishSnapshot()
    onMissionOccurrencesChanged: publishSnapshot()
    onPhysicalWaypointGroupsChanged: publishSnapshot()
    onCurrentLegIndexChanged: publishSnapshot()
    onCurrentLegProgressChanged: publishSnapshot()
    Component.onCompleted: publishSnapshot()
}
