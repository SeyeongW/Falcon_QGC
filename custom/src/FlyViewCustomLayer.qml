import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets

Item {
    id: root

    property var parentToolInsets                       // These insets tell you what screen real estate is available for positioning the controls in your overlay
    property var totalToolInsets:   _totalToolInsets    // The insets updated for the custom overlay additions
    property var mapControl
    property real lowerPanelHeight
    property real topPanelHeight
    property real consolePanelWidth
    property real paneSpacing
    property bool use3DRouteView: true
    property bool force3DVisibleForDebug: false

    // Active mission phase relayed from the (ROS-only) MissionPhasePanel, so the
    // fly-view layout can adapt without importing Custom.Ros into core. -1 when
    // the panel is absent (non-ROS build) or the orchestrator link is down.
    readonly property int missionPhase: (missionPhaseLoader.item &&
                                         missionPhaseLoader.item.activePhase !== undefined)
                                            ? missionPhaseLoader.item.activePhase : -1

    readonly property string noGPS:         qsTr("NO GPS")
    readonly property real   indicatorValueWidth:   ScreenTools.defaultFontPixelWidth * 7
    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
    property var    _missionController:     (typeof globals !== "undefined" && globals.planMasterControllerFlyView)
                                              ? globals.planMasterControllerFlyView.missionController : null
    property real   _indicatorDiameter:     ScreenTools.defaultFontPixelWidth * 18
    property real   _indicatorsHeight:      ScreenTools.defaultFontPixelHeight
    property var    _sepColor:              qgcPal.globalTheme === QGCPalette.Light ? Qt.rgba(0,0,0,0.5) : Qt.rgba(1,1,1,0.5)
    property color  _indicatorsColor:       qgcPal.text
    property bool   _isVehicleGps:          _activeVehicle ? _activeVehicle.gps.count.rawValue > 1 && _activeVehicle.gps.hdop.rawValue < 1.4 : false
    property string _altitude:              _activeVehicle ? (isNaN(_activeVehicle.altitudeRelative.value) ? "0.0" : _activeVehicle.altitudeRelative.value.toFixed(1)) + ' ' + _activeVehicle.altitudeRelative.units : "0.0"
    property string _distanceStr:           isNaN(_distance) ? "0" : _distance.toFixed(0) + ' ' + QGroundControl.unitsConversion.appSettingsHorizontalDistanceUnitsString
    property real   _heading:               _activeVehicle   ? _activeVehicle.heading.rawValue : 0
    property real   _distance:              _activeVehicle ? _activeVehicle.distanceToHome.rawValue : 0
    property string _messageTitle:          ""
    property string _messageText:           ""
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color  _falconNavy:   FalconTheme.base
    readonly property color  _falconPanel:  FalconTheme.surface2
    readonly property color  _falconCyan:   FalconTheme.accent
    readonly property color  _falconBlue:   "#1D4ED8"
    readonly property color  _falconMint:   FalconTheme.textMuted

    // --- phase-driven emphasis ----------------------------------------------
    // Exactly one panel carries the left accent bar at a time. If everything is
    // emphasised, nothing is. Phase 0 and the route phases (1, 3) put the bar on
    // the lower panel; with no orchestrator link the mission console is what the
    // operator needs, so the bar sits there instead. Phase 2 marks neither -- the
    // video is emphasised by taking over the whole main pane.
    readonly property bool consoleIsFocus:    missionPhase < 0
    readonly property bool lowerPanelIsFocus: missionPhase === 0
                                              || missionPhase === 1
                                              || missionPhase === 3

    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value))
    }

    function secondsToHHMMSS(timeS) {
        var sec_num = parseInt(timeS, 10);
        var hours   = Math.floor(sec_num / 3600);
        var minutes = Math.floor((sec_num - (hours * 3600)) / 60);
        var seconds = sec_num - (hours * 3600) - (minutes * 60);
        if (hours   < 10) {hours   = "0"+hours;}
        if (minutes < 10) {minutes = "0"+minutes;}
        if (seconds < 10) {seconds = "0"+seconds;}
        return hours+':'+minutes+':'+seconds;
    }

    function telemetryValue(fact, showUnits) {
        if (!fact) {
            return qsTr("–")
        }
        return fact.enumOrValueString + (showUnits && fact.units ? " " + fact.units : "")
    }

    // Avionics readout stack: a dim, wide-tracked uppercase caption above a large
    // tabular-figure value. Caption and value are on separate lines (not side by
    // side) so a column of readouts aligns on the numbers, which is what the eye
    // scans in flight.
    component TelemetryCorner: Column {
        property var  values: []
        property real labelFontPixelSize: FalconTheme.fontCaption
        property real valueFontPixelSize: FalconTheme.fontValue

        spacing: FalconTheme.space2

        Repeater {
            model: values

            Column {
                spacing: 0

                QGCLabel {
                    text:                modelData.label
                    color:               FalconTheme.textMuted
                    font.family:         FalconTheme.fontFamily
                    font.pixelSize:      labelFontPixelSize
                    font.letterSpacing:  FalconTheme.captionLetterSpacing
                }

                QGCLabel {
                    text:            telemetryValue(modelData.fact, modelData.showUnits)
                    color:           FalconTheme.textPrimary
                    font.family:     FalconTheme.fontFamilyMono
                    font.pixelSize:  valueFontPixelSize
                }
            }
        }
    }

    /// Single signed-degree attitude value with a caption above it. Tints amber
    /// once |angle| reaches warnAngle so an excessive bank/pitch stands out.
    component AngleReadout: Column {
        property string label
        property real   angle:      0
        property real   warnAngle:  0       // 0 disables the warning tint
        property bool   wrap360:    false   // heading style: 000-359, zero padded
        property real   labelFontPixelSize: ScreenTools.defaultFontPixelHeight * 0.6
        property real   valueFontPixelSize: ScreenTools.defaultFontPixelHeight * 0.72

        spacing: ScreenTools.defaultFontPixelHeight * 0.05

        QGCLabel {
            anchors.horizontalCenter: parent.horizontalCenter
            text:               parent.label
            color:              FalconTheme.textMuted
            font.pixelSize:     parent.labelFontPixelSize
            font.letterSpacing: FalconTheme.captionLetterSpacing
        }

        QGCLabel {
            anchors.horizontalCenter: parent.horizontalCenter
            text: {
                if (!_activeVehicle) {
                    return qsTr("–")
                }
                if (parent.wrap360) {
                    const wrapped = ((parent.angle % 360) + 360) % 360
                    return ("00" + wrapped.toFixed(0)).slice(-3) + "°"
                }
                // Signed, so the operator reads the direction of the bank/pitch
                // and not just its magnitude.
                return (parent.angle >= 0 ? "+" : "−") + Math.abs(parent.angle).toFixed(1) + "°"
            }
            color: (parent.warnAngle > 0 && Math.abs(parent.angle) >= parent.warnAngle)
                       ? FalconTheme.caution : FalconTheme.textPrimary
            font.family:    FalconTheme.fontFamilyMono
            font.pixelSize: parent.valueFontPixelSize
        }
    }

    QGCToolInsets {
        id:                     _totalToolInsets
        leftEdgeTopInset:       parentToolInsets.leftEdgeTopInset
        leftEdgeCenterInset:    exampleRectangle.leftEdgeCenterInset
        leftEdgeBottomInset:    parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      parentToolInsets.rightEdgeTopInset
        rightEdgeCenterInset:   parentToolInsets.rightEdgeCenterInset
        rightEdgeBottomInset:   parentToolInsets.rightEdgeBottomInset
        topEdgeLeftInset:       parentToolInsets.topEdgeLeftInset
        topEdgeCenterInset:     compassArrowIndicator.y + compassArrowIndicator.height
        topEdgeRightInset:      parentToolInsets.topEdgeRightInset
        bottomEdgeLeftInset:    parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  parentToolInsets.bottomEdgeCenterInset
        bottomEdgeRightInset:   parentToolInsets.bottomEdgeRightInset
    }

    // This is an example of how you can use parent tool insets to position an element on the custom fly view layer
    // - we use parent topEdgeLeftInset to position the widget below the toolstrip
    // - we use parent bottomEdgeLeftInset to dodge the virtual joystick if enabled
    // - we use the parent leftEdgeTopInset to size our element to the same width as the ToolStripAction
    // - we export the width of this element as the leftEdgeCenterInset so that the map will recenter if the vehicle flys behind this element
    Rectangle {
        id: exampleRectangle
        visible: false // to see this example, set this to true. To view insets, enable the insets viewer FlyView.qml
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.topMargin: parentToolInsets.topEdgeLeftInset + _toolsMargin
        anchors.bottomMargin: parentToolInsets.bottomEdgeLeftInset + _toolsMargin
        anchors.leftMargin: _toolsMargin
        width: parentToolInsets.leftEdgeTopInset - _toolsMargin
        color: 'red'

        property real leftEdgeCenterInset: visible ? x + width : 0
    }

    //-------------------------------------------------------------------------
    //-- FGC panel: branding header + collapsible mission phases.
    //   Collapsed = just the FGC header (default). The ▼ toggle drops the mission
    //   phase list + abort controls below (scrollable, ROS build only). Drag to
    //   move, grab a corner to resize.
    Item {
        id:                         missionHeader
        property bool expanded:     true

        x:                          0
        y:                          0
        width:                      consolePanelWidth
        height:                     topPanelHeight

        Rectangle {
            anchors.fill:           parent
            radius:                 FalconTheme.radiusPanel
            color:                  FalconTheme.surface1
            border.width:           0
            clip:                   true

            // Left accent bar: marks the panel this phase is about, giving
            // hierarchy without enclosing anything in another box.
            Rectangle {
                anchors.left:       parent.left
                anchors.top:        parent.top
                anchors.bottom:     parent.bottom
                width:              FalconTheme.activeBarWidth
                color:              FalconTheme.accent
                z:                  10
                visible:            root.consoleIsFocus
            }

            ColumnLayout {
                anchors.fill:       parent
                anchors.margins:    _toolsMargin
                spacing:            _toolsMargin

                // --- FGC branding header (always visible). Logo, FGC and the
                //     status word share one vertical centre line. ---
                RowLayout {
                    Layout.fillWidth:       true
                    Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 2.2
                    spacing:                _toolsMargin

                    Image {
                        Layout.alignment:       Qt.AlignVCenter
                        Layout.preferredWidth:  ScreenTools.defaultFontPixelHeight * 2.2
                        Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 2.2
                        source:                 "qrc:/Custom/res/custom_qgroundcontrol.svg"
                        fillMode:               Image.PreserveAspectFit
                        mipmap:                 true
                    }

                    ColumnLayout {
                        Layout.alignment:       Qt.AlignVCenter
                        Layout.fillWidth:       true
                        spacing:                0

                        QGCLabel {
                            text:               qsTr("FGC")
                            color:              "white"
                            font.bold:          true
                            font.pointSize:     ScreenTools.defaultFontPointSize
                        }

                        QGCLabel {
                            visible:            _activeVehicle
                            text:               qsTr("LIVE MISSION CONSOLE")
                            color:              _falconCyan
                            font.pointSize:     ScreenTools.smallFontPointSize
                            font.letterSpacing: 1
                        }
                    }

                    QGCLabel {
                        Layout.alignment:     Qt.AlignRight | Qt.AlignVCenter
                        horizontalAlignment:  Text.AlignRight
                        text:                 _activeVehicle ? qsTr("VEHICLE") : qsTr("STANDBY")
                        color:                _activeVehicle ? "#86EFAC" : FalconTheme.caution
                        font.bold:            true
                        font.pointSize:       ScreenTools.smallFontPointSize
                    }
                }

                // --- collapsible mission phase panel (ROS build only) ---
                Loader {
                    id:                 missionPhaseLoader
                    Layout.fillWidth:   true
                    Layout.fillHeight:  true
                    visible:            missionHeader.expanded
                    active:             (typeof customRosEnabled !== 'undefined') && customRosEnabled
                    source:             active ? "qrc:/qml/Custom/Widgets/MissionPhasePanel.qml" : ""
                }

                // Bottom expand/collapse chevron — small, white, centered, like a
                // "show more" toggle on a web page. Reserves a thin strip so it
                // never overlaps the panel content.
                Item {
                    Layout.fillWidth:       true
                    Layout.preferredHeight: ScreenTools.defaultFontPixelHeight

                    QGCLabel {
                        anchors.centerIn:   parent
                        text:               missionHeader.expanded ? "▲" : "▼"
                        color:              "white"
                        opacity:            chevronMouse.containsMouse ? 1.0 : 0.55
                        font.pointSize:     ScreenTools.smallFontPointSize
                    }

                    MouseArea {
                        id:             chevronMouse
                        anchors.fill:   parent
                        hoverEnabled:   true
                        cursorShape:    Qt.PointingHandCursor
                        onClicked:      missionHeader.expanded = !missionHeader.expanded
                    }
                }
            }
        }
    }

    //-------------------------------------------------------------------------
    //-- Heading Indicator
    Rectangle {
        id:                         compassBar
        visible:                    false
        height:                     ScreenTools.defaultFontPixelHeight * 1.5
        width:                      ScreenTools.defaultFontPixelWidth  * 50
        anchors.top:                parent.top
        anchors.topMargin:          _toolsMargin
        color:                      FalconTheme.surface1
        radius:                     5
        border.color:               Qt.rgba(0.34, 0.59, 0.71, 0.62)
        border.width:               1
        clip:                       true
        anchors.horizontalCenter:   parent.horizontalCenter
        Repeater {
            model: 720
            QGCLabel {
                function _normalize(degrees) {
                    var a = degrees % 360
                    if (a < 0) a += 360
                    return a
                }
                property int _startAngle: modelData + 180 + _heading
                property int _angle: _normalize(_startAngle)
                anchors.verticalCenter: parent.verticalCenter
                x:              visible ? ((modelData * (compassBar.width / 360)) - (width * 0.5)) : 0
                visible:        _angle % 45 == 0
                color:          Qt.rgba(0.75, 0.88, 1.0, 0.72)
                font.pointSize: ScreenTools.smallFontPointSize
                text: {
                    switch(_angle) {
                    case 0:     return "N"
                    case 45:    return "NE"
                    case 90:    return "E"
                    case 135:   return "SE"
                    case 180:   return "S"
                    case 225:   return "SW"
                    case 270:   return "W"
                    case 315:   return "NW"
                    }
                    return ""
                }
            }
        }
    }
    Rectangle {
        id:                         headingIndicator
        visible:                    false
        height:                     ScreenTools.defaultFontPixelHeight
        width:                      ScreenTools.defaultFontPixelWidth * 4
        color:                      _falconMint
        radius:                     3
        border.color:               Qt.lighter(_falconMint, 1.18)
        border.width:               1
        anchors.top:                compassBar.top
        anchors.topMargin:          -headingIndicator.height / 2
        anchors.horizontalCenter:   parent.horizontalCenter
        QGCLabel {
            text:                   _heading
            color:                  "white"
            font.pointSize:         ScreenTools.smallFontPointSize
            anchors.centerIn:       parent
        }
    }
    Image {
        id:                         compassArrowIndicator
        visible:                    false
        height:                     _indicatorsHeight
        width:                      height
        source:                     "/custom/img/compass_pointer.svg"
        fillMode:                   Image.PreserveAspectFit
        sourceSize.height:          height
        anchors.top:                compassBar.bottom
        anchors.topMargin:          -height / 2
        anchors.horizontalCenter:   parent.horizontalCenter
    }

    //-------------------------------------------------------------------------
    //-- Aircraft visualization placeholder for the future mission-progress pane.
    //   It mirrors the video pane directly below it in both width and height.
    Item {
        id:                     aircraftPanelBounds
        anchors.left:           parent.left
        anchors.right:          parent.right
        anchors.bottom:         parent.bottom
        height:                 Math.min(lowerPanelHeight, parent.height)
    }

    Item {
        id:                     aircraftFlightPanelContainer
        parent:                 aircraftPanelBounds
        anchors.fill:           parent

        Rectangle {
            id:                     controlSurfacePanel
            anchors.fill:           parent
            radius:                 FalconTheme.radiusPanel
            color:                  FalconTheme.surface1
            border.width:           0
            clip:                   true

            // Left accent bar: marks the panel this phase is about, giving
            // hierarchy without enclosing anything in another box.
            Rectangle {
                anchors.left:       parent.left
                anchors.top:        parent.top
                anchors.bottom:     parent.bottom
                width:              FalconTheme.activeBarWidth
                color:              FalconTheme.accent
                z:                  10
                visible:            root.lowerPanelIsFocus
            }


            readonly property bool route3DReady: root.use3DRouteView
                                                 && route3DLoader.status === Loader.Ready
                                                 && route3DLoader.item !== null
            readonly property bool route3DActive: route3DReady
                                                  && (root.force3DVisibleForDebug
                                                      || route3DLoader.item.routeDataValid)

            function logRoute3DDebugState(reason) {
                const loaderHasItem = route3DLoader.item !== null
                const routeDataValid = loaderHasItem
                        ? route3DLoader.item.routeDataValid
                        : false
                console.log("[MissionRoute3D Debug]", reason,
                            "use3DRouteView:", root.use3DRouteView,
                            "force3DVisibleForDebug:", root.force3DVisibleForDebug,
                            "loader.status:", route3DLoader.status,
                            "loader.source:", route3DLoader.source,
                            "loader.item:", loaderHasItem,
                            "item.routeDataValid:", routeDataValid,
                            "3D view visible:", route3DLoader.visible,
                            "MissionRouteView.visible:", missionRouteView.visible,
                            "2D fallback visible:", !controlSurfacePanel.route3DActive,
                            "loader.opacity:", route3DLoader.opacity,
                            "loader.z:", route3DLoader.z,
                            "loader size:", route3DLoader.width, "x", route3DLoader.height)
            }

            // During pre-flight the mission route has nothing to report, so the
            // pane shows the pre-arm/sensor status instead. It hands the pane
            // back to the route views as soon as phase 0 completes.
            readonly property bool preflightFocus: root.missionPhase === 0

            MissionRouteView {
                id:                missionRouteView
                anchors.fill:      parent
                anchors.margins:   _toolsMargin
                z:                 0
                visible:           !controlSurfacePanel.preflightFocus
                // Keep the info table, drop the 2D route drawing whenever the
                // (transparent) 3D view is covering that same area.
                routeAreaVisible:  !controlSurfacePanel.route3DActive
                missionController: root._missionController
                activeVehicle:     root._activeVehicle
                missionAvailable:  root._activeVehicle ? true : false
            }

            PreflightStatusPanel {
                anchors.fill:    parent
                anchors.margins: _toolsMargin
                z:               2
                visible:         controlSurfacePanel.preflightFocus
                vehicle:         root._activeVehicle
            }

            Loader {
                id:      route3DLoader
                x:       missionRouteView.x
                y:       missionRouteView.y
                width:   missionRouteView.routeAreaWidth
                height:  missionRouteView.height
                active:  root.use3DRouteView
                visible: controlSurfacePanel.route3DActive && !controlSurfacePanel.preflightFocus
                opacity: 1
                z:       1
                source:  active ? "qrc:/qml/Custom/Widgets/MissionRoute3DView.qml" : ""

                onStatusChanged: {
                    console.log("[MissionRoute3D Debug] Loader status changed:",
                                status,
                                "source:", source,
                                "item:", item !== null)
                    if (status === Loader.Error) {
                        console.error("[MissionRoute3D Debug] Loader.Error for",
                                      source,
                                      "- check the adjacent QML error in Application Output")
                    }
                    Qt.callLater(function() {
                        controlSurfacePanel.logRoute3DDebugState("Loader.onStatusChanged")
                    })
                }

                onLoaded: {
                    item.missionOccurrences = Qt.binding(function() {
                        return missionRouteView.missionOccurrences
                    })
                    item.physicalWaypointGroups = Qt.binding(function() {
                        return missionRouteView.physicalWaypointGroups
                    })
                    item.traversalOccurrences = Qt.binding(function() {
                        return missionRouteView.traversalOccurrences
                    })
                    item.missionProgressRevision = Qt.binding(function() {
                        return missionRouteView.missionProgressRevision
                    })
                    item.showDebugSequenceNumbers = Qt.binding(function() {
                        return missionRouteView.showDebugSequenceNumbers
                    })
                    item.activeVehicle = Qt.binding(function() {
                        return root._activeVehicle
                    })
                    item.missionController = Qt.binding(function() {
                        return root._missionController
                    })
                    item.missionAvailable = Qt.binding(function() {
                        return root._activeVehicle ? true : false
                    })
                    item.activeWaypointIndex = Qt.binding(function() {
                        return missionRouteView.activeWaypointIndex
                    })
                    item.currentLegIndex = Qt.binding(function() {
                        return missionRouteView.currentLegIndex
                    })
                    item.currentLegProgress = Qt.binding(function() {
                        return missionRouteView.currentLegProgress
                    })
                    item.waypointStates = Qt.binding(function() {
                        return missionRouteView.waypointStates
                    })
                    Qt.callLater(function() {
                        item.logDebugState("Loader.onLoaded")
                        controlSurfacePanel.logRoute3DDebugState("Loader.onLoaded")
                    })
                }
            }

            Connections {
                target: route3DLoader.item
                ignoreUnknownSignals: true

                function onRouteDataValidChanged() {
                    Qt.callLater(function() {
                        controlSurfacePanel.logRoute3DDebugState(
                                    "MissionRoute3DView.routeDataValid changed")
                    })
                }
            }

            onRoute3DActiveChanged: {
                Qt.callLater(function() {
                    controlSurfacePanel.logRoute3DDebugState("route3DActive changed")
                })
            }

            Component.onCompleted: {
                Qt.callLater(function() {
                    controlSurfacePanel.logRoute3DDebugState(
                                "controlSurfacePanel completed")
                })
            }

            Item {
                id: aircraftImageLayers

                readonly property real layerSize: Math.min(56, missionRouteView.routeAreaWidth * 0.12)
                readonly property real routeLeft: missionRouteView.x
                readonly property real routeRight: routeLeft + missionRouteView.routeAreaWidth
                readonly property real routePointX: routeLeft + missionRouteView.aircraftPoint.x
                readonly property real routePointY: missionRouteView.y + missionRouteView.aircraftPoint.y
                readonly property real routeClearance: Math.max(_toolsMargin,
                                                                missionRouteView.routeAreaWidth * 0.015)
                readonly property bool placeRightOfRoute: routePointX + routeClearance + width
                                                          <= routeRight - _toolsMargin

                visible: !controlSurfacePanel.route3DActive
                         && !controlSurfacePanel.preflightFocus
                         && missionRouteView.hasMissionData
                         && missionRouteView.routeDataValid
                width:   layerSize
                height:  layerSize
                x:       Math.max(routeLeft + _toolsMargin,
                                  Math.min(routeRight - width - _toolsMargin,
                                           placeRightOfRoute
                                           ? routePointX + routeClearance
                                           : routePointX - width - routeClearance))
                y:       Math.max(_toolsMargin,
                                  Math.min(parent.height - height - _toolsMargin,
                                           routePointY - (height / 2)))

                Behavior on x {
                    NumberAnimation {
                        duration:    150
                        easing.type: Easing.OutQuad
                    }
                }

                Behavior on y {
                    NumberAnimation {
                        duration:    150
                        easing.type: Easing.OutQuad
                    }
                }

                Image {
                    anchors.fill:        parent
                    source:              "/custom/img/aircraft_iso.png"
                    fillMode:            Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment:   Image.AlignVCenter
                    mipmap:              true
                }

                Image {
                    anchors.fill:        parent
                    source:              "/custom/img/lift_prop_fl.png"
                    fillMode:            Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment:   Image.AlignVCenter
                    mipmap:              true
                }

                Image {
                    anchors.fill:        parent
                    source:              "/custom/img/lift_prop_fr.png"
                    fillMode:            Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment:   Image.AlignVCenter
                    mipmap:              true
                }

                Image {
                    anchors.fill:        parent
                    source:              "/custom/img/lift_prop_rl.png"
                    fillMode:            Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment:   Image.AlignVCenter
                    mipmap:              true
                }

                Image {
                    anchors.fill:        parent
                    source:              "/custom/img/lift_prop_rr.png"
                    fillMode:            Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment:   Image.AlignVCenter
                    mipmap:              true
                }

                Image {
                    anchors.fill:        parent
                    source:              "/custom/img/pusher_prop.png"
                    fillMode:            Image.PreserveAspectFit
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment:   Image.AlignVCenter
                    mipmap:              true
                }
            }
        }
    }

    Item {
        id:                     headingFlightPanel
        x:                      consolePanelWidth + paneSpacing
        y:                      0
        width:                  Math.max(0, root.width - consolePanelWidth - paneSpacing)
        height:                 topPanelHeight

        Rectangle {
            anchors.fill:       parent
            radius:             FalconTheme.radiusPanel
            color:              FalconTheme.surface1
            border.width:       0
        }

        Item {
                id:                     flightInfoSection
                anchors.fill:           parent

                readonly property real _contentMargin:      clamp(Math.min(width, height) * 0.045, 8, 18)
                readonly property real _infoFontPixelSize:  clamp(width * 0.045, 12, 20)
                readonly property real _labelFontPixelSize: clamp(width * 0.034, 10, 16)
                readonly property real _compassSize:        Math.min(width * 0.55, height * 0.60)

                CustomAttitudeWidget {
                    id:                 attitudeDial
                    anchors.horizontalCenter:   parent.horizontalCenter
                    anchors.verticalCenter:     parent.verticalCenter
                    // Lift the dial so the numeric readout below it stays inside
                    // the panel instead of overlapping the bottom telemetry.
                    anchors.verticalCenterOffset: -attitudeReadout.height * 0.55
                    size:               flightInfoSection._compassSize
                    vehicle:            _activeVehicle
                    showHeading:        true
                }

                // Numeric bank/pitch readout. The artificial horizon shows the
                // attitude, but not *how much* the airframe is tilted — during
                // cruise and transition that number is what the operator reads.
                Row {
                    id:                         attitudeReadout
                    anchors.horizontalCenter:   parent.horizontalCenter
                    anchors.top:                attitudeDial.bottom
                    anchors.topMargin:          flightInfoSection._contentMargin * 0.5
                    spacing:                    flightInfoSection._contentMargin * 1.4

                    AngleReadout {
                        label:              qsTr("ROLL")
                        angle:              _activeVehicle ? _activeVehicle.roll.rawValue : 0
                        warnAngle:          35
                        labelFontPixelSize: flightInfoSection._labelFontPixelSize
                        valueFontPixelSize: flightInfoSection._infoFontPixelSize
                    }

                    AngleReadout {
                        label:              qsTr("PITCH")
                        angle:              _activeVehicle ? _activeVehicle.pitch.rawValue : 0
                        warnAngle:          25
                        labelFontPixelSize: flightInfoSection._labelFontPixelSize
                        valueFontPixelSize: flightInfoSection._infoFontPixelSize
                    }

                    AngleReadout {
                        label:              qsTr("YAW")
                        angle:              _activeVehicle ? _activeVehicle.heading.rawValue : 0
                        wrap360:            true
                        labelFontPixelSize: flightInfoSection._labelFontPixelSize
                        valueFontPixelSize: flightInfoSection._infoFontPixelSize
                    }
                }

                TelemetryCorner {
                    anchors.left:           parent.left
                    anchors.top:            parent.top
                    anchors.margins:        flightInfoSection._contentMargin
                    labelFontPixelSize:     flightInfoSection._labelFontPixelSize
                    valueFontPixelSize:     flightInfoSection._infoFontPixelSize
                    values: [
                        { label: qsTr("ALT"), fact: _activeVehicle ? _activeVehicle.altitudeRelative : null, showUnits: true }
                    ]
                }

                TelemetryCorner {
                    anchors.right:          parent.right
                    anchors.top:            parent.top
                    anchors.margins:        flightInfoSection._contentMargin
                    labelFontPixelSize:     flightInfoSection._labelFontPixelSize
                    valueFontPixelSize:     flightInfoSection._infoFontPixelSize
                    values: [
                        { label: qsTr("CLIMB"), fact: _activeVehicle ? _activeVehicle.climbRate : null,   showUnits: true },
                        { label: qsTr("GND"),   fact: _activeVehicle ? _activeVehicle.groundSpeed : null, showUnits: true }
                    ]
                }

                TelemetryCorner {
                    anchors.left:           parent.left
                    anchors.bottom:         parent.bottom
                    anchors.margins:        flightInfoSection._contentMargin
                    labelFontPixelSize:     flightInfoSection._labelFontPixelSize
                    valueFontPixelSize:     flightInfoSection._infoFontPixelSize
                    values: [
                        { label: qsTr("AIR"), fact: _activeVehicle ? _activeVehicle.airSpeed : null,    showUnits: true },
                        { label: qsTr("THR"), fact: _activeVehicle ? _activeVehicle.throttlePct : null, showUnits: true }
                    ]
                }

                TelemetryCorner {
                    anchors.right:          parent.right
                    anchors.bottom:         parent.bottom
                    anchors.margins:        flightInfoSection._contentMargin
                    labelFontPixelSize:     flightInfoSection._labelFontPixelSize
                    valueFontPixelSize:     flightInfoSection._infoFontPixelSize
                    values: [
                        { label: qsTr("TIME"), fact: _activeVehicle ? _activeVehicle.flightTime : null, showUnits: false }
                    ]
                }
        }
    }

}
