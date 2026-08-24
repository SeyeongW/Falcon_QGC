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
    readonly property color  _falconNavy:   "#071526"
    readonly property color  _falconPanel:  "#0B1D33"
    readonly property color  _falconCyan:   "#38BDF8"
    readonly property color  _falconBlue:   "#1D4ED8"
    readonly property color  _falconMint:   "#5796B4"

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
            radius:                 6
            color:                  Qt.rgba(0.03, 0.08, 0.14, 0.90)
            border.color:           Qt.rgba(0.22, 0.74, 0.97, 0.75)
            border.width:           1
            clip:                   true

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
                        color:                _activeVehicle ? "#86EFAC" : "#FCD34D"
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
                    active:             missionHeader.expanded && (typeof customRosEnabled !== 'undefined') && customRosEnabled
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
        color:                      Qt.rgba(0.03, 0.08, 0.14, 0.88)
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
            radius:                 6
            color:                  Qt.rgba(0.03, 0.08, 0.14, 0.92)
            border.color:           Qt.rgba(0.34, 0.59, 0.71, 0.70)
            border.width:           1
            clip:                   true

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

            MissionRouteView {
                id:                missionRouteView
                anchors.fill:      parent
                anchors.margins:   _toolsMargin
                z:                 0
                visible:           true
                missionController: root._missionController
                activeVehicle:     root._activeVehicle
                missionAvailable:  root._activeVehicle ? true : false
            }

            Loader {
                id:      route3DLoader
                x:       missionRouteView.x
                y:       missionRouteView.y
                width:   missionRouteView.routeAreaWidth
                height:  missionRouteView.height
                active:  root.use3DRouteView
                visible: controlSurfacePanel.route3DActive
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
            radius:             6
            color:              Qt.rgba(0.03, 0.08, 0.14, 0.92)
            border.color:       Qt.rgba(0.34, 0.59, 0.71, 0.70)
            border.width:       1
        }

        Item {
            id:                     flightInfoSection
            anchors.fill:           parent

            readonly property real _compassSize: Math.min(width * 0.76, height * 0.65)

            CustomAttitudeWidget {
                anchors.centerIn:            parent
                anchors.verticalCenterOffset: parent.height * 0.07
                size:                        flightInfoSection._compassSize
                vehicle:                     _activeVehicle
                showHeading:                 true
            }
        }
    }

}
