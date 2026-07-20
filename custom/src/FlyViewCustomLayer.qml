import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls
import Custom.Widgets

Item {
    property var parentToolInsets                       // These insets tell you what screen real estate is available for positioning the controls in your overlay
    property var totalToolInsets:   _totalToolInsets    // The insets updated for the custom overlay additions
    property var mapControl

    readonly property string noGPS:         qsTr("NO GPS")
    readonly property real   indicatorValueWidth:   ScreenTools.defaultFontPixelWidth * 7
    readonly property bool   aircraftCompactMode: width < 1400 || height < 800
    readonly property bool   aircraftLargeMode: width >= 2200 && height >= 1200
    readonly property real   aircraftPanelResponsiveWidth: aircraftLargeMode
                                                               ? clamp(width * 0.16, 420, 560)
                                                               : aircraftCompactMode
                                                                   ? clamp(width * 0.20, 220, 280)
                                                                   : clamp(width * 0.18, 300, 420)
    readonly property real   aircraftPanelResponsiveHeight: Math.min(
                                                                       aircraftPanelResponsiveWidth * 1.05,
                                                                       height * (aircraftCompactMode ? 0.38 : 0.46)
                                                                   )
    readonly property real   aircraftTitlePixelSize: clamp(
                                                        aircraftPanelResponsiveWidth * 0.045,
                                                        aircraftCompactMode ? 12 : 14,
                                                        aircraftLargeMode ? 22 : 19
                                                    )
    readonly property real   aircraftReadoutPixelSize: clamp(
                                                          aircraftPanelResponsiveWidth * 0.035,
                                                          aircraftCompactMode ? 10 : 12,
                                                          aircraftLargeMode ? 18 : 15
                                                      )

    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
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

    component TelemetryCorner: Column {
        property var  values: []
        property real labelFontPixelSize: ScreenTools.defaultFontPixelHeight * 0.6
        property real valueFontPixelSize: ScreenTools.defaultFontPixelHeight * 0.72

        spacing: ScreenTools.defaultFontPixelHeight * 0.1

        Repeater {
            model: values

            Row {
                spacing: ScreenTools.defaultFontPixelWidth * 0.35

                QGCLabel {
                    anchors.baseline: valueLabel.baseline
                    text:             modelData.label
                    color:            _falconMint
                    font.bold:        false
                    font.pixelSize:   labelFontPixelSize
                }

                QGCLabel {
                    id:               valueLabel
                    text:             telemetryValue(modelData.fact, modelData.showUnits)
                    color:            "white"
                    font.bold:        false
                    font.pixelSize:   valueFontPixelSize
                }
            }
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
    MovablePanel {
        id:                         missionHeader
        property bool expanded:     false
        readonly property real collapsedHeight: ScreenTools.defaultFontPixelHeight * 4.4
        readonly property real expandedHeight:  ScreenTools.defaultFontPixelHeight * 26

        x:                          parentToolInsets.leftEdgeTopInset + _toolsMargin
        y:                          parentToolInsets.topEdgeLeftInset + _toolsMargin
        width:                      ScreenTools.defaultFontPixelWidth * 38
        height:                     collapsedHeight
        minWidth:                   ScreenTools.defaultFontPixelWidth * 24
        minHeight:                  ScreenTools.defaultFontPixelHeight * 3.4

        // Toggling sets height imperatively (breaks the initial binding), so a
        // later user resize still sticks.
        onExpandedChanged:          height = expanded ? expandedHeight : collapsedHeight

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
    //-- Draggable aircraft and flight-information panel
    //   Uses live SERVO_OUTPUT_RAW values when connected and sits directly
    //   above the bottom-right Fly View bar.
    Item {
        id:                     aircraftPanelBounds
        x:                      0
        y:                      parentToolInsets.topEdgeRightInset + _toolsMargin
        width:                  parent ? parent.width : 0
        height:                 parent ? Math.max(0, parent.height - parentToolInsets.bottomEdgeRightInset - y) : 0

        onWidthChanged: {
            if (aircraftFlightPanelContainer) {
                aircraftFlightPanelContainer._adjustPositionForGeometry()
            }
        }
        onHeightChanged: {
            if (aircraftFlightPanelContainer) {
                aircraftFlightPanelContainer._adjustPositionForGeometry()
            }
        }
    }

    MovablePanel {
        id:                     aircraftFlightPanelContainer
        parent:                 aircraftPanelBounds
        x:                      0
        y:                      0
        width:                  aircraftPanelResponsiveWidth
        height:                 _sectionHeight * 2 + _dividerHeight
        movable:                true
        resizable:              false
        minWidth:               0
        minHeight:              0

        readonly property real _dividerHeight: 1
        readonly property real _sectionHeight: Math.max(0, Math.min(
                                                             aircraftPanelResponsiveHeight,
                                                             ((parent ? parent.height : 0) - _dividerHeight) / 2
                                                         ))
        property bool _positionInitialized: false
        property bool _hasCustomPosition:   false
        property bool _updatingGeometry:    false

        function _defaultX() {
            const maximumX = parent ? Math.max(0, parent.width - width) : 0
            return clamp((parent ? parent.width : 0) - width - _toolsMargin, 0, maximumX)
        }

        function _defaultY() {
            return parent ? Math.max(0, parent.height - height) : 0
        }

        function _resetPosition() {
            _hasCustomPosition = false
            _updatingGeometry = true
            x = _defaultX()
            y = _defaultY()
            _updatingGeometry = false
        }

        function _adjustPositionForGeometry() {
            if (!_positionInitialized || !parent) {
                return
            }

            _updatingGeometry = true
            width = Math.min(aircraftPanelResponsiveWidth, parent.width)
            height = _sectionHeight * 2 + _dividerHeight
            if (_hasCustomPosition) {
                clampToParent()
            } else {
                x = _defaultX()
                y = _defaultY()
            }
            _updatingGeometry = false
        }

        onXChanged: {
            if (_positionInitialized && !_updatingGeometry) {
                _hasCustomPosition = true
            }
        }
        onYChanged: {
            if (_positionInitialized && !_updatingGeometry) {
                _hasCustomPosition = true
            }
        }
        Component.onCompleted: {
            _positionInitialized = true
            _adjustPositionForGeometry()
        }

        Rectangle {
            id:                     controlSurfacePanel
            anchors.fill:           parent
            radius:                 6
            color:                  Qt.rgba(0.03, 0.08, 0.14, 0.92)
            border.color:           Qt.rgba(0.34, 0.59, 0.71, 0.70)
            border.width:           1
            clip:                   true

            property bool motorDemoEnabled:          false
            property bool surfaceOutputUsesPwm:      false
            property int  actuatorDataTimeoutMs:     1000
            readonly property real embeddedAttitudeSize: clamp(
                                                                  Math.min(
                                                                      aircraftSection.width * 0.24,
                                                                      aircraftSection.height * 0.28
                                                                  ),
                                                                  70,
                                                                  130
                                                              )
            readonly property real embeddedAttitudeMargin: clamp(
                                                                    Math.min(aircraftSection.width, aircraftSection.height) * 0.025,
                                                                    6,
                                                                    12
                                                                )
            property var  _servo:                    []
            property bool _actuatorDataFresh:        false
            property double _lastActuatorDataMs:     0
            readonly property var  _vehicle:         _activeVehicle
            readonly property bool _haveServo:       _servo.length >= 8 && _servo[0] > 0
            readonly property bool _haveMotorOutputs: _channelValid(0) || _channelValid(1) || _channelValid(2)
                                                       || _channelValid(3) || _channelValid(4)
            readonly property bool _vehicleArmed:    !!(_vehicle && _vehicle.armed)
            readonly property bool _communicationAvailable: !!(_vehicle && _vehicle.vehicleLinkManager
                                                                 && !_vehicle.vehicleLinkManager.communicationLost)
            readonly property bool _actuatorDataValid: _vehicleArmed && _communicationAvailable
                                                        && _actuatorDataFresh && _haveMotorOutputs

            on_VehicleChanged: {
                _actuatorDataFresh = false
                _lastActuatorDataMs = 0
            }

            Connections {
                target: controlSurfacePanel._vehicle
                ignoreUnknownSignals: true
                function onServoOutputsChanged(servoValues) { controlSurfacePanel._updateActuatorData(servoValues) }
            }

            // Temporary SITL motor mapping: ch1-4 lift motors, ch5 pusher.
            // The FL/FR/RL/RR order for ch1-4 is provisional until roll/pitch
            // output changes and the PX4 actuator configuration are verified.
            // TODO: Before real-aircraft deployment, verify these output functions and
            // evaluate ACTUATOR_OUTPUT_STATUS; this panel currently uses SERVO_OUTPUT_RAW.
            function _channelValid(ch) { return _servo.length > ch && _servo[ch] > 0 }
            function _surfaceChannelValid(ch) {
                return !!_vehicle && _communicationAvailable && _actuatorDataFresh
                        && _servo.length > ch && isFinite(_servo[ch]) && _servo[ch] !== -1
            }
            function _surf(ch) { return _haveServo && _servo[ch] > 0 ? (_servo[ch] - 1500) / 500 : 0 }

            function normalizeMotorOutput(rawValue) {
                if (!isFinite(rawValue) || rawValue <= 0) {
                    return 0.0
                }

                if (rawValue < 1000) {
                    return clamp(rawValue / 1000.0, 0.0, 1.0)
                }

                return clamp((rawValue - 1000.0) / 1000.0, 0.0, 1.0)
            }

            function normalizeSurfaceOutput(rawValue) {
                if (!isFinite(rawValue)) {
                    return 0.0
                }

                if (surfaceOutputUsesPwm) {
                    return clamp((rawValue - 1500.0) / 500.0, -1.0, 1.0)
                }

                return clamp(rawValue / 1000.0, -1.0, 1.0)
            }

            function _mot(ch) { return _channelValid(ch) ? normalizeMotorOutput(_servo[ch]) : 0.0 }

            function _surface(ch) { return _surfaceChannelValid(ch) ? normalizeSurfaceOutput(_servo[ch]) : 0.0 }

            function _updateActuatorData(servoValues) {
                _servo = servoValues
                _lastActuatorDataMs = Date.now()
                _actuatorDataFresh = true
                actuatorDataTimeoutTimer.restart()
            }

            Timer {
                id:         actuatorDataTimeoutTimer
                interval:   controlSurfacePanel.actuatorDataTimeoutMs
                repeat:     false
                onTriggered: controlSurfacePanel._actuatorDataFresh = false
            }

            // Keep the existing control-surface mock driver. Synthetic motor
            // values are used only when motorDemoEnabled is explicitly enabled.
            property real _t: 0
            NumberAnimation on _t {
                from: 0; to: Math.PI * 2; duration: 4000
                loops: Animation.Infinite; running: !controlSurfacePanel._haveServo || controlSurfacePanel.motorDemoEnabled
            }
            property bool _demoFwd: true
            Timer {
                interval: 6000; running: !controlSurfacePanel._haveServo; repeat: true
                onTriggered: controlSurfacePanel._demoFwd = !controlSurfacePanel._demoFwd
            }
            // Use the connected vehicle mode when available, otherwise the demo toggle.
            property bool _fwdMode: _activeVehicle
                                    ? (_activeVehicle.vtol ? _activeVehicle.vtolInFwdFlight : _activeVehicle.fixedWing)
                                    : _demoFwd
            readonly property real _elevatorValue: _haveServo ? _surf(1) : Math.sin(_t * 0.7)
            readonly property real _rudderValue:   _haveServo ? _surf(3) : Math.sin(_t * 0.5)

            Item {
                id:                 aircraftSection
                anchors.top:        parent.top
                anchors.left:       parent.left
                anchors.right:      parent.right
                height:             aircraftFlightPanelContainer._sectionHeight
            }

            Item {
                id:                     aircraftTitleArea
                parent:                 aircraftSection
                anchors.top:            parent.top
                anchors.left:           parent.left
                anchors.right:          parent.right
                height:                 controlSurfaceTitle.implicitHeight + _toolsMargin

                Row {
                    anchors.top:              parent.top
                    anchors.topMargin:        _toolsMargin * 0.5
                    anchors.horizontalCenter: parent.horizontalCenter
                    spacing:                  _toolsMargin

                    QGCLabel {
                        id:             controlSurfaceTitle
                        text:           qsTr("Aircraft  ·  ") + (controlSurfacePanel._fwdMode ? qsTr("FORWARD") : qsTr("HOVER"))
                        color:          "white"
                        font.pixelSize: aircraftTitlePixelSize
                    }

                    QGCLabel {
                        anchors.verticalCenter: parent.verticalCenter
                        text:                   controlSurfacePanel.motorDemoEnabled
                                                    ? qsTr("DEMO")
                                                    : controlSurfacePanel._actuatorDataValid ? qsTr("LIVE") : qsTr("NO MOTOR DATA")
                        color:                  controlSurfacePanel.motorDemoEnabled
                                                    ? "#FCD34D"
                                                    : controlSurfacePanel._actuatorDataValid ? "#86EFAC" : _falconMint
                        font.bold:              true
                        font.pixelSize:         Math.max(8, aircraftReadoutPixelSize * 0.65)
                    }
                }

                HoverHandler {
                    cursorShape: Qt.SizeAllCursor
                }

                TapHandler {
                    acceptedButtons: Qt.LeftButton
                    onDoubleTapped: aircraftFlightPanelContainer._resetPosition()
                }
            }

            Item {
                id:                     aircraftContentArea
                parent:                 aircraftSection
                anchors.top:            aircraftTitleArea.bottom
                anchors.bottom:         parent.bottom
                anchors.left:           parent.left
                anchors.right:          parent.right
                anchors.leftMargin:     6
                anchors.rightMargin:    6
                clip:                   true

                ControlSurfaceWidget {
                    id:             controlSurfaceWidget
                    anchors.fill:   parent

                    fixedWingMode:      controlSurfacePanel._fwdMode
                    actuatorDataValid:  controlSurfacePanel.motorDemoEnabled || controlSurfacePanel._actuatorDataValid
                    vehicleArmed:       controlSurfacePanel.motorDemoEnabled || controlSurfacePanel._vehicleArmed

                    // Temporary SITL surface mapping. Verify S6/S7 independently with
                    // roll input and the PX4 actuator configuration before finalizing it.
                    aileronLeftDeflection:  controlSurfacePanel._surface(5)
                    aileronRightDeflection: controlSurfacePanel._surface(6)
                    elevatorDeflection:     controlSurfacePanel._elevatorValue
                    rudderDeflection:       controlSurfacePanel._rudderValue

                    // Temporary UI/simulation mix. Replace these bindings with the
                    // actual left/right Ruddervator servo output channels on the aircraft.
                    ruddervatorLeftDeflection:  clamp(controlSurfacePanel._elevatorValue + controlSurfacePanel._rudderValue, -1, 1)
                    ruddervatorRightDeflection: clamp(controlSurfacePanel._elevatorValue - controlSurfacePanel._rudderValue, -1, 1)

                    liftThrottleFL: controlSurfacePanel.motorDemoEnabled
                                        ? 0.55 + 0.15 * Math.sin(controlSurfacePanel._t * 2)
                                        : controlSurfacePanel._actuatorDataValid ? controlSurfacePanel._mot(0) : 0
                    liftThrottleFR: controlSurfacePanel.motorDemoEnabled
                                        ? 0.55 + 0.15 * Math.sin(controlSurfacePanel._t * 2 + 1)
                                        : controlSurfacePanel._actuatorDataValid ? controlSurfacePanel._mot(1) : 0
                    liftThrottleRL: controlSurfacePanel.motorDemoEnabled
                                        ? 0.55 + 0.15 * Math.sin(controlSurfacePanel._t * 2 + 2)
                                        : controlSurfacePanel._actuatorDataValid ? controlSurfacePanel._mot(2) : 0
                    liftThrottleRR: controlSurfacePanel.motorDemoEnabled
                                        ? 0.55 + 0.15 * Math.sin(controlSurfacePanel._t * 2 + 3)
                                        : controlSurfacePanel._actuatorDataValid ? controlSurfacePanel._mot(3) : 0
                    pusherThrottle: controlSurfacePanel.motorDemoEnabled
                                        ? 0.6
                                        : controlSurfacePanel._actuatorDataValid ? controlSurfacePanel._mot(4) : 0
                }

            }

            Item {
                id:                     embeddedAttitudeIndicator
                parent:                 aircraftSection
                anchors.right:          aircraftSection.right
                anchors.bottom:         aircraftSection.bottom
                anchors.rightMargin:    controlSurfacePanel.embeddedAttitudeMargin
                anchors.bottomMargin:   controlSurfacePanel.embeddedAttitudeMargin
                width:                  controlSurfacePanel.embeddedAttitudeSize
                height:                 width
                z:                      10

                CustomAttitudeWidget {
                    anchors.centerIn:   parent
                    size:               parent.width
                    vehicle:            _activeVehicle
                    showHeading:        false
                }
            }

            Rectangle {
                id:                     sectionDivider
                anchors.top:            aircraftSection.bottom
                anchors.left:           parent.left
                anchors.right:          parent.right
                height:                 aircraftFlightPanelContainer._dividerHeight
                color:                  Qt.rgba(0.34, 0.59, 0.71, 0.70)
            }

            Item {
                id:                     flightInfoSection
                anchors.top:            sectionDivider.bottom
                anchors.left:           parent.left
                anchors.right:          parent.right
                height:                 aircraftFlightPanelContainer._sectionHeight

                readonly property real _contentMargin:      clamp(Math.min(width, height) * 0.045, 8, 18)
                readonly property real _infoFontPixelSize:  clamp(width * 0.045, 12, 20)
                readonly property real _labelFontPixelSize: clamp(width * 0.034, 10, 16)
                readonly property real _compassSize:        Math.min(width * 0.55, height * 0.60)

                Rectangle {
                    id:                     compassBezel
                    anchors.centerIn:       parent
                    width:                  flightInfoSection._compassSize
                    height:                 width
                    radius:                 height / 2
                    border.color:           _falconMint
                    border.width:           1
                    color:                  "transparent"
                }

                Rectangle {
                    id:                         northLabelBackground
                    anchors.top:                compassBezel.top
                    anchors.topMargin:          -height / 2
                    anchors.horizontalCenter:   compassBezel.horizontalCenter
                    width:                      northLabel.contentWidth * 1.5
                    height:                     northLabel.contentHeight * 1.5
                    radius:                     3
                    color:                      _falconPanel

                    QGCLabel {
                        id:                 northLabel
                        anchors.centerIn:   parent
                        text:               "N"
                        color:              "white"
                        font.pixelSize:     clamp(compassBezel.width * 0.09, 11, 20)
                    }
                }

                Image {
                    id:                 headingNeedle
                    anchors.centerIn:   compassBezel
                    width:              compassBezel.width * 0.75
                    height:             width
                    source:             "/custom/img/falcon_tailsitter.svg"
                    fillMode:           Image.PreserveAspectFit
                    mipmap:             true
                    transform: [
                        Rotation {
                            origin.x:   headingNeedle.width / 2
                            origin.y:   headingNeedle.height / 2
                            angle:      _heading
                        }
                    ]
                }

                Rectangle {
                    id:                         headingLabelBackground
                    anchors.top:                compassBezel.bottom
                    anchors.topMargin:          -height / 2
                    anchors.horizontalCenter:   compassBezel.horizontalCenter
                    width:                      headingLabel.contentWidth * 1.5
                    height:                     headingLabel.contentHeight * 1.5
                    radius:                     3
                    color:                      _falconPanel

                    QGCLabel {
                        id:                 headingLabel
                        anchors.centerIn:   parent
                        text:               _heading
                        color:              "white"
                        font.pixelSize:     clamp(compassBezel.width * 0.09, 11, 20)
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

    //-------------------------------------------------------------------------
    //-- Recognition video panel (VTOL-GCS, ROS build only)
    //   Loaded by URL and gated on `customRosEnabled` (set from CustomPlugin)
    //   so non-ROS builds never try to import Custom.Ros.
    //   Drag to move, grab a corner to resize.
    MovablePanel {
        id:                     rosVideoFrame
        visible:                (typeof customRosEnabled !== 'undefined') && customRosEnabled
        x:                      _toolsMargin
        y:                      (parent ? parent.height : 0) - height - _toolsMargin - parentToolInsets.bottomEdgeLeftInset
        width:                  ScreenTools.defaultFontPixelWidth * 34
        height:                 ScreenTools.defaultFontPixelHeight * 15
        minWidth:               ScreenTools.defaultFontPixelWidth * 22
        minHeight:              ScreenTools.defaultFontPixelHeight * 10

        Loader {
            id:                 rosVideoLoader
            anchors.fill:       parent
            active:             rosVideoFrame.visible
            source:             active ? "qrc:/qml/Custom/Widgets/RosVideoPanel.qml" : ""
        }
    }
}
