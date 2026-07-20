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
        property var values: []

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
                    font.pointSize:   ScreenTools.smallFontPointSize * 0.85
                }

                QGCLabel {
                    id:               valueLabel
                    text:             telemetryValue(modelData.fact, modelData.showUnits)
                    color:            "white"
                    font.bold:        false
                    font.pointSize:   ScreenTools.smallFontPointSize
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
        bottomEdgeLeftInset:    parent.height - compassBackground.y
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

    Rectangle {
        id:                     compassBackground
        anchors.bottom:         parent.bottom
        anchors.bottomMargin:   0
        anchors.left:           parent.left
        anchors.leftMargin:     _toolsMargin + parentToolInsets.leftEdgeBottomInset
        width:                  ScreenTools.defaultFontPixelWidth * 18
        height:                 attitudeIndicator.height * 1.2
        radius:                 6
        color:                  Qt.rgba(0.03, 0.08, 0.14, 0.88)
        border.color:           Qt.rgba(0.34, 0.59, 0.71, 0.70)
        border.width:           1

        Rectangle {
            id:                     compassBezel
            anchors.centerIn:       parent
            width:                  height
            height:                 parent.height * 0.42
            radius:                 height / 2
            border.color:           _falconMint
            border.width:           1
            color:                  Qt.rgba(0,0,0,0)
        }

        Rectangle {
            id:                         northLabelBackground
            anchors.top:                compassBezel.top
            anchors.topMargin:          -height / 2
            anchors.horizontalCenter:   compassBezel.horizontalCenter
            width:                      northLabel.contentWidth * 1.5
            height:                     northLabel.contentHeight * 1.5
            radius:                     ScreenTools.defaultFontPixelWidth  * 0.25
            color:                      _falconPanel

            QGCLabel {
                id:                 northLabel
                anchors.centerIn:   parent
                text:               "N"
                color:              "white"
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }

        Image {
            id:                 headingNeedle
            anchors.centerIn:   compassBezel
            height:             compassBezel.height * 0.75
            width:              height
            source:             "/custom/img/falcon_tailsitter.svg"
            fillMode:           Image.PreserveAspectFit
            mipmap:             true
            transform: [
                Rotation {
                    origin.x:   headingNeedle.width  / 2
                    origin.y:   headingNeedle.height / 2
                    angle:      _heading
                }]
        }

        Rectangle {
            id:                         headingLabelBackground
            anchors.top:                compassBezel.bottom
            anchors.topMargin:          -height / 2
            anchors.horizontalCenter:   compassBezel.horizontalCenter
            width:                      headingLabel.contentWidth * 1.5
            height:                     headingLabel.contentHeight * 1.5
            radius:                     ScreenTools.defaultFontPixelWidth  * 0.25
            color:                      _falconPanel

            QGCLabel {
                id:                 headingLabel
                anchors.centerIn:   parent
                text:               _heading
                color:              "white"
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }

        TelemetryCorner {
            anchors.left:           parent.left
            anchors.top:            parent.top
            anchors.margins:        _toolsMargin
            values: [
                { label: qsTr("ALT"), fact: _activeVehicle ? _activeVehicle.altitudeRelative : null, showUnits: true }
            ]
        }

        TelemetryCorner {
            anchors.right:          parent.right
            anchors.top:            parent.top
            anchors.margins:        _toolsMargin
            values: [
                { label: qsTr("CLIMB"), fact: _activeVehicle ? _activeVehicle.climbRate : null,   showUnits: true },
                { label: qsTr("GND"),   fact: _activeVehicle ? _activeVehicle.groundSpeed : null, showUnits: true }
            ]
        }

        TelemetryCorner {
            anchors.left:           parent.left
            anchors.bottom:         parent.bottom
            anchors.margins:        _toolsMargin
            values: [
                { label: qsTr("AIR"), fact: _activeVehicle ? _activeVehicle.airSpeed : null,    showUnits: true },
                { label: qsTr("THR"), fact: _activeVehicle ? _activeVehicle.throttlePct : null, showUnits: true }
            ]
        }

        TelemetryCorner {
            anchors.right:          parent.right
            anchors.bottom:         parent.bottom
            anchors.margins:        _toolsMargin
            values: [
                { label: qsTr("TIME"), fact: _activeVehicle ? _activeVehicle.flightTime : null, showUnits: false }
            ]
        }
    }

    Rectangle {
        id:                     attitudeIndicator
        anchors.top:            compassArrowIndicator.bottom
        anchors.topMargin:      _toolsMargin
        anchors.horizontalCenter: parent.horizontalCenter
        height:                 ScreenTools.defaultFontPixelHeight * 6
        width:                  height
        radius:                 height * 0.5
        color:                  Qt.rgba(0.03, 0.08, 0.14, 0.88)
        border.color:           Qt.rgba(0.34, 0.59, 0.71, 0.70)
        border.width:           1

        CustomAttitudeWidget {
            size:               parent.height * 0.95
            vehicle:            _activeVehicle
            showHeading:        false
            anchors.centerIn:   parent
        }
    }

    //-------------------------------------------------------------------------
    //-- Aircraft control-surface panel (from dev/daehyeon)
    //   Uses live SERVO_OUTPUT_RAW values when connected and sits directly
    //   above the bottom-right Fly View bar.
    Rectangle {
        id:                     controlSurfacePanel
        anchors.bottom:         parent.bottom
        anchors.right:          parent.right
        anchors.bottomMargin:   parentToolInsets.bottomEdgeRightInset
        anchors.rightMargin:    _toolsMargin
        width:                  aircraftPanelResponsiveWidth
        height:                 aircraftPanelResponsiveHeight
        radius:                 6
        color:                  Qt.rgba(0.03, 0.08, 0.14, 0.92)
        border.color:           Qt.rgba(0.34, 0.59, 0.71, 0.70)
        border.width:           1

        property var  _servo:     []
        property bool _haveServo: _servo.length >= 8 && _servo[0] > 0

        Connections {
            target: _activeVehicle
            ignoreUnknownSignals: true
            function onServoOutputsChanged(servoValues) { controlSurfacePanel._servo = servoValues }
        }

        // Channel mapping: ch1 aileron, ch2 elevator, ch3 pusher,
        // ch4 rudder, ch5-8 lift motors.
        function _surf(ch) { return _haveServo && _servo[ch] > 0 ? (_servo[ch] - 1500) / 500 : 0 }
        function _mot(ch)  { return _haveServo && _servo[ch] > 0 ? (_servo[ch] - 1000) / 1000 : 0 }

        property bool _fwdMode: _activeVehicle
                                ? (_activeVehicle.vtol ? _activeVehicle.vtolInFwdFlight : _activeVehicle.fixedWing)
                                : false

        Item {
            id:                     aircraftTitleArea
            anchors.top:            parent.top
            anchors.left:           parent.left
            anchors.right:          parent.right
            height:                 controlSurfaceTitle.implicitHeight + _toolsMargin

            QGCLabel {
                id:                         controlSurfaceTitle
                anchors.top:                parent.top
                anchors.topMargin:          _toolsMargin * 0.5
                anchors.horizontalCenter:   parent.horizontalCenter
                text:                       qsTr("Aircraft  ·  ") + (controlSurfacePanel._fwdMode ? qsTr("FORWARD") : qsTr("HOVER"))
                color:                      "white"
                font.pixelSize:             aircraftTitlePixelSize
            }
        }

        Item {
            id:                     aircraftContentArea
            anchors.top:            aircraftTitleArea.bottom
            anchors.bottom:         aircraftReadoutArea.top
            anchors.left:           parent.left
            anchors.right:          parent.right
            anchors.leftMargin:     6
            anchors.rightMargin:    6

            ControlSurfaceWidget {
                id:             controlSurfaceWidget
                anchors.fill:   parent

                fixedWingMode:      controlSurfacePanel._fwdMode

                aileronLeftDeflection:  controlSurfacePanel._surf(0)
                aileronRightDeflection: -controlSurfacePanel._surf(0)
                elevatorDeflection:     controlSurfacePanel._surf(1)
                rudderDeflection:       controlSurfacePanel._surf(3)

                liftThrottleFL: controlSurfacePanel._mot(4)
                liftThrottleFR: controlSurfacePanel._mot(5)
                liftThrottleRL: controlSurfacePanel._mot(6)
                liftThrottleRR: controlSurfacePanel._mot(7)
                pusherThrottle: controlSurfacePanel._mot(2)
            }
        }

        Item {
            id:                     aircraftReadoutArea
            anchors.left:           parent.left
            anchors.right:          parent.right
            anchors.bottom:         parent.bottom
            height:                 aircraftReadoutPixelSize * 1.8

            Row {
                anchors.centerIn: parent
                spacing: aircraftPanelResponsiveWidth * 0.06

                QGCLabel {
                    text: "L AIL " + (controlSurfaceWidget.aileronLeftDeflection >= 0 ? "+" : "") + controlSurfaceWidget.aileronLeftDeflection.toFixed(2)
                    color: controlSurfaceWidget.aileronLeftDeflection >= 0 ? controlSurfaceWidget.downColor : controlSurfaceWidget.upColor
                    font.pixelSize: aircraftReadoutPixelSize
                    font.bold: true
                }

                QGCLabel {
                    text: "R AIL " + (controlSurfaceWidget.aileronRightDeflection >= 0 ? "+" : "") + controlSurfaceWidget.aileronRightDeflection.toFixed(2)
                    color: controlSurfaceWidget.aileronRightDeflection >= 0 ? controlSurfaceWidget.downColor : controlSurfaceWidget.upColor
                    font.pixelSize: aircraftReadoutPixelSize
                    font.bold: true
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
