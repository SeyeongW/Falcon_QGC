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
        rightEdgeBottomInset:   parent.width - compassBackground.x
        topEdgeLeftInset:       parentToolInsets.topEdgeLeftInset
        topEdgeCenterInset:     compassArrowIndicator.y + compassArrowIndicator.height
        topEdgeRightInset:      parentToolInsets.topEdgeRightInset
        bottomEdgeLeftInset:    parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  parentToolInsets.bottomEdgeCenterInset
        bottomEdgeRightInset:   parent.height - attitudeIndicator.y
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
    //-- Heading Indicator
    Rectangle {
        id:                         compassBar
        height:                     ScreenTools.defaultFontPixelHeight * 1.5
        width:                      ScreenTools.defaultFontPixelWidth  * 50
        anchors.bottom:             parent.bottom
        anchors.bottomMargin:       _toolsMargin
        color:                      "#DEDEDE"
        radius:                     2
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
                color:          "#75505565"
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
        color:                      qgcPal.windowShadeDark
        anchors.top:                compassBar.top
        anchors.topMargin:          -headingIndicator.height / 2
        anchors.horizontalCenter:   parent.horizontalCenter
        QGCLabel {
            text:                   _heading
            color:                  qgcPal.text
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
        anchors.bottom:         attitudeIndicator.bottom
        anchors.right:          attitudeIndicator.left
        anchors.rightMargin:    -attitudeIndicator.width / 2
        width:                  -anchors.rightMargin + compassBezel.width + (_toolsMargin * 2)
        height:                 attitudeIndicator.height * 0.75
        radius:                 2
        color:                  qgcPal.window

        Rectangle {
            id:                     compassBezel
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin:     _toolsMargin
            anchors.left:           parent.left
            width:                  height
            height:                 parent.height - (northLabelBackground.height / 2) - (headingLabelBackground.height / 2)
            radius:                 height / 2
            border.color:           qgcPal.text
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
            color:                      qgcPal.windowShade

            QGCLabel {
                id:                 northLabel
                anchors.centerIn:   parent
                text:               "N"
                color:              qgcPal.text
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }

        Image {
            id:                 headingNeedle
            anchors.centerIn:   compassBezel
            height:             compassBezel.height * 0.75
            width:              height
            source:             "/custom/img/compass_needle.svg"
            fillMode:           Image.PreserveAspectFit
            sourceSize.height:  height
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
            color:                      qgcPal.windowShade

            QGCLabel {
                id:                 headingLabel
                anchors.centerIn:   parent
                text:               _heading
                color:              qgcPal.text
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }
    }

    Rectangle {
        id:                     attitudeIndicator
        anchors.bottomMargin:   _toolsMargin + parentToolInsets.bottomEdgeRightInset
        anchors.rightMargin:    _toolsMargin
        anchors.bottom:         parent.bottom
        anchors.right:          parent.right
        height:                 ScreenTools.defaultFontPixelHeight * 6
        width:                  height
        radius:                 height * 0.5
        color:                  qgcPal.windowShade

        CustomAttitudeWidget {
            size:               parent.height * 0.95
            vehicle:            _activeVehicle
            showHeading:        false
            anchors.centerIn:   parent
        }
    }

    //-------------------------------------------------------------------------
    //-- Aircraft status panel (VTOL-GCS)
    //   Top-down quadplane whose shape adapts to flight mode: hover shows the
    //   spinning lift rotors, forward flight shows the pusher + control surfaces.
    Rectangle {
        id:                     controlSurfacePanel
        anchors.top:            parent.top
        anchors.right:          parent.right
        anchors.topMargin:      parentToolInsets.topEdgeRightInset + _toolsMargin
        anchors.rightMargin:    _toolsMargin
        width:                  ScreenTools.defaultFontPixelWidth * 28
        height:                 width + controlSurfaceTitle.height + (_toolsMargin * 2)
        radius:                 4
        color:                  qgcPal.window
        opacity:                0.92

        // --- Live actuator source, priority: MAVROS RCOut (real aircraft
        //     behavior) > MAVLink SERVO_OUTPUT_RAW > nothing (static/neutral).
        //     No mock animation: with no data the aircraft stays still. ---
        property var  _rosSource: rosActuatorLoader.item
        property bool _haveRos:   _rosSource ? _rosSource.have : false
        property var  _rosChan:   (_rosSource && _rosSource.channels) ? _rosSource.channels : []

        property var  _servo:     []      // latest SERVO_OUTPUT_RAW (ch1..16, us)
        property bool _haveServo: _servo.length >= 8 && _servo[0] > 0

        Connections {
            target: _activeVehicle
            ignoreUnknownSignals: true
            function onServoOutputsChanged(servoValues) { controlSurfacePanel._servo = servoValues }
        }

        // Unified channel array ([0] = ch1); prefer MAVROS when it is live.
        property bool _haveLive: _haveRos || _haveServo
        property var  _chan:     _haveRos ? _rosChan : _servo

        // Channel -> normalized helpers (matches custom/tools/fake_mavlink.py and
        // the RCOut channel order): ch1 aileron, ch2 elevator, ch3 pusher,
        // ch4 rudder, ch5-8 lift motors. Returns 0 (neutral) when no live data.
        function _surf(ch) { return _haveLive && _chan[ch] > 0 ? (_chan[ch] - 1500) / 500  : 0 }
        function _mot(ch)  { return _haveLive && _chan[ch] > 0 ? (_chan[ch] - 1000) / 1000 : 0 }

        // Mode: real VTOL state when connected, otherwise hover (no fake toggle).
        property bool _fwdMode: (_activeVehicle && _activeVehicle.vtol)
                                ? _activeVehicle.vtolInFwdFlight
                                : false

        QGCLabel {
            id:                         controlSurfaceTitle
            anchors.top:                parent.top
            anchors.topMargin:          _toolsMargin * 0.5
            anchors.horizontalCenter:   parent.horizontalCenter
            text:                       qsTr("Aircraft  ·  ") + (controlSurfacePanel._fwdMode ? qsTr("FORWARD") : qsTr("HOVER"))
            color:                      qgcPal.text
            font.pointSize:             ScreenTools.smallFontPointSize
        }

        ControlSurfaceWidget {
            anchors.top:        controlSurfaceTitle.bottom
            anchors.left:       parent.left
            anchors.right:      parent.right
            anchors.bottom:     parent.bottom
            anchors.margins:    _toolsMargin

            fixedWingMode:      controlSurfacePanel._fwdMode

            // Real actuator values; helpers return 0 (neutral) when no live data.
            aileronLeftDeflection:   controlSurfacePanel._surf(0)
            aileronRightDeflection: -controlSurfacePanel._surf(0)
            elevatorDeflection:      controlSurfacePanel._surf(1)
            rudderDeflection:        controlSurfacePanel._surf(3)

            liftThrottleFL: controlSurfacePanel._mot(4)
            liftThrottleFR: controlSurfacePanel._mot(5)
            liftThrottleRL: controlSurfacePanel._mot(6)
            liftThrottleRR: controlSurfacePanel._mot(7)
            pusherThrottle: controlSurfacePanel._mot(2)
        }
    }

    //-------------------------------------------------------------------------
    //-- MAVROS actuator (RCOut) source for the control-surface widget.
    //   Non-visual; gated like the video panel so non-ROS builds don't import
    //   Custom.Ros. When inactive the widget falls back to MAVLink servo / static.
    Loader {
        id:     rosActuatorLoader
        active: (typeof customRosEnabled !== 'undefined') && customRosEnabled
        source: active ? "qrc:/qml/Custom/Widgets/RosActuatorSource.qml" : ""
    }

    //-------------------------------------------------------------------------
    //-- Recognition video panel (VTOL-GCS, ROS build only)
    //   Loaded by URL and gated on `customRosEnabled` (set from CustomPlugin)
    //   so non-ROS builds never try to import Custom.Ros.
    Loader {
        id:                     rosVideoLoader
        active:                 (typeof customRosEnabled !== 'undefined') && customRosEnabled
        source:                 active ? "qrc:/qml/Custom/Widgets/RosVideoPanel.qml" : ""
        anchors.left:           parent.left
        anchors.bottom:         parent.bottom
        anchors.leftMargin:     _toolsMargin
        anchors.bottomMargin:   _toolsMargin + parentToolInsets.bottomEdgeLeftInset
        width:                  ScreenTools.defaultFontPixelWidth * 34
        height:                 ScreenTools.defaultFontPixelHeight * 15
    }
}
