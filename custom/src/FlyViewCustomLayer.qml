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
    readonly property color  _falconNavy:   "#071526"
    readonly property color  _falconPanel:  "#0B1D33"
    readonly property color  _falconCyan:   "#38BDF8"
    readonly property color  _falconBlue:   "#1D4ED8"
    readonly property color  _falconMint:   "#5796B4"

    function _clampMissionHeader() {
        if (!missionHeader || !missionHeader.parent) {
            return
        }
        missionHeader.x = Math.max(0, Math.min(missionHeader.x, missionHeader.parent.width - missionHeader.width))
        missionHeader.y = Math.max(0, Math.min(missionHeader.y, missionHeader.parent.height - missionHeader.height))
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
    //-- VTOL-GCS mission header
    Rectangle {
        id:                         missionHeader
        x:                          parentToolInsets.leftEdgeTopInset + _toolsMargin
        y:                          parentToolInsets.topEdgeLeftInset + _toolsMargin
        width:                      ScreenTools.defaultFontPixelWidth * 38
        height:                     ScreenTools.defaultFontPixelHeight * 4.2
        radius:                     6
        color:                      Qt.rgba(0.03, 0.08, 0.14, 0.90)
        border.color:               Qt.rgba(0.22, 0.74, 0.97, 0.75)
        border.width:               1

        onWidthChanged:             _clampMissionHeader()
        onHeightChanged:            _clampMissionHeader()
        Component.onCompleted:      _clampMissionHeader()

        RowLayout {
            anchors.fill:           parent
            anchors.margins:        _toolsMargin
            spacing:                _toolsMargin

            Rectangle {
                Layout.preferredWidth:  ScreenTools.defaultFontPixelHeight * 2.2
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 2.2
                radius:                 5
                color:                  _falconMint
                border.color:           Qt.lighter(_falconMint, 1.18)
                border.width:           1

                QGCLabel {
                    anchors.centerIn:   parent
                    text:               "F"
                    color:              "white"
                    font.bold:          true
                    font.pointSize:     ScreenTools.mediumFontPointSize
                }
            }

            ColumnLayout {
                Layout.fillWidth:       true
                spacing:                0

                QGCLabel {
                    text:               qsTr("FALCON VTOL-GCS")
                    color:              "white"
                    font.bold:          true
                    font.pointSize:     ScreenTools.defaultFontPointSize
                }

                QGCLabel {
                    text:               _activeVehicle ? qsTr("LIVE MISSION CONSOLE") : qsTr("SIMULATION CONSOLE")
                    color:              _falconCyan
                    font.pointSize:     ScreenTools.smallFontPointSize
                    font.letterSpacing: 1
                }
            }

            Rectangle {
                Layout.preferredWidth:  ScreenTools.defaultFontPixelWidth * 7.5
                Layout.preferredHeight: ScreenTools.defaultFontPixelHeight * 1.7
                radius:                 4
                color:                  _activeVehicle ? Qt.rgba(0.13, 0.77, 0.37, 0.18)
                                                       : Qt.rgba(0.96, 0.62, 0.04, 0.18)
                border.color:           _activeVehicle ? "#22C55E" : "#F59E0B"
                border.width:           1

                QGCLabel {
                    anchors.centerIn:   parent
                    text:               _activeVehicle ? qsTr("VEHICLE") : qsTr("STANDBY")
                    color:              _activeVehicle ? "#86EFAC" : "#FCD34D"
                    font.bold:          true
                    font.pointSize:     ScreenTools.smallFontPointSize
                }
            }
        }

        MouseArea {
            anchors.fill:           parent
            acceptedButtons:        Qt.LeftButton
            cursorShape:            Qt.SizeAllCursor
            preventStealing:        true
            drag.target:            missionHeader
            drag.axis:              Drag.XAndYAxis
            drag.minimumX:          0
            drag.minimumY:          0
            drag.maximumX:          missionHeader.parent ? missionHeader.parent.width - missionHeader.width : 0
            drag.maximumY:          missionHeader.parent ? missionHeader.parent.height - missionHeader.height : 0

            onReleased:             _clampMissionHeader()
            onCanceled:             _clampMissionHeader()
        }
    }

    //-------------------------------------------------------------------------
    //-- Heading Indicator
    Rectangle {
        id:                         compassBar
        height:                     ScreenTools.defaultFontPixelHeight * 1.5
        width:                      ScreenTools.defaultFontPixelWidth  * 50
        anchors.bottom:             parent.bottom
        anchors.bottomMargin:       _toolsMargin
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
        color:                      _falconBlue
        radius:                     3
        border.color:               _falconMint
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
        anchors.bottom:         attitudeIndicator.bottom
        anchors.right:          attitudeIndicator.left
        anchors.rightMargin:    -attitudeIndicator.width / 2
        width:                  -anchors.rightMargin + compassBezel.width + (_toolsMargin * 2)
        height:                 attitudeIndicator.height * 0.75
        radius:                 2
        color:                  Qt.rgba(0.03, 0.08, 0.14, 0.88)
        border.color:           Qt.rgba(0.34, 0.59, 0.71, 0.70)
        border.width:           1

        Rectangle {
            id:                     compassBezel
            anchors.verticalCenter: parent.verticalCenter
            anchors.leftMargin:     _toolsMargin
            anchors.left:           parent.left
            width:                  height
            height:                 parent.height - (northLabelBackground.height / 2) - (headingLabelBackground.height / 2)
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
            color:                      _falconPanel

            QGCLabel {
                id:                 headingLabel
                anchors.centerIn:   parent
                text:               _heading
                color:              "white"
                font.pointSize:     ScreenTools.smallFontPointSize
            }
        }
    }

    Rectangle {
        id:                     attitudeIndicator
        // When the (tall) mission phase panel is present it would overlap the
        // attitude indicator, so stack the attitude directly below the panel on
        // the right edge. Non-ROS builds (no panel) keep the original bottom-right
        // position. Clearing the unused anchor with `undefined` avoids
        // over-constraining.
        anchors.top:            missionPhaseLoader.active ? missionPhaseLoader.bottom : undefined
        anchors.topMargin:      _toolsMargin
        anchors.bottom:         missionPhaseLoader.active ? undefined : parent.bottom
        anchors.bottomMargin:   _toolsMargin + parentToolInsets.bottomEdgeRightInset
        anchors.rightMargin:    _toolsMargin
        anchors.right:          parent.right
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
    //-- Mission phase orchestrator panel (VTOL-GCS, ROS build only)
    //   Sequential phase checklist with live progress / section text, driven by
    //   RosBridge (command/run_phase <-> command/status). Top-right corner
    //   (replaces the old control-surface panel); the attitude indicator stacks
    //   directly below it.
    Loader {
        id:                     missionPhaseLoader
        active:                 (typeof customRosEnabled !== 'undefined') && customRosEnabled
        source:                 active ? "qrc:/qml/Custom/Widgets/MissionPhasePanel.qml" : ""
        anchors.top:            parent.top
        anchors.right:          parent.right
        anchors.topMargin:      parentToolInsets.topEdgeRightInset + _toolsMargin
        anchors.rightMargin:    _toolsMargin
        width:                  ScreenTools.defaultFontPixelWidth * 32
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
