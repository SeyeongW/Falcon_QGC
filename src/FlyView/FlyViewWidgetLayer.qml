import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QtLocation
import QtPositioning
import QtQuick.Window
import QtQml.Models

import QGroundControl
import QGroundControl.Controls
import QGroundControl.FlyView
import QGroundControl.FlightMap
import QGroundControl.Viewer3D
import Custom.Widgets

// This is the ui overlay layer for the widgets/tools for Fly View
Item {
    id: _root

    property var    parentToolInsets
    property var    totalToolInsets:        _totalToolInsets
    property var    mapControl
    property Item   telemetryContainer

    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
    property var    _planMasterController:  globals.planMasterControllerFlyView
    property var    _missionController:     _planMasterController.missionController
    property var    _geoFenceController:    _planMasterController.geoFenceController
    property var    _rallyPointController:  _planMasterController.rallyPointController
    property var    _guidedController:      globals.guidedControllerFlyView
    property real   _margins:               ScreenTools.defaultFontPixelWidth / 2
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75
    property rect   _centerViewport:        Qt.rect(0, 0, width, height)
    property real   _rightPanelWidth:       ScreenTools.defaultFontPixelWidth * 30
    property real   _layoutMargin:          ScreenTools.defaultFontPixelWidth * 0.75
    property bool   _layoutSpacing:         ScreenTools.defaultFontPixelWidth
    property bool   _showSingleVehicleUI:   true
    property var    _primaryBattery:        _activeVehicle && _activeVehicle.batteries.count > 0
                                                ? _activeVehicle.batteries.get(0)
                                                : null

    function _telemetryText(fact, decimalPlaces) {
        if (!fact || !isFinite(Number(fact.value))) {
            return qsTr("–")
        }

        const units = fact.units ? " " + fact.units : ""
        return Number(fact.value).toFixed(decimalPlaces) + units
    }

    function _factTelemetryText(fact, showUnits) {
        if (!fact) {
            return qsTr("–")
        }

        return fact.enumOrValueString + (showUnits && fact.units ? " " + fact.units : "")
    }

    function _descentRateText() {
        if (!_activeVehicle || !isFinite(Number(_activeVehicle.climbRate.value))) {
            return qsTr("–")
        }

        const descentRate = Math.max(0, -Number(_activeVehicle.climbRate.value))
        const units = _activeVehicle.climbRate.units
                ? " " + _activeVehicle.climbRate.units
                : ""
        return descentRate.toFixed(1) + units
    }

    QGCToolInsets {
        id:                     _totalToolInsets
        leftEdgeTopInset:       parentToolInsets.leftEdgeTopInset
        leftEdgeCenterInset:    parentToolInsets.leftEdgeCenterInset
        leftEdgeBottomInset:    virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.leftEdgeBottomInset : parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      Math.max(toolStrip.rightEdgeTopInset, topRightPanel.rightEdgeTopInset)
        rightEdgeCenterInset:   topRightPanel.rightEdgeCenterInset
        rightEdgeBottomInset:   bottomRightRowLayout.rightEdgeBottomInset
        topEdgeLeftInset:       parentToolInsets.topEdgeLeftInset
        topEdgeCenterInset:     mapScale.topEdgeCenterInset
        topEdgeRightInset:      Math.max(toolStrip.topEdgeRightInset, topRightPanel.topEdgeRightInset)
        bottomEdgeLeftInset:    virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.bottomEdgeLeftInset : parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  bottomRightRowLayout.bottomEdgeCenterInset
        bottomEdgeRightInset:   virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.bottomEdgeRightInset : bottomRightRowLayout.bottomEdgeRightInset
    }

    FlyViewTopRightPanel {
        id:                     topRightPanel
        anchors.top:            parent.top
        anchors.right:          parent.right
        maximumHeight:          parent.height - (bottomRightRowLayout.height + _margins * 4)

        property real topEdgeRightInset:    height + _layoutMargin
        property real rightEdgeTopInset:    width + _layoutMargin
        property real rightEdgeCenterInset: rightEdgeTopInset
    }

    FlyViewTopRightColumnLayout {
        id:                 topRightColumnLayout
        anchors.top:        parent.top
        anchors.right:      parent.right
        spacing:            _layoutSpacing
        visible:           !topRightPanel.visible

        property real topEdgeRightInset:    childrenRect.height + _layoutMargin
        property real rightEdgeTopInset:    width + _layoutMargin
        property real rightEdgeCenterInset: rightEdgeTopInset
    }

    Item {
        id:                     telemetryPanel
        parent:                 _root.telemetryContainer ? _root.telemetryContainer : _root
        anchors.fill:           parent
        visible:                _root.visible && _activeVehicle
        z:                      QGroundControl.zOrderWidgets

        readonly property real _contentScale: Math.max(
                                                  0.9,
                                                  Math.min(
                                                      1.5,
                                                      height / (ScreenTools.defaultFontPixelHeight * 6)
                                                  )
                                              )

        GridLayout {
            anchors.fill:        parent
            anchors.leftMargin:  _toolsMargin
            anchors.rightMargin: _toolsMargin
            anchors.topMargin:   ScreenTools.defaultFontPixelHeight * 0.35
            anchors.bottomMargin: anchors.topMargin
            columns:             5
            columnSpacing:       ScreenTools.defaultFontPixelWidth * 0.8
            rowSpacing:          ScreenTools.defaultFontPixelHeight * 0.2

            Repeater {
                model: [
                    { label: qsTr("CURRENT"), value: _telemetryText(_primaryBattery ? _primaryBattery.current : null, 1) },
                    { label: qsTr("ROLL"),    value: _telemetryText(_activeVehicle ? _activeVehicle.roll : null, 1) },
                    { label: qsTr("PITCH"),   value: _telemetryText(_activeVehicle ? _activeVehicle.pitch : null, 1) },
                    { label: qsTr("DESCENT"), value: _descentRateText() },
                    { label: qsTr("ALT"),     value: _factTelemetryText(_activeVehicle ? _activeVehicle.altitudeRelative : null, true) },
                    { label: qsTr("V/S"),     value: _factTelemetryText(_activeVehicle ? _activeVehicle.climbRate : null, true) },
                    { label: qsTr("G/S"),     value: _factTelemetryText(_activeVehicle ? _activeVehicle.groundSpeed : null, true) },
                    { label: qsTr("A/S"),     value: _factTelemetryText(_activeVehicle ? _activeVehicle.airSpeed : null, true) },
                    { label: qsTr("THR"),     value: _factTelemetryText(_activeVehicle ? _activeVehicle.throttlePct : null, true) },
                    { label: qsTr("TIME"),    value: _factTelemetryText(_activeVehicle ? _activeVehicle.flightTime : null, false) }
                ]

                ColumnLayout {
                    required property var modelData

                    Layout.fillWidth:  true
                    Layout.fillHeight: true
                    spacing:           0

                    QGCLabel {
                        Layout.fillWidth:      true
                        text:                  modelData.label
                        color:                 "#5796B4"
                        horizontalAlignment:   Text.AlignHCenter
                        font.bold:             true
                        font.pixelSize:        ScreenTools.defaultFontPixelHeight
                                               * 0.70
                                               * telemetryPanel._contentScale
                    }

                    QGCLabel {
                        Layout.fillWidth:      true
                        text:                  modelData.value
                        color:                 "white"
                        elide:                 Text.ElideRight
                        horizontalAlignment:   Text.AlignHCenter
                        font.bold:             true
                        font.pixelSize:        ScreenTools.defaultFontPixelHeight
                                               * 0.90
                                               * telemetryPanel._contentScale
                    }
                }
            }
        }
    }

    FlyViewBottomRightRowLayout {
        id:                 bottomRightRowLayout
        anchors.bottom:     parent.bottom
        anchors.right:      parent.right
        spacing:            _layoutSpacing

        property real bottomEdgeRightInset:     height + _layoutMargin
        property real bottomEdgeCenterInset:    bottomEdgeRightInset
        property real rightEdgeBottomInset:     width + _layoutMargin
    }

    FlyViewMissionCompleteDialog {
        missionController:      _missionController
        geoFenceController:     _geoFenceController
        rallyPointController:   _rallyPointController
    }

    //-- Virtual Joystick
    Loader {
        id:                         virtualJoystickMultiTouch
        z:                          QGroundControl.zOrderTopMost + 1
        anchors.right:              parent.right
        anchors.rightMargin:        anchors.leftMargin
        height:                     Math.min(parent.height * 0.25, ScreenTools.defaultFontPixelWidth * 16)
        visible:                    _virtualJoystickEnabled && !QGroundControl.videoManager.fullScreen && !(_activeVehicle ? _activeVehicle.usingHighLatencyLink : false)
        anchors.bottom:             parent.bottom
        anchors.bottomMargin:       bottomLoaderMargin
        anchors.left:               parent.left
        anchors.leftMargin:         ( y > toolStrip.y + toolStrip.height ? toolStrip.width / 2 : toolStrip.width * 1.05 + toolStrip.x)
        source:                     "qrc:/qml/QGroundControl/FlyView/VirtualJoystick.qml"
        active:                     _virtualJoystickEnabled && !(_activeVehicle ? _activeVehicle.usingHighLatencyLink : false)

        property real bottomEdgeLeftInset:     parent.height-y
        property bool autoCenterThrottle:      QGroundControl.settingsManager.appSettings.virtualJoystickAutoCenterThrottle.rawValue
        property bool leftHandedMode:          QGroundControl.settingsManager.appSettings.virtualJoystickLeftHandedMode.rawValue
        property bool _virtualJoystickEnabled: QGroundControl.settingsManager.appSettings.virtualJoystick.rawValue
        property real bottomEdgeRightInset:    parent.height-y
        property var  _pipViewMargin:          _pipView.visible ? parentToolInsets.bottomEdgeLeftInset + ScreenTools.defaultFontPixelHeight * 2 :
                                               bottomRightRowLayout.height + ScreenTools.defaultFontPixelHeight * 1.5

        property var  bottomLoaderMargin:      _pipViewMargin >= parent.height / 2 ? parent.height / 2 : _pipViewMargin

        // Width is difficult to access directly hence this hack which may not work in all circumstances
        property real leftEdgeBottomInset:  visible ? bottomEdgeLeftInset + width/18 - ScreenTools.defaultFontPixelHeight*2 : 0
        property real rightEdgeBottomInset: visible ? bottomEdgeRightInset + width/18 - ScreenTools.defaultFontPixelHeight*2 : 0
        property real rootWidth:            _root.width
        property var  itemX:                virtualJoystickMultiTouch.x   // real X on screen

        onRootWidthChanged: virtualJoystickMultiTouch.status == Loader.Ready && visible ? virtualJoystickMultiTouch.item.uiTotalWidth = rootWidth : undefined
        onItemXChanged:     virtualJoystickMultiTouch.status == Loader.Ready && visible ? virtualJoystickMultiTouch.item.uiRealX = itemX : undefined

        //Loader status logic
        onLoaded: {
            if (virtualJoystickMultiTouch.visible) {
                virtualJoystickMultiTouch.item.calibration = true
                virtualJoystickMultiTouch.item.uiTotalWidth = rootWidth
                virtualJoystickMultiTouch.item.uiRealX = itemX
            } else {
                virtualJoystickMultiTouch.item.calibration = false
            }
        }
    }

    FlyViewToolStrip {
        id:                     toolStrip
        anchors.right:          parent.right
        anchors.top:            parent.top
        z:                      QGroundControl.zOrderWidgets
        maxHeight:              parent.height - y - parentToolInsets.bottomEdgeLeftInset - _toolsMargin
        maxWidth:               parent.width
        horizontal:             true
        visible:                !QGroundControl.videoManager.fullScreen

        onDisplayPreFlightChecklist: {
            if (!preFlightChecklistLoader.active) {
                preFlightChecklistLoader.active = true
            }
            preFlightChecklistLoader.item.open()
        }

        property real topEdgeRightInset:    visible ? y + height : 0
        property real rightEdgeTopInset:    visible ? parent.width - x : 0
    }

    VehicleWarnings {
        anchors.centerIn:   parent
        z:                  QGroundControl.zOrderTopMost
    }

    MapScale {
        id:                 mapScale
        anchors.right:      toolStrip.left
        anchors.rightMargin: _toolsMargin
        anchors.top:        parent.top
        mapControl:         _mapControl
        autoHide:           true
        visible:            !ScreenTools.isTinyScreen && QGroundControl.corePlugin.options.flyView.showMapScale && QGCViewer3DManager.displayMode !== QGCViewer3DManager.View3D && mapControl.pipState.state === mapControl.pipState.fullState

        property real topEdgeCenterInset: visible ? y + height : 0
    }

    Loader {
        id: preFlightChecklistLoader
        sourceComponent: preFlightChecklistPopup
        active: false
    }

    Component {
        id: preFlightChecklistPopup
        FlyViewPreFlightChecklistPopup {
        }
    }
}
