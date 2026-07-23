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
import QGroundControl.Toolbar
import QGroundControl.Viewer3D

Item {
    id: _root

    readonly property bool _is3DMode:       QGCViewer3DManager.displayMode === QGCViewer3DManager.View3D
    readonly property bool _keepSceneAlive: QGroundControl.settingsManager.viewer3DSettings.keepSceneAlive.rawValue

    // These should only be used by MainRootWindow
    property var planController:    _planController
    property var guidedController:  _guidedController

    PlanMasterController {
        id:                     _planController
        flyView:                true
        Component.onCompleted:  start()
    }

    property bool   _mainWindowIsMap:       mapControl.pipState.state === mapControl.pipState.fullState
    property bool   _isFullWindowItemDark:  _mainWindowIsMap ? mapControl.isSatelliteMap : true
    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
    property var    _missionController:     _planController.missionController
    property var    _geoFenceController:    _planController.geoFenceController
    property var    _rallyPointController:  _planController.rallyPointController
    property real   _margins:               ScreenTools.defaultFontPixelWidth / 2
    property var    _guidedController:      guidedActionsController
    property var    _guidedValueSlider:     guidedValueSlider
    property var    _widgetLayer:           widgetLayer
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75
    property rect   _centerViewport:        Qt.rect(0, 0, width, height)
    property real   _rightPanelWidth:       ScreenTools.defaultFontPixelWidth * 30
    property var    _mapControl:            mapControl
    property real   _widgetMargin:          ScreenTools.defaultFontPixelWidth * 0.75
    property real   _leftPaneRatio:         0.40
    property real   _videoPaneShare:        0.50
    property real   _consolePaneShare:      0.50
    readonly property real _leftPaneWidth:  width * _leftPaneRatio
    readonly property real _paneSpacing:    Math.max(2, ScreenTools.defaultFontPixelWidth * 0.25)
    readonly property real _topSquareSize:  Math.max(0, (_leftPaneWidth - _paneSpacing) / 2)
    property real   _topPaneHeight:         _topSquareSize
    readonly property real _leftPaneHeight: Math.max(0, mapHolder.height - toolbar.height)
    readonly property real _stackedPaneHeight: Math.max(0, _leftPaneHeight - _topPaneHeight - _paneSpacing * 2)
    readonly property real _videoPaneHeight: _stackedPaneHeight * _videoPaneShare
    readonly property real _modelPaneHeight: _stackedPaneHeight - _videoPaneHeight
    readonly property real _consolePaneWidth: Math.max(0, (_leftPaneWidth - _paneSpacing) * _consolePaneShare)

    property real   _fullItemZorder:    0

    function _calcCenterViewPort() {
        var newToolInset = Qt.rect(0, 0, width, height)
        toolstrip.adjustToolInset(newToolInset)
    }

    function dropMainStatusIndicatorTool() {
        toolbar.dropMainStatusIndicatorTool();
    }

    QGCToolInsets {
        id:                     _toolInsets
        bottomEdgeLeftInset:    0
        bottomEdgeCenterInset:  0
        bottomEdgeRightInset:   0
        leftEdgeBottomInset:    0
    }

    Item {
        id:                 mapHolder
        anchors.fill:       parent

        Rectangle {
            anchors.fill:   parent
            color:          "#071526"
        }

        Item {
            id:                     leftPane
            anchors.left:           parent.left
            anchors.top:            parent.top
            anchors.bottom:         parent.bottom
            anchors.bottomMargin:   toolbar.height
            width:                  _leftPaneWidth
        }

        Item {
            id:                     leftInfoPane
            anchors.left:           leftPane.left
            anchors.right:          leftPane.right
            anchors.top:            leftPane.top
            anchors.bottom:         videoPane.top
            anchors.bottomMargin:   _paneSpacing
            clip:                   true
        }

        Item {
            id:                     videoPane
            anchors.left:           leftPane.left
            anchors.right:          leftPane.right
            anchors.bottom:         leftPane.bottom
            height:                 _videoPaneHeight
            clip:                   true
        }

        Item {
            id:                     mapPane
            anchors.left:           leftPane.right
            anchors.leftMargin:     _paneSpacing
            anchors.right:          parent.right
            anchors.top:            parent.top
            anchors.bottom:         parent.bottom
            anchors.bottomMargin:   toolbar.height
            clip:                   true
        }

        Rectangle {
            id:                     topPaneDivider
            anchors.left:           leftPane.left
            anchors.right:          leftPane.right
            y:                      _topPaneHeight
            height:                 _paneSpacing
            color:                  "#38BDF8"
            opacity:                0.65
            z:                      QGroundControl.zOrderWidgets

            MouseArea {
                anchors.centerIn:   parent
                width:              parent.width
                height:             Math.max(parent.height, ScreenTools.defaultFontPixelHeight)
                hoverEnabled:       true
                cursorShape:        Qt.SplitVCursor

                onPositionChanged: (mouse) => {
                    if (!pressed || _leftPaneHeight <= 0) {
                        return
                    }
                    const point = mapToItem(leftPane, mouse.x, mouse.y)
                    const minimumStackPaneHeight = ScreenTools.defaultFontPixelHeight * 6
                    _topPaneHeight = Math.max(
                                         ScreenTools.defaultFontPixelHeight * 8,
                                         Math.min(
                                             _leftPaneHeight - _paneSpacing * 2 - minimumStackPaneHeight * 2,
                                             point.y
                                         )
                                     )
                }
            }
        }

        Rectangle {
            id:                     consolePaneDivider
            x:                      _consolePaneWidth
            y:                      0
            width:                  _paneSpacing
            height:                 _topPaneHeight
            color:                  "#38BDF8"
            opacity:                0.65
            z:                      QGroundControl.zOrderWidgets

            MouseArea {
                anchors.centerIn:   parent
                width:              Math.max(parent.width, ScreenTools.defaultFontPixelWidth * 2)
                height:             parent.height
                hoverEnabled:       true
                cursorShape:        Qt.SplitHCursor

                onPositionChanged: (mouse) => {
                    if (!pressed || _leftPaneWidth <= _paneSpacing) {
                        return
                    }
                    const point = mapToItem(leftPane, mouse.x, mouse.y)
                    _consolePaneShare = Math.max(
                                            0.25,
                                            Math.min(0.75, point.x / (_leftPaneWidth - _paneSpacing))
                                        )
                }
            }
        }

        Rectangle {
            id:                     verticalPaneDivider
            anchors.left:           leftPane.right
            anchors.top:            parent.top
            anchors.bottom:         parent.bottom
            anchors.bottomMargin:   toolbar.height
            width:                  _paneSpacing
            color:                  "#38BDF8"
            opacity:                0.65
            z:                      QGroundControl.zOrderWidgets

            MouseArea {
                anchors.centerIn:   parent
                width:              Math.max(parent.width, ScreenTools.defaultFontPixelWidth * 2)
                height:             parent.height
                hoverEnabled:       true
                cursorShape:        Qt.SplitHCursor

                onPositionChanged: (mouse) => {
                    if (!pressed || mapHolder.width <= 0) {
                        return
                    }
                    const point = mapToItem(mapHolder, mouse.x, mouse.y)
                    _leftPaneRatio = Math.max(0.25, Math.min(0.65, point.x / mapHolder.width))
                }
            }
        }

        Rectangle {
            id:                     videoPaneDivider
            anchors.left:           leftPane.left
            anchors.right:          leftPane.right
            anchors.bottom:         videoPane.top
            height:                 _paneSpacing
            color:                  "#38BDF8"
            opacity:                0.65
            z:                      QGroundControl.zOrderWidgets

            MouseArea {
                anchors.centerIn:   parent
                width:              parent.width
                height:             Math.max(parent.height, ScreenTools.defaultFontPixelHeight)
                hoverEnabled:       true
                cursorShape:        Qt.SplitVCursor

                onPositionChanged: (mouse) => {
                    if (!pressed || _stackedPaneHeight <= 0) {
                        return
                    }
                    const point = mapToItem(leftPane, mouse.x, mouse.y)
                    const requestedVideoHeight = leftPane.height - point.y
                    _videoPaneShare = Math.max(0.20, Math.min(0.80, requestedVideoHeight / _stackedPaneHeight))
                }
            }
        }

        PipView {
            id:                     mapLayout
            parent:                 mapPane
            anchors.fill:           parent
            item1IsFullSettingsKey: "MainFlyWindowIsMap"
            item1:                  mapControl
            show:                   false
        }

        FlyViewMap {
            id:                     mapControl
            planMasterController:   _planController
            rightPanelWidth:        ScreenTools.defaultFontPixelHeight * 9
            pipView:                mapLayout
            pipMode:                false
            toolInsets:             widgetLayer.totalToolInsets
            mapName:                "FlightDisplayView"
            enabled:                !_is3DMode
            visible:                !_is3DMode
        }

        PipView {
            id:                     videoLayout
            parent:                 videoPane
            anchors.fill:           parent
            item1IsFullSettingsKey: "MainFlyWindowIsVideo"
            item1:                  videoControl
            show:                   false
        }

        FlyViewVideo {
            id:         videoControl
            pipView:    videoLayout
        }

        FlyViewWidgetLayer {
            id:                     widgetLayer
            parent:                 mapPane
            anchors.fill:           parent
            anchors.margins:        _widgetMargin
            z:                      _fullItemZorder + 2
            parentToolInsets:       _toolInsets
            mapControl:             _mapControl
            visible:                !QGroundControl.videoManager.fullScreen
        }

        FlyViewCustomLayer {
            id:                 customOverlay
            parent:             leftInfoPane
            anchors.fill:       parent
            z:                  _fullItemZorder + 2
            parentToolInsets:   _toolInsets
            mapControl:         _mapControl
            lowerPanelHeight:   _modelPaneHeight
            topPanelHeight:     _topPaneHeight
            consolePanelWidth:  _consolePaneWidth
            paneSpacing:        _paneSpacing
            visible:            !QGroundControl.videoManager.fullScreen
        }

        // Development tool for visualizing the insets for a paticular layer, show if needed
        FlyViewInsetViewer {
            id:                     widgetLayerInsetViewer
            parent:                 mapPane
            anchors.fill:           parent
            z:                      widgetLayer.z + 1
            insetsToView:           widgetLayer.totalToolInsets
            visible:                false
        }

        GuidedActionsController {
            id:                 guidedActionsController
            missionController:  _missionController
            guidedValueSlider:     _guidedValueSlider
        }

        //-- Guided value slider (e.g. altitude)
        GuidedValueSlider {
            id:                 guidedValueSlider
            anchors.right:      parent.right
            anchors.top:        parent.top
            anchors.bottom:     parent.bottom
            anchors.bottomMargin: toolbar.height
            z:                  QGroundControl.zOrderTopMost
            visible:            false
        }

        Loader {
            id:           viewer3DLoader
            parent:       mapPane
            z:            1
            anchors.fill: parent
            visible:      _is3DMode
        }

        Connections {
            target: QGCViewer3DManager
            function onDisplayModeChanged() {
                if (QGCViewer3DManager.displayMode === QGCViewer3DManager.View3D) {
                    if (!viewer3DLoader.item) {
                        viewer3DLoader.setSource(
                            "qrc:/qml/QGroundControl/Viewer3D/Models3D/Viewer3DModel.qml",
                            { missionController: Qt.binding(() => _missionController) }
                        )
                    }
                } else if (!_keepSceneAlive) {
                    viewer3DLoader.source = ""
                }
            }
        }
    }

    FlyViewToolBar {
        id:                 toolbar
        anchors.bottom:     parent.bottom
        guidedValueSlider:  _guidedValueSlider
        visible:            !QGroundControl.videoManager.fullScreen
    }
}
