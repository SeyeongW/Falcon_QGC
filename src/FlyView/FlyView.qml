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
    // Panels no longer draw an enclosing border, so the gutter is what separates
    // them. It is sized to Carbon spacing-03 rather than a hairline gap.
    readonly property real _paneSpacing:    Math.max(8, ScreenTools.defaultFontPixelWidth * 0.9)
    readonly property real _topSquareSize:  Math.max(0, (_leftPaneWidth - _paneSpacing) / 2)
    property real   _topPaneHeight:         _topSquareSize
    // Height reserved for the bottom toolbar strip. The strip is hidden in this
    // build -- the fly view is a full-bleed instrument surface -- so it reserves
    // nothing; the binding keeps the layout correct if it is ever shown again.
    readonly property real _bottomChromeHeight: toolbar.visible ? toolbar.height : 0
    readonly property real _leftPaneHeight: Math.max(0, mapHolder.height - _bottomChromeHeight)
    readonly property real _stackedPaneHeight: Math.max(0, _leftPaneHeight - _topPaneHeight - _paneSpacing * 2)
    readonly property real _videoPaneHeight: _stackedPaneHeight * _videoPaneShare
    readonly property real _modelPaneHeight: _stackedPaneHeight - _videoPaneHeight
    readonly property real _consolePaneWidth: Math.max(0, (_leftPaneWidth - _paneSpacing) * _consolePaneShare)

    property real   _fullItemZorder:    0

    // --- Mission-phase adaptive ("active") layout ---------------------------
    // When enabled, the pane split ratios follow the active mission phase so the
    // GCS emphasizes what matters at each stage (status on takeoff, map on
    // cruise, video on target approach). A manual divider drag pins the layout by
    // switching this off; the AUTO toggle turns it back on.
    // A manual divider drag temporarily pins the split (and suspends the
    // animations so dragging stays responsive); the next mission-phase
    // transition re-arms it. There is deliberately no on-screen AUTO/MANUAL
    // control -- the behaviour is self-correcting, so the chrome was noise.
    property bool   phaseAdaptiveLayout: true

    // Target detection/approach (phase 2) inverts the main split: the camera feed
    // takes over the large right-hand pane and the map drops to a small corner
    // inset, because at that point the operator is flying off the video, not the
    // map. Only engaged while the layout is AUTO.
    readonly property bool _videoFocusMode: phaseAdaptiveLayout && customOverlay.missionPhase === 2

    // --- Video source selection ---------------------------------------------
    // The ROS topic feed and the stock RTSP/UDP stream are both first-class
    // sources; which one fills the video pane is an operator choice, persisted
    // across runs. The stream itself is configured as usual under
    // Application Settings > Video.
    readonly property string _videoSourceSettingsKey: "FlyViewUseRosVideoSource"
    readonly property bool  _rosVideoAvailable: (typeof customRosEnabled !== 'undefined') && customRosEnabled
    property bool   useRosVideoSource:  true
    readonly property bool _rosVideoShown: _rosVideoAvailable && useRosVideoSource

    onUseRosVideoSourceChanged: QGroundControl.saveBoolGlobalSetting(_videoSourceSettingsKey, useRosVideoSource)

    function _calcCenterViewPort() {
        var newToolInset = Qt.rect(0, 0, width, height)
        toolstrip.adjustToolInset(newToolInset)
    }

    function dropMainStatusIndicatorTool() {
        toolbar.dropMainStatusIndicatorTool();
    }

    // Per-phase target split ratios. Indices match MissionPhasePanel phases:
    // 0 pre-check (sensor status + attitude), 1 takeoff/recon (map), 2
    // detect/approach (video takes the main pane, see _videoFocusMode), 3
    // return/landing (map + status). Returns null for unknown phases (-1).
    function _phaseLayoutTargets(phase) {
        switch (phase) {
        // Pre-flight: widen the left column and grow the attitude/status row so
        // the instruments and the sensor checklist are both readable.
        case 0:  return { left: 0.46, video: 0.26, topFactor: 1.32 }
        case 1:  return { left: 0.30, video: 0.38, topFactor: 1.00 }
        // Detection: the left video pane collapses because the feed has moved to
        // the main pane; the freed height goes to the mission route.
        case 2:  return { left: 0.28, video: 0.00, topFactor: 0.90 }
        case 3:  return { left: 0.34, video: 0.42, topFactor: 1.10 }
        default: return null
        }
    }

    function _applyPhaseLayout() {
        if (!phaseAdaptiveLayout) {
            return
        }
        const target = _phaseLayoutTargets(customOverlay.missionPhase)
        if (!target) {
            return
        }
        _leftPaneRatio  = target.left
        _videoPaneShare = target.video
        const minTop = ScreenTools.defaultFontPixelHeight * 8
        const maxTop = Math.max(minTop,
                                _leftPaneHeight - _paneSpacing * 2
                                - (ScreenTools.defaultFontPixelHeight * 6) * 2)
        _topPaneHeight = Math.max(minTop, Math.min(maxTop, target.topFactor * _topSquareSize))
    }

    // Animate the ratios only while adaptive mode drives them; disabled during a
    // manual drag so the divider stays responsive.
    Behavior on _leftPaneRatio {
        enabled: _root.phaseAdaptiveLayout
        NumberAnimation { duration: 450; easing.type: Easing.InOutQuad }
    }
    Behavior on _videoPaneShare {
        enabled: _root.phaseAdaptiveLayout
        NumberAnimation { duration: 450; easing.type: Easing.InOutQuad }
    }
    Behavior on _topPaneHeight {
        enabled: _root.phaseAdaptiveLayout
        NumberAnimation { duration: 450; easing.type: Easing.InOutQuad }
    }

    onPhaseAdaptiveLayoutChanged: if (phaseAdaptiveLayout) _applyPhaseLayout()

    Component.onCompleted: {
        _applyPhaseLayout()
        useRosVideoSource = QGroundControl.loadBoolGlobalSetting(_videoSourceSettingsKey, true)
    }

    Connections {
        target: customOverlay
        function onMissionPhaseChanged() {
            // Re-arm after any manual drag, so each phase starts from its
            // intended layout.
            _root.phaseAdaptiveLayout = true
            _root._applyPhaseLayout()
        }
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
            color:          "#000000"
        }

        Item {
            id:                     leftPane
            anchors.left:           parent.left
            anchors.top:            parent.top
            anchors.bottom:         parent.bottom
            anchors.bottomMargin:   _bottomChromeHeight
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
            anchors.bottomMargin:   _bottomChromeHeight
            clip:                   true
        }

        Rectangle {
            id:                     topPaneDivider
            anchors.left:           leftPane.left
            anchors.right:          leftPane.right
            y:                      _topPaneHeight
            height:                 _paneSpacing
            color:                  "#82CFFF"
            opacity:                0.18
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
                    _root.phaseAdaptiveLayout = false
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
            color:                  "#82CFFF"
            opacity:                0.18
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
            anchors.bottomMargin:   _bottomChromeHeight
            width:                  _paneSpacing
            color:                  "#82CFFF"
            opacity:                0.18
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
                    _root.phaseAdaptiveLayout = false
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
            color:                  "#82CFFF"
            opacity:                0.18
            // Collapsed in video-focus mode, where there is no video pane left to
            // resize and the handle would otherwise float at the pane bottom.
            visible:                _videoPaneHeight > 0
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
                    _root.phaseAdaptiveLayout = false
                    const point = mapToItem(leftPane, mouse.x, mouse.y)
                    const requestedVideoHeight = leftPane.height - point.y
                    _videoPaneShare = Math.max(0.20, Math.min(0.80, requestedVideoHeight / _stackedPaneHeight))
                }
            }
        }

        // Small map inset used while the camera feed owns the main pane. Sized as
        // a fraction of the main pane so it stays legible on any window size.
        Item {
            id:                     cornerMapPane
            parent:                 mapPane
            anchors.right:          parent.right
            anchors.bottom:         parent.bottom
            anchors.margins:        _toolsMargin
            width:                  Math.max(ScreenTools.defaultFontPixelWidth * 24, parent.width * 0.26)
            height:                 Math.max(ScreenTools.defaultFontPixelHeight * 10, parent.height * 0.30)
            visible:                _videoFocusMode
            z:                      _fullItemZorder + 1.5
            clip:                   true
        }

        // PipState.fullState reparents and anchors the map to `pipView.parent`
        // through a State, which is applied once on init. Reparenting the PipView
        // afterwards therefore leaves the map behind -- that is why the corner map
        // came up empty in phase 2. Instead the PipView keeps a fixed host and the
        // host's geometry is what moves, so the anchors stay live.
        Item {
            id:                     mapHost
            parent:                 _videoFocusMode ? cornerMapPane : mapPane
            anchors.fill:           parent
        }

        PipView {
            id:                     mapLayout
            parent:                 mapHost
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

        // Same host indirection as the map: FlyViewVideo is anchored to
        // `pipView.parent` by PipState, so the host is what gets reparented.
        // Whichever source is selected is the one promoted to the main pane in
        // video-focus mode.
        Item {
            id:                     videoHost
            parent:                 (_videoFocusMode && !_rosVideoShown) ? mapPane : videoPane
            // Sits below the source selector rather than under it.
            anchors.left:           parent.left
            anchors.right:          parent.right
            anchors.top:            videoSourceSelector.visible ? videoSourceSelector.bottom : parent.top
            anchors.bottom:         parent.bottom
            anchors.topMargin:      videoSourceSelector.visible ? _toolsMargin : 0
            visible:                !_rosVideoShown
            z:                      _fullItemZorder + 1
        }

        PipView {
            id:                     videoLayout
            parent:                 videoHost
            anchors.fill:           parent
            item1IsFullSettingsKey: "MainFlyWindowIsVideo"
            item1:                  videoControl
            show:                   false
        }

        FlyViewVideo {
            id:         videoControl
            pipView:    videoLayout
        }

        // ROS AI vision feed (topic picker + AR waypoint overlay). Loaded by URL
        // rather than imported so this core view stays free of Custom.Ros, which
        // only exists in the ROS-enabled custom build. Unloaded entirely when the
        // operator selects the RTSP/UDP stream instead.
        Loader {
            id:             rosVideoLoader
            parent:         _videoFocusMode ? mapPane : videoPane
            anchors.left:   parent.left
            anchors.right:  parent.right
            anchors.top:    videoSourceSelector.visible ? videoSourceSelector.bottom : parent.top
            anchors.bottom: parent.bottom
            anchors.topMargin: videoSourceSelector.visible ? _toolsMargin : 0
            z:              _fullItemZorder + 1
            active:         _rosVideoShown
            source:         active ? "qrc:/qml/Custom/Widgets/RosVideoPanel.qml" : ""
            visible:        active && !QGroundControl.videoManager.fullScreen
        }

        // Video source selection field, pinned to the top of whichever pane is
        // currently showing the video. Owns both the ROS topic list and the
        // RTSP/UDP stream address, so there is one place to configure the feed.
        // Absent in non-ROS builds, where there is only one source to pick.
        Loader {
            id:                     videoSourceSelector
            parent:                 _videoFocusMode ? mapPane : videoPane
            anchors.left:           parent.left
            anchors.right:          parent.right
            anchors.top:            parent.top
            anchors.margins:        _toolsMargin
            z:                      _fullItemZorder + 3
            active:                 _rosVideoAvailable
            source:                 active ? "qrc:/qml/Custom/Widgets/VideoSourceSelector.qml" : ""
            visible:                active && !QGroundControl.videoManager.fullScreen

            // Seed the field from the restored setting; from then on the field is
            // the one that drives the fly view (see Connections below).
            onLoaded: item.useRosSource = _root.useRosVideoSource
        }

        Connections {
            target:  videoSourceSelector.item
            ignoreUnknownSignals: true
            function onUseRosSourceChanged() {
                _root.useRosVideoSource = videoSourceSelector.item.useRosSource
            }
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
            anchors.bottomMargin: _bottomChromeHeight
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
        // Hidden: the mission is flown autonomously and the strip's content is
        // already covered by the custom panels (arm state, flight mode, GPS and
        // battery in the pre-flight panel; phase and progress in the mission
        // console). Set to true to bring the stock strip back.
        visible:            false
    }
}
