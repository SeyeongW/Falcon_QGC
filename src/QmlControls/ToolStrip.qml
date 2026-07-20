import QtQuick
import QtQuick.Controls

import QGroundControl
import QGroundControl.Controls

Rectangle {
    id:         _root
    color:      qgcPal.windowTransparent
    width:      horizontal ? Math.min(maxWidth, toolStripGrid.width + (flickable.anchors.margins * 2))
                           : ScreenTools.defaultFontPixelWidth * 7
    height:     horizontal ? ScreenTools.defaultFontPixelWidth * 7
                           : Math.min(maxHeight, toolStripGrid.height + (flickable.anchors.margins * 2))
    radius:     ScreenTools.defaultFontPixelWidth / 2

    property alias  model:              repeater.model
    property real   maxHeight           ///< Maximum height for control, determines whether text is hidden to make control shorter
    property real   maxWidth:            Number.POSITIVE_INFINITY
    property bool   horizontal:          false
    property var    fontSize:           ScreenTools.smallFontPointSize
    readonly property real _buttonExtent: ScreenTools.defaultFontPixelWidth * 6.2

    property var _dropPanel: dropPanel

    function simulateClick(buttonIndex) {
        var button = repeater.itemAt(buttonIndex)
        if (button.checkable) {
            button.checked = !button.checked
        }
        button.clicked()
    }

    signal dropped(int index)

    DeadMouseArea {
        anchors.fill: parent
    }

    QGCFlickable {
        id:                 flickable
        anchors.margins:    ScreenTools.defaultFontPixelWidth * 0.4
        anchors.fill:       parent
        contentWidth:       toolStripGrid.width
        contentHeight:      toolStripGrid.height
        flickableDirection: horizontal ? Flickable.HorizontalFlick : Flickable.VerticalFlick
        clip:               true

        Grid {
            id:             toolStripGrid
            columns:        horizontal ? 0 : 1
            rows:           horizontal ? 1 : 0
            flow:           horizontal ? Grid.TopToBottom : Grid.LeftToRight
            spacing:        ScreenTools.defaultFontPixelWidth * 0.25

            Repeater {
                id: repeater

                ToolStripHoverButton {
                    id:                 buttonTemplate
                    width:              _root._buttonExtent
                    height:             _root._buttonExtent
                    radius:             ScreenTools.defaultFontPixelWidth / 2
                    fontPointSize:      _root.fontSize
                    toolStripAction:    modelData
                    dropPanel:          _dropPanel
                    onDropped: (index) => _root.dropped(index)

                    onCheckedChanged: {
                        // We deal with exclusive check state manually since usinug autoExclusive caused all sorts of crazt problems
                        if (checked) {
                            for (var i=0; i<repeater.count; i++) {
                                if (i != index) {
                                    var button = repeater.itemAt(i)
                                    if (button.checked) {
                                        button.checked = false
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    ToolStripDropPanel {
        id:         dropPanel
        toolStrip:  _root
    }
}
