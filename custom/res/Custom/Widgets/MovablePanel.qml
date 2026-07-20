import QtQuick

import QGroundControl
import QGroundControl.Controls

/// Reusable floating container for the VTOL-GCS overlay panels.
///
/// Drag any empty area of the panel to move it; move the cursor to an edge or corner
/// (the cursor turns into a resize arrow) and drag to resize. There are no visible
/// handles. Child controls (combo boxes, buttons) keep working: the move layer sits
/// *below* the content, so a press on the panel background falls through to it while a
/// press on a control is consumed by the control. Position/size stay within the
/// parent's bounds and are session-only (not persisted).
///
/// Usage: give it an initial `x`/`y`/`width`/`height` and put one filling child inside,
/// e.g. `MovablePanel { ...; Rectangle { anchors.fill: parent; ... } }`.
Item {
    id: root

    default property alias content: contentHolder.data

    property bool movable:    true
    property bool resizable:  true
    property real minWidth:   ScreenTools.defaultFontPixelWidth * 14
    property real minHeight:  ScreenTools.defaultFontPixelHeight * 3
    property real edgeMargin: ScreenTools.defaultFontPixelWidth * 0.9   // resize grab thickness

    function clampToParent() {
        if (!parent) {
            return
        }
        width  = Math.max(minWidth,  Math.min(width,  parent.width))
        height = Math.max(minHeight, Math.min(height, parent.height))
        x = Math.max(0, Math.min(x, parent.width  - width))
        y = Math.max(0, Math.min(y, parent.height - height))
    }

    onWidthChanged:        clampToParent()
    onHeightChanged:       clampToParent()
    Component.onCompleted: clampToParent()

    // Move layer (bottom): a press on the panel background falls through the content
    // to here and drags the whole panel. Interactive children sit above and win.
    MouseArea {
        id:            dragArea
        anchors.fill:  parent
        enabled:       root.movable
        cursorShape:   Qt.SizeAllCursor
        drag.target:   root
        drag.axis:     Drag.XAndYAxis
        drag.minimumX: 0
        drag.minimumY: 0
        drag.maximumX: root.parent ? root.parent.width  - root.width  : 0
        drag.maximumY: root.parent ? root.parent.height - root.height : 0
        onReleased:    root.clampToParent()
    }

    // Content (middle): fills the panel, above the move layer.
    Item {
        id:           contentHolder
        anchors.fill: parent
    }

    // Resize borders (top): thin invisible strips on each edge/corner. Hovering shows
    // a resize cursor; dragging moves the matching edge(s).
    component ResizeArea: MouseArea {
        property bool rLeft:   false
        property bool rRight:  false
        property bool rTop:    false
        property bool rBottom: false

        visible:         root.resizable
        enabled:         root.resizable
        hoverEnabled:    true
        preventStealing: true
        onPositionChanged: (mouse) => {
            if (!pressed || !root.parent) {
                return
            }
            const p = mapToItem(root.parent, mouse.x, mouse.y)
            let left   = root.x
            let top    = root.y
            let right  = root.x + root.width
            let bottom = root.y + root.height
            if (rLeft)   left   = Math.max(0, Math.min(p.x, right - root.minWidth))
            if (rRight)  right  = Math.min(root.parent.width,  Math.max(p.x, left + root.minWidth))
            if (rTop)    top    = Math.max(0, Math.min(p.y, bottom - root.minHeight))
            if (rBottom) bottom = Math.min(root.parent.height, Math.max(p.y, top + root.minHeight))
            root.x      = left
            root.y      = top
            root.width  = right - left
            root.height = bottom - top
        }
    }

    // Edges
    ResizeArea {   // left
        rLeft: true; cursorShape: Qt.SizeHorCursor
        width: root.edgeMargin
        anchors { left: parent.left; top: parent.top; bottom: parent.bottom; topMargin: root.edgeMargin; bottomMargin: root.edgeMargin }
    }
    ResizeArea {   // right
        rRight: true; cursorShape: Qt.SizeHorCursor
        width: root.edgeMargin
        anchors { right: parent.right; top: parent.top; bottom: parent.bottom; topMargin: root.edgeMargin; bottomMargin: root.edgeMargin }
    }
    ResizeArea {   // top
        rTop: true; cursorShape: Qt.SizeVerCursor
        height: root.edgeMargin
        anchors { top: parent.top; left: parent.left; right: parent.right; leftMargin: root.edgeMargin; rightMargin: root.edgeMargin }
    }
    ResizeArea {   // bottom
        rBottom: true; cursorShape: Qt.SizeVerCursor
        height: root.edgeMargin
        anchors { bottom: parent.bottom; left: parent.left; right: parent.right; leftMargin: root.edgeMargin; rightMargin: root.edgeMargin }
    }
    // Corners
    ResizeArea {   // top-left
        rLeft: true; rTop: true; cursorShape: Qt.SizeFDiagCursor
        width: root.edgeMargin; height: root.edgeMargin
        anchors { left: parent.left; top: parent.top }
    }
    ResizeArea {   // top-right
        rRight: true; rTop: true; cursorShape: Qt.SizeBDiagCursor
        width: root.edgeMargin; height: root.edgeMargin
        anchors { right: parent.right; top: parent.top }
    }
    ResizeArea {   // bottom-left
        rLeft: true; rBottom: true; cursorShape: Qt.SizeBDiagCursor
        width: root.edgeMargin; height: root.edgeMargin
        anchors { left: parent.left; bottom: parent.bottom }
    }
    ResizeArea {   // bottom-right
        rRight: true; rBottom: true; cursorShape: Qt.SizeFDiagCursor
        width: root.edgeMargin; height: root.edgeMargin
        anchors { right: parent.right; bottom: parent.bottom }
    }
}
