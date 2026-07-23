import QtQuick

import QGroundControl
import QGroundControl.Controls

Rectangle {
    id: root

    property real headerHeight: Math.max(28, Math.min(ScreenTools.defaultFontPixelHeight * 1.5, 34))
    property real resizeHandleSize: 20

    color:        Qt.rgba(0.03, 0.08, 0.14, 0.90)
    radius:       6
    border.color: Qt.rgba(0.22, 0.74, 0.97, 0.75)
    border.width: 1
    clip:         true

    readonly property real _panelMargin: ScreenTools.defaultFontPixelWidth * 0.75
    readonly property color _accent: "#38BDF8"

    Item {
        id:             header
        anchors.top:    parent.top
        anchors.left:   parent.left
        anchors.right:  parent.right
        height:         root.headerHeight

        QGCLabel {
            anchors.left:           parent.left
            anchors.leftMargin:     root._panelMargin
            anchors.verticalCenter: parent.verticalCenter
            text:                   qsTr("AIRCRAFT MISSION STATUS")
            color:                  root._accent
            font.bold:              true
            font.letterSpacing:     1
        }

        Rectangle {
            anchors.left:   parent.left
            anchors.right:  parent.right
            anchors.bottom: parent.bottom
            height:         1
            color:          Qt.rgba(0.22, 0.74, 0.97, 0.28)
        }
    }

    Item {
        id:              contentArea
        anchors.left:    parent.left
        anchors.right:   parent.right
        anchors.top:     header.bottom
        anchors.bottom:  parent.bottom
        anchors.margins: root._panelMargin
    }

    Item {
        id:                  resizeHandleVisual
        anchors.right:       parent.right
        anchors.bottom:      parent.bottom
        anchors.rightMargin: 2
        anchors.bottomMargin: 2
        width:               root.resizeHandleSize
        height:              root.resizeHandleSize

        QGCLabel {
            anchors.centerIn: parent
            text:             "◢"
            color:            root._accent
            opacity:          0.75
            font.pixelSize:   root.resizeHandleSize
        }
    }
}
