import QtQuick

Item {
    id: root

    property bool showHeading: true

    // Keep the existing public interface so FlyView and any other callers do
    // not need to know that the circular attitude indicator became a PFD.
    property bool showPitch: true
    property real size
    property var vehicle: null

    height: size
    width: size

    FalconPfdWidget {
        anchors.fill: parent
        vehicle: root.vehicle
    }
}
