import QtQuick

import Custom.Ros

/// Exposes MAVROS actuator (RCOut) data to FlyViewCustomLayer without that file
/// importing Custom.Ros directly (which only exists in ROS-enabled builds). It is
/// loaded through a Loader that is only active when `customRosEnabled` is true.
/// A (non-visual) Item, not QtObject, because QQuickLoader only loads Items.
Item {
    visible: false
    readonly property bool have:     RosBridge.haveActuator
    readonly property var  channels: RosBridge.servoChannels   // PWM per channel, [0] = ch1
}
