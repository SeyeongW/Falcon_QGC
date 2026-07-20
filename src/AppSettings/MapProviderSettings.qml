import QtQuick
import QtQuick.Layouts

import QGroundControl
import QGroundControl.Controls

/// Fixed Falcon map configuration. We intentionally hide provider/type choices
/// so the custom GCS presents a single dark map experience.
SettingsGroupLayout {
    Layout.fillWidth: true

    property Fact _mapProviderFact:     QGroundControl.settingsManager.flightMapSettings.mapProvider
    property Fact _mapTypeFact:         QGroundControl.settingsManager.flightMapSettings.mapType

    Component.onCompleted: {
        _mapProviderFact.rawValue = "CARTO"
        _mapTypeFact.rawValue = "Dark Matter"
    }

    QGCLabel {
        Layout.fillWidth:   true
        text:               qsTr("Flight map")
        font.bold:          true
    }

    QGCLabel {
        Layout.fillWidth:   true
        text:               qsTr("CARTO Dark Matter")
    }
}
