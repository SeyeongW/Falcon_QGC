#include "CustomFirmwarePlugin.h"
#include "CustomAutoPilotPlugin.h"
#include "px4_custom_mode.h"
#include "Vehicle.h"

CustomFirmwarePlugin::CustomFirmwarePlugin()
{
    // Use the stock PX4 flight-mode set (no custom narrowing) so all normal modes
    // remain selectable like in stock QGC.
}

AutoPilotPlugin* CustomFirmwarePlugin::autopilotPlugin(Vehicle *vehicle) const
{
    return new CustomAutoPilotPlugin(vehicle, vehicle);
}

const QVariantList& CustomFirmwarePlugin::toolIndicators(const Vehicle *vehicle)
{
    if (_toolIndicatorList.size() == 0) {
        // First call the base class to get the standard QGC list. This way we are guaranteed to always get
        // any new toolbar indicators which are added upstream in our custom build.
        _toolIndicatorList = FirmwarePlugin::toolIndicators(vehicle);
        // Then specifically remove the RC RSSI indicator.
        _toolIndicatorList.removeOne(QVariant::fromValue(QUrl::fromUserInput("qrc:/qml/QGroundControl/Toolbar/RCRSSIIndicator.qml")));
    }

    return _toolIndicatorList;
}

bool CustomFirmwarePlugin::hasGimbal(Vehicle* /*vehicle*/, bool &rollSupported, bool &pitchSupported, bool &yawSupported) const
{
    rollSupported = false;
    pitchSupported = true;
    yawSupported = true;

    return true;
}
