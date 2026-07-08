#pragma once

#include <QtPositioning/QGeoPositionInfoSource>

/// Wraps the platform default position source (geoclue2 on Linux) so QGC will
/// accept a coarse "laptop GPS" fix as the ground-station position.
///
/// QGC's PositionManager only takes a GCS position when the reported horizontal
/// accuracy is <= 100 m, but WiFi geolocation is usually coarser, so those fixes
/// would be dropped and the map would never center on the ground station. This
/// source forwards the inner coordinate unchanged while clamping the reported
/// accuracy under that gate. It also keeps the inner source on *all* positioning
/// methods so WiFi geolocation isn't disabled when QGC asks for satellite-only.
class CustomGeoclueSource : public QGeoPositionInfoSource
{
    Q_OBJECT

public:
    explicit CustomGeoclueSource(QObject *parent = nullptr);

    QGeoPositionInfo lastKnownPosition(bool fromSatellitePositioningMethodsOnly = false) const override;
    PositioningMethods supportedPositioningMethods() const override;
    int minimumUpdateInterval() const override;
    Error error() const override;
    void setUpdateInterval(int msec) override;
    void setPreferredPositioningMethods(PositioningMethods methods) override;

public slots:
    void startUpdates() override;
    void stopUpdates() override;
    void requestUpdate(int timeout = 0) override;

private:
    void _forward(const QGeoPositionInfo &info);

    QGeoPositionInfoSource *_inner = nullptr;
};
