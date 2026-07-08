#include "CustomGeoclueSource.h"

#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(CustomGeoclueLog, "Custom.Geoclue")

namespace {
// Below QGC's PositionManager::kMinHorizonalAccuracyMeters (100 m) gate.
constexpr qreal kAcceptAccuracyMeters = 50.0;
}

CustomGeoclueSource::CustomGeoclueSource(QObject *parent)
    : QGeoPositionInfoSource(parent)
{
    _inner = QGeoPositionInfoSource::createDefaultSource(this);
    if (!_inner) {
        qCWarning(CustomGeoclueLog) << "no platform position source (geoclue) available";
        return;
    }

    // WiFi geolocation must stay enabled even though QGC prefers satellite.
    _inner->setPreferredPositioningMethods(AllPositioningMethods);

    connect(_inner, &QGeoPositionInfoSource::positionUpdated, this, &CustomGeoclueSource::_forward);
    connect(_inner, &QGeoPositionInfoSource::errorOccurred, this,
            [this](QGeoPositionInfoSource::Error e) { emit errorOccurred(e); });
}

void CustomGeoclueSource::_forward(const QGeoPositionInfo &info)
{
    QGeoPositionInfo out = info;
    // Keep the coordinate; clamp accuracy so QGC's 100 m gate accepts the fix.
    if (!out.hasAttribute(QGeoPositionInfo::HorizontalAccuracy) ||
        out.attribute(QGeoPositionInfo::HorizontalAccuracy) > kAcceptAccuracyMeters) {
        out.setAttribute(QGeoPositionInfo::HorizontalAccuracy, kAcceptAccuracyMeters);
    }
    emit positionUpdated(out);
}

QGeoPositionInfo CustomGeoclueSource::lastKnownPosition(bool fromSatellitePositioningMethodsOnly) const
{
    return _inner ? _inner->lastKnownPosition(fromSatellitePositioningMethodsOnly) : QGeoPositionInfo();
}

QGeoPositionInfoSource::PositioningMethods CustomGeoclueSource::supportedPositioningMethods() const
{
    return _inner ? _inner->supportedPositioningMethods() : NoPositioningMethods;
}

int CustomGeoclueSource::minimumUpdateInterval() const
{
    return _inner ? _inner->minimumUpdateInterval() : 1000;
}

QGeoPositionInfoSource::Error CustomGeoclueSource::error() const
{
    return _inner ? _inner->error() : QGeoPositionInfoSource::UnknownSourceError;
}

void CustomGeoclueSource::setUpdateInterval(int msec)
{
    QGeoPositionInfoSource::setUpdateInterval(msec);
    if (_inner) {
        _inner->setUpdateInterval(msec);
    }
}

void CustomGeoclueSource::setPreferredPositioningMethods(PositioningMethods methods)
{
    QGeoPositionInfoSource::setPreferredPositioningMethods(methods);
    // Deliberately keep the inner source on all methods so WiFi stays available.
    if (_inner) {
        _inner->setPreferredPositioningMethods(AllPositioningMethods);
    }
}

void CustomGeoclueSource::startUpdates()
{
    if (_inner) {
        _inner->startUpdates();
    }
}

void CustomGeoclueSource::stopUpdates()
{
    if (_inner) {
        _inner->stopUpdates();
    }
}

void CustomGeoclueSource::requestUpdate(int timeout)
{
    if (_inner) {
        _inner->requestUpdate(timeout);
    }
}
