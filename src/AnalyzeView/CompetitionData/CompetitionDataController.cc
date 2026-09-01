#include "CompetitionDataController.h"

#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QSaveFile>
#include <QtCore/QTextStream>
#include <QtCore/QUrl>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace {

constexpr double kOutputIntervalSeconds = 0.1;
constexpr double kMinimumGpsRateHz = 9.9;
constexpr double kMaximumGpsGapSeconds = 0.15;
constexpr double kMaximumAuxiliaryAgeSeconds = 0.2;
constexpr qint64 kGpsEpochUnixSeconds = 315964800;
constexpr double kGpsUtcOffsetSeconds = 18.0;
constexpr int kPreviewRowCount = 8;

const QString kNavStateField = QStringLiteral("vehicle_status.nav_state");
const QString kMissionSequenceField = QStringLiteral("mission_result.seq_current");

} // namespace

CompetitionDataController::CompetitionDataController(QObject *parent)
    : QObject(parent)
    , _parser(this)
{
    connect(&_parser, &LogFileParser::parsingChanged, this, &CompetitionDataController::parsingChanged);
    connect(&_parser, &LogFileParser::parseProgressChanged, this, &CompetitionDataController::parseProgressChanged);
    connect(&_parser, &LogFileParser::parseFileFinished, this, &CompetitionDataController::_parseFinished);
}

CompetitionDataController::~CompetitionDataController() = default;

void CompetitionDataController::loadULog(const QString &filePath)
{
    clear();

    const QString localPath = _localPath(filePath);
    if (localPath.isEmpty() || !QFileInfo::exists(localPath)) {
        _setError(tr("Select an existing PX4 ULog file."));
        return;
    }
    if (!localPath.endsWith(QStringLiteral(".ulg"), Qt::CaseInsensitive)) {
        _setError(tr("Competition data export currently supports PX4 .ulg files only."));
        return;
    }

    _sourcePath = localPath;
    emit resultChanged();
    _parser.startParsingAsync(localPath);
}

bool CompetitionDataController::exportAscii(const QString &filePath)
{
    if (!_ready) {
        const QString message = tr("Load and validate a ULog file before exporting.");
        emit exportFinished(QString(), false, message);
        return false;
    }

    const QString localPath = _localPath(filePath);
    if (localPath.isEmpty()) {
        const QString message = tr("Select an output file.");
        emit exportFinished(QString(), false, message);
        return false;
    }

    QDir outputDirectory = QFileInfo(localPath).dir();
    if (!outputDirectory.exists() && !outputDirectory.mkpath(QStringLiteral("."))) {
        const QString message = tr("Failed to create the output directory.");
        emit exportFinished(localPath, false, message);
        return false;
    }

    const QVector<ExportRow> rows = _generateRows();
    if (rows.size() != _rowCount) {
        const QString message = tr("The selected range no longer contains complete 10 Hz rows.");
        emit exportFinished(localPath, false, message);
        return false;
    }

    QSaveFile outputFile(localPath);
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        const QString message = tr("Failed to open the output file: %1").arg(outputFile.errorString());
        emit exportFinished(localPath, false, message);
        return false;
    }

    QTextStream stream(&outputFile);
    stream.setEncoding(QStringConverter::Latin1);
    stream << "AUTO_MANUAL\tEVENT\tGPST\tLATITUDE\tLONGITUDE\tALTITUDE\tAX\tAY\tAZ\tROLL\tPITCH\tYAW\n";
    for (const ExportRow &row : rows) {
        stream << _rowText(row) << '\n';
    }

    if (!outputFile.commit()) {
        const QString message = tr("Failed to save the output file: %1").arg(outputFile.errorString());
        emit exportFinished(localPath, false, message);
        return false;
    }

    emit exportFinished(localPath, true, QString());
    return true;
}

void CompetitionDataController::clear()
{
    _parser.clear();
    _gpsFields = {};
    _gpsCandidates.clear();
    _gpsSourceOptions.clear();
    _selectedGpsSourceIndex = -1;
    _accelerationFields = {};
    _attitudeFields = {};
    _exportRange = {};
    _armTimes.clear();
    _disarmTimes.clear();
    _armEvents.clear();
    _disarmEvents.clear();
    _selectedArmIndex = -1;
    _selectedDisarmIndex = -1;
    _dataPrepared = false;
    _ready = false;
    _sourcePath.clear();
    _errorMessage.clear();
    _warningMessage.clear();
    _gpsSource.clear();
    _imuSource.clear();
    _attitudeSource.clear();
    _suggestedOutputPath.clear();
    _sourceRateHz = 0.0;
    _durationSeconds = 0.0;
    _rowCount = 0;
    _previewRows.clear();
    emit resultChanged();
}

void CompetitionDataController::setSelectedArmIndex(int index)
{
    if (index == _selectedArmIndex || index < 0 || index >= _armTimes.size()) {
        return;
    }
    _selectedArmIndex = index;
    _updateSelectedRange();
}

void CompetitionDataController::setSelectedDisarmIndex(int index)
{
    if (index == _selectedDisarmIndex || index < 0 || index >= _disarmTimes.size()) {
        return;
    }
    _selectedDisarmIndex = index;
    _updateSelectedRange();
}

void CompetitionDataController::setSelectedGpsSourceIndex(int index)
{
    if (index < 0 || index > _gpsCandidates.size() || index == _selectedGpsSourceIndex) {
        return;
    }
    int candidateIndex = index - 1;
    if (index == 0) {
        double fastestRate = -1.0;
        candidateIndex = 0;
        for (int candidate = 0; candidate < _gpsCandidates.size(); ++candidate) {
            const double rate = _fieldRateHz(_gpsCandidates[candidate]);
            if (rate > fastestRate) {
                fastestRate = rate;
                candidateIndex = candidate;
            }
        }
    }
    _selectedGpsSourceIndex = index;
    _gpsFields = _gpsCandidates[candidateIndex];
    _gpsSource = _gpsFields.latitude.section(QLatin1Char('.'), 0, 0);
    _updateSelectedRange();
}

void CompetitionDataController::_parseFinished(const QString &filePath, bool ok, const QString &errorMessage)
{
    if (filePath != _sourcePath) {
        return;
    }
    if (!ok) {
        _setError(errorMessage);
        return;
    }
    _prepareResult();
}

void CompetitionDataController::_prepareResult()
{
    _gpsCandidates = _availableGpsFields();
    if (_gpsCandidates.isEmpty()) {
        _setError(tr("No supported raw GPS topic was found. Expected sensor_gps or vehicle_gps_position."));
        return;
    }
    _gpsSourceOptions.clear();
    QVariantMap automaticOption;
    automaticOption[QStringLiteral("label")] = tr("자동 선택 (가장 높은 주기)");
    automaticOption[QStringLiteral("rate")] = 0.0;
    automaticOption[QStringLiteral("source")] = QStringLiteral("auto");
    _gpsSourceOptions.append(automaticOption);
    int fastestIndex = 0;
    double fastestRate = -1.0;
    for (int index = 0; index < _gpsCandidates.size(); ++index) {
        const double rate = _fieldRateHz(_gpsCandidates[index]);
        QVariantMap option;
        option[QStringLiteral("label")] = QStringLiteral("%1 (%2 Hz)")
                                                .arg(_gpsCandidates[index].latitude.section(QLatin1Char('.'), 0, 0))
                                                .arg(rate, 0, 'f', 2);
        option[QStringLiteral("rate")] = rate;
        option[QStringLiteral("source")] = _gpsCandidates[index].latitude.section(QLatin1Char('.'), 0, 0);
        _gpsSourceOptions.append(option);
        if (rate > fastestRate) {
            fastestRate = rate;
            fastestIndex = index;
        }
    }
    _selectedGpsSourceIndex = fastestIndex + 1;
    _gpsFields = _gpsCandidates[fastestIndex];
    _gpsSource = _gpsFields.latitude.section(QLatin1Char('.'), 0, 0);
    if (_samples(_gpsFields.utcTime).isEmpty()) {
        _setError(tr("The GPS topic has no GNSS UTC time, so GPST cannot be generated."));
        return;
    }
    _gpsSource = _gpsFields.latitude.section(QLatin1Char('.'), 0, 0);

    _accelerationFields = _selectAccelerationFields();
    if (_accelerationFields.x.isEmpty()) {
        _setError(tr("No body-frame IMU acceleration was found. Expected sensor_combined.accelerometer_m_s2."));
        return;
    }
    _imuSource = _accelerationFields.x.section(QLatin1Char('.'), 0, 0);

    _attitudeFields = _selectAttitudeFields();
    if (_attitudeFields.w.isEmpty()) {
        _setError(tr("No vehicle attitude quaternion was found. Expected vehicle_attitude.q."));
        return;
    }
    _attitudeSource = _attitudeFields.w.section(QLatin1Char('.'), 0, 0);

    const TimeRange gpsRange = _gpsRange(_gpsFields);
    if (!gpsRange.valid) {
        _setError(tr("The log does not contain a usable GPS time range."));
        return;
    }

    const QVector<QPointF> &latitudeSamples = _samples(_gpsFields.latitude);
    double intervalSum = 0.0;
    int intervalCount = 0;
    for (int index = 1; index < latitudeSamples.size(); ++index) {
        const double interval = latitudeSamples[index].x() - latitudeSamples[index - 1].x();
        if (interval > 0.0) {
            intervalSum += interval;
            ++intervalCount;
        }
    }
    if (intervalCount == 0) {
        _setError(tr("The GPS topic does not contain enough samples."));
        return;
    }
    _sourceRateHz = 1.0 / (intervalSum / intervalCount);
    _detectArmDisarmEvents();
    if (_armTimes.isEmpty() || _disarmTimes.isEmpty()) {
        _setError(tr("No selectable ARM/DISARM boundaries were found in the log."));
        return;
    }

    _selectedArmIndex = 0;
    _selectedDisarmIndex = _disarmTimes.size() - 1;
    _dataPrepared = true;

    const QFileInfo sourceInfo(_sourcePath);
    _suggestedOutputPath = sourceInfo.dir().filePath(sourceInfo.completeBaseName()
                                                     + QStringLiteral("_competition.txt"));
    _updateSelectedRange();
}

void CompetitionDataController::_updateSelectedRange()
{
    _ready = false;
    _errorMessage.clear();
    _warningMessage.clear();
    _exportRange = {};
    _durationSeconds = 0.0;
    _rowCount = 0;
    _previewRows.clear();

    if (!_dataPrepared || _selectedArmIndex < 0 || _selectedArmIndex >= _armTimes.size()
        || _selectedDisarmIndex < 0 || _selectedDisarmIndex >= _disarmTimes.size()) {
        emit resultChanged();
        return;
    }

    const TimeRange gpsRange = _gpsRange(_gpsFields);
    _exportRange.start = std::max(_armTimes[_selectedArmIndex], gpsRange.start);
    _exportRange.end = std::min(_disarmTimes[_selectedDisarmIndex], gpsRange.end);
    _exportRange.valid = _exportRange.end > _exportRange.start;
    if (!_exportRange.valid) {
        _setError(tr("The selected DISARM must occur after the selected ARM and overlap valid GPS data."));
        return;
    }

    const QVector<QPointF> &latitude = _samples(_gpsFields.latitude);
    double sourceIntervalSum = 0.0;
    double sourceMaximumInterval = 0.0;
    int sourceIntervalCount = 0;
    auto previous = std::lower_bound(latitude.cbegin(), latitude.cend(), _exportRange.start,
                                     [](const QPointF &sample, double timestamp) {
                                         return sample.x() < timestamp;
                                     });
    if (previous != latitude.cend()) {
        for (auto sample = previous + 1; sample != latitude.cend() && sample->x() <= _exportRange.end; ++sample) {
            const double interval = sample->x() - previous->x();
            if (interval > 0.0) {
                sourceIntervalSum += interval;
                sourceMaximumInterval = std::max(sourceMaximumInterval, interval);
                ++sourceIntervalCount;
            }
            previous = sample;
        }
    }
    if (sourceIntervalCount == 0) {
        _setError(tr("The selected ARM/DISARM range does not contain enough GPS samples."));
        return;
    }
    _sourceRateHz = 1.0 / (sourceIntervalSum / sourceIntervalCount);
    if (_sourceRateHz < kMinimumGpsRateHz) {
        _setError(tr("The selected range has %1 Hz GPS data. Competition export requires native GPS data at 10 Hz or faster.")
                      .arg(_sourceRateHz, 0, 'f', 2));
        return;
    }
    const bool sourceHasGap = sourceMaximumInterval > kMaximumGpsGapSeconds;

    const QVector<int> gpsIndices = _selectedGpsIndices();
    if (gpsIndices.size() < 2) {
        _setError(tr("The selected ARM/DISARM range does not contain enough valid GPS samples."));
        return;
    }

    double maximumInterval = 0.0;
    for (int index = 1; index < gpsIndices.size(); ++index) {
        maximumInterval = std::max(maximumInterval,
                                   latitude[gpsIndices[index]].x() - latitude[gpsIndices[index - 1]].x());
    }
    const bool outputHasGap = maximumInterval > kMaximumGpsGapSeconds;

    const QVector<ExportRow> rows = _generateRows();
    if (rows.size() != gpsIndices.size()) {
        _setError(tr("The selected range contains invalid GPS, IMU, or attitude samples."));
        return;
    }

    _rowCount = rows.size();
    _durationSeconds = _exportRange.end - _exportRange.start;
    const int previewCount = std::min(kPreviewRowCount, static_cast<int>(rows.size()));
    for (int index = 0; index < previewCount; ++index) {
        const ExportRow &row = rows[index];
        QVariantMap previewRow;
        previewRow[QStringLiteral("automatic")] = QString::number(row.automatic);
        previewRow[QStringLiteral("event")] = QString::number(row.waypoint);
        previewRow[QStringLiteral("gpst")] = QString::number(row.gpst, 'f', 3);
        previewRow[QStringLiteral("latitude")] = QString::number(row.latitude, 'f', 6);
        previewRow[QStringLiteral("longitude")] = QString::number(row.longitude, 'f', 6);
        previewRow[QStringLiteral("altitude")] = QString::number(row.altitude, 'f', 1);
        previewRow[QStringLiteral("ax")] = QString::number(row.ax, 'f', 3);
        previewRow[QStringLiteral("ay")] = QString::number(row.ay, 'f', 3);
        previewRow[QStringLiteral("az")] = QString::number(row.az, 'f', 3);
        previewRow[QStringLiteral("roll")] = QString::number(row.roll, 'f', 2);
        previewRow[QStringLiteral("pitch")] = QString::number(row.pitch, 'f', 2);
        previewRow[QStringLiteral("yaw")] = QString::number(row.yaw, 'f', 2);
        _previewRows.append(previewRow);
    }

    QStringList warnings;
    if (sourceHasGap || outputHasGap) {
        const double largestGap = std::max(sourceMaximumInterval, maximumInterval);
        warnings.append(tr("The selected range contains a %1 second GPS gap. The export will continue without interpolation.")
                             .arg(largestGap, 0, 'f', 3));
    }
    if (_samples(kNavStateField).isEmpty()) {
        warnings.append(tr("vehicle_status.nav_state is missing; AUTO_MANUAL defaults to manual (0)."));
    }
    if (_samples(kMissionSequenceField).isEmpty()) {
        warnings.append(tr("mission_result.seq_current is missing; EVENT defaults to -1."));
    }
    warnings.append(tr("The selected ARM-to-DISARM span is exported continuously, including any intermediate DISARM/ARM cycles."));
    _warningMessage = warnings.join(QLatin1Char('\n'));
    _ready = true;
    emit resultChanged();
}

void CompetitionDataController::_detectArmDisarmEvents()
{
    _armTimes.clear();
    _disarmTimes.clear();
    _armEvents.clear();
    _disarmEvents.clear();

    const QStringList candidates = {
        QStringLiteral("vehicle_status.arming_state"),
        QStringLiteral("actuator_armed.armed"),
    };

    for (const QString &field : candidates) {
        const QVector<QPointF> &samples = _samples(field);
        if (samples.isEmpty()) {
            continue;
        }

        const bool armingState = field.endsWith(QStringLiteral("arming_state"));
        bool previouslyArmed = false;
        bool stateKnown = false;
        for (const QPointF &sample : samples) {
            const bool armed = armingState ? qRound(sample.y()) == 2 : sample.y() > 0.5;
            if (!stateKnown) {
                stateKnown = true;
                previouslyArmed = armed;
                if (armed) {
                    _armTimes.append(sample.x());
                }
                continue;
            }
            if (armed == previouslyArmed) {
                continue;
            }
            if (armed) {
                _armTimes.append(sample.x());
            } else {
                _disarmTimes.append(sample.x());
            }
            previouslyArmed = armed;
        }
        break;
    }

    for (int index = 0; index < _armTimes.size(); ++index) {
        QVariantMap event;
        event[QStringLiteral("label")] = _eventLabel(tr("ARM"), index + 1, _armTimes[index]);
        event[QStringLiteral("timestamp")] = _armTimes[index];
        _armEvents.append(event);
    }
    for (int index = 0; index < _disarmTimes.size(); ++index) {
        QVariantMap event;
        event[QStringLiteral("label")] = _eventLabel(tr("DISARM"), index + 1, _disarmTimes[index]);
        event[QStringLiteral("timestamp")] = _disarmTimes[index];
        _disarmEvents.append(event);
    }
}

CompetitionDataController::FieldSelection CompetitionDataController::_selectGpsFields() const
{
    struct Candidate {
        const char *latitude;
        const char *longitude;
        const char *altitude;
        const char *utcTime;
        const char *fixType;
    };

    static const Candidate candidates[] = {
        {"sensor_gps.latitude_deg", "sensor_gps.longitude_deg", "sensor_gps.altitude_msl_m",
         "sensor_gps.time_utc_usec", "sensor_gps.fix_type"},
        {"sensor_gps[0].latitude_deg", "sensor_gps[0].longitude_deg", "sensor_gps[0].altitude_msl_m",
         "sensor_gps[0].time_utc_usec", "sensor_gps[0].fix_type"},
        {"vehicle_gps_position.latitude_deg", "vehicle_gps_position.longitude_deg",
         "vehicle_gps_position.altitude_msl_m", "vehicle_gps_position.time_utc_usec",
         "vehicle_gps_position.fix_type"},
        {"vehicle_gps_position[0].latitude_deg", "vehicle_gps_position[0].longitude_deg",
         "vehicle_gps_position[0].altitude_msl_m", "vehicle_gps_position[0].time_utc_usec",
         "vehicle_gps_position[0].fix_type"},
    };

    for (const Candidate &candidate : candidates) {
        if (!_samples(QLatin1String(candidate.latitude)).isEmpty()
            && !_samples(QLatin1String(candidate.longitude)).isEmpty()
            && !_samples(QLatin1String(candidate.altitude)).isEmpty()) {
            return {QLatin1String(candidate.latitude), QLatin1String(candidate.longitude),
                    QLatin1String(candidate.altitude), QLatin1String(candidate.utcTime),
                    QLatin1String(candidate.fixType)};
        }
    }
    return {};
}

QVector<CompetitionDataController::FieldSelection> CompetitionDataController::_availableGpsFields() const
{
    const QVector<FieldSelection> candidates = {
        {QStringLiteral("sensor_gps.latitude_deg"), QStringLiteral("sensor_gps.longitude_deg"), QStringLiteral("sensor_gps.altitude_msl_m"), QStringLiteral("sensor_gps.time_utc_usec"), QStringLiteral("sensor_gps.fix_type")},
        {QStringLiteral("sensor_gps[0].latitude_deg"), QStringLiteral("sensor_gps[0].longitude_deg"), QStringLiteral("sensor_gps[0].altitude_msl_m"), QStringLiteral("sensor_gps[0].time_utc_usec"), QStringLiteral("sensor_gps[0].fix_type")},
        {QStringLiteral("vehicle_gps_position.latitude_deg"), QStringLiteral("vehicle_gps_position.longitude_deg"), QStringLiteral("vehicle_gps_position.altitude_msl_m"), QStringLiteral("vehicle_gps_position.time_utc_usec"), QStringLiteral("vehicle_gps_position.fix_type")},
        {QStringLiteral("vehicle_gps_position[0].latitude_deg"), QStringLiteral("vehicle_gps_position[0].longitude_deg"), QStringLiteral("vehicle_gps_position[0].altitude_msl_m"), QStringLiteral("vehicle_gps_position[0].time_utc_usec"), QStringLiteral("vehicle_gps_position[0].fix_type")},
    };
    QVector<FieldSelection> available;
    for (const FieldSelection &candidate : candidates) {
        if (!_samples(candidate.latitude).isEmpty() && !_samples(candidate.longitude).isEmpty()
            && !_samples(candidate.altitude).isEmpty() && !_samples(candidate.utcTime).isEmpty()) {
            available.append(candidate);
        }
    }
    return available;
}

double CompetitionDataController::_fieldRateHz(const FieldSelection &fields) const
{
    const auto &samples = _samples(fields.latitude);
    double intervalSum = 0.0;
    int count = 0;
    for (int index = 1; index < samples.size(); ++index) {
        const double interval = samples[index].x() - samples[index - 1].x();
        if (interval > 0.0) {
            intervalSum += interval;
            ++count;
        }
    }
    return count > 0 ? 1.0 / (intervalSum / count) : 0.0;
}

CompetitionDataController::AxisSelection CompetitionDataController::_selectAccelerationFields() const
{
    const QStringList prefixes = {
        QStringLiteral("sensor_combined.accelerometer_m_s2"),
        QStringLiteral("sensor_combined[0].accelerometer_m_s2"),
    };
    for (const QString &prefix : prefixes) {
        const AxisSelection fields = {
            prefix + QStringLiteral("[0]"),
            prefix + QStringLiteral("[1]"),
            prefix + QStringLiteral("[2]"),
        };
        if (!_samples(fields.x).isEmpty() && !_samples(fields.y).isEmpty() && !_samples(fields.z).isEmpty()) {
            return fields;
        }
    }
    return {};
}

CompetitionDataController::QuaternionSelection CompetitionDataController::_selectAttitudeFields() const
{
    const QStringList prefixes = {
        QStringLiteral("vehicle_attitude.q"),
        QStringLiteral("vehicle_attitude[0].q"),
    };
    for (const QString &prefix : prefixes) {
        const QuaternionSelection fields = {
            prefix + QStringLiteral("[0]"),
            prefix + QStringLiteral("[1]"),
            prefix + QStringLiteral("[2]"),
            prefix + QStringLiteral("[3]"),
        };
        if (!_samples(fields.w).isEmpty() && !_samples(fields.x).isEmpty()
            && !_samples(fields.y).isEmpty() && !_samples(fields.z).isEmpty()) {
            return fields;
        }
    }
    return {};
}

const QVector<QPointF> &CompetitionDataController::_samples(const QString &fieldName) const
{
    return _parser.fieldSamplesVector(fieldName);
}

CompetitionDataController::TimeRange CompetitionDataController::_gpsRange(const FieldSelection &fields) const
{
    const QVector<QPointF> &latitude = _samples(fields.latitude);
    const QVector<QPointF> &longitude = _samples(fields.longitude);
    const QVector<QPointF> &altitude = _samples(fields.altitude);
    const QVector<QPointF> &utcTime = _samples(fields.utcTime);
    if (latitude.isEmpty() || longitude.isEmpty() || altitude.isEmpty() || utcTime.isEmpty()) {
        return {};
    }

    const double start = std::max({latitude.constFirst().x(), longitude.constFirst().x(),
                                   altitude.constFirst().x(), utcTime.constFirst().x()});
    const double end = std::min({latitude.constLast().x(), longitude.constLast().x(),
                                 altitude.constLast().x(), utcTime.constLast().x()});
    return {start, end, end >= start};
}

QVector<int> CompetitionDataController::_selectedGpsIndices() const
{
    QVector<int> selected;
    if (!_exportRange.valid) {
        return selected;
    }

    const QVector<QPointF> &samples = _samples(_gpsFields.latitude);
    auto next = std::lower_bound(samples.cbegin(), samples.cend(), _exportRange.start,
                                 [](const QPointF &sample, double timestamp) {
                                     return sample.x() < timestamp;
                                 });
    double target = next != samples.cend() ? next->x() : _exportRange.end + 1.0;
    while (next != samples.cend() && target < _exportRange.end - 1e-6) {
        const auto after = std::lower_bound(next, samples.cend(), target,
                                            [](const QPointF &sample, double timestamp) {
                                                return sample.x() < timestamp;
                                            });
        if (after == samples.cend()) {
            break;
        }

        auto chosen = after;
        if (after != next) {
            const auto before = after - 1;
            if (std::abs(before->x() - target) <= std::abs(after->x() - target)) {
                chosen = before;
            }
        }
        if (chosen->x() >= _exportRange.end - 1e-6) {
            break;
        }

        const int index = static_cast<int>(std::distance(samples.cbegin(), chosen));
        if (selected.isEmpty() || selected.constLast() != index) {
            selected.append(index);
        }
        next = chosen + 1;
        target += kOutputIntervalSeconds;
    }
    return selected;
}

QVector<CompetitionDataController::ExportRow> CompetitionDataController::_generateRows(int previewLimit) const
{
    QVector<ExportRow> rows;
    const QVector<int> gpsIndices = _selectedGpsIndices();
    if (gpsIndices.isEmpty()) {
        return rows;
    }

    const int maximumRows = previewLimit >= 0 ? previewLimit : gpsIndices.size();
    rows.reserve(std::min(maximumRows, static_cast<int>(gpsIndices.size())));

    const QVector<QPointF> &latitude = _samples(_gpsFields.latitude);
    const QVector<QPointF> &longitude = _samples(_gpsFields.longitude);
    const QVector<QPointF> &altitude = _samples(_gpsFields.altitude);
    const QVector<QPointF> &utcTime = _samples(_gpsFields.utcTime);
    const QVector<QPointF> &fixType = _samples(_gpsFields.fixType);
    const QVector<QPointF> &navState = _samples(kNavStateField);
    const QVector<QPointF> &missionSequence = _samples(kMissionSequenceField);

    for (const int gpsIndex : gpsIndices) {
        if (rows.size() >= maximumRows) {
            break;
        }
        const double timestamp = latitude[gpsIndex].x();
        const double utcUsec = _nearestValue(utcTime, timestamp);
        const double latitudeValue = _nearestValue(latitude, timestamp);
        const double longitudeValue = _nearestValue(longitude, timestamp);
        const double altitudeValue = _nearestValue(altitude, timestamp);
        const double fix = fixType.isEmpty() ? 3.0 : _nearestValue(fixType, timestamp);
        const double ax = _nearestValue(_samples(_accelerationFields.x), timestamp);
        const double ay = _nearestValue(_samples(_accelerationFields.y), timestamp);
        const double az = _nearestValue(_samples(_accelerationFields.z), timestamp);
        const std::array<double, 3> attitude = _eulerDegrees(timestamp);

        if (!std::isfinite(utcUsec) || utcUsec <= 0.0 || !std::isfinite(latitudeValue)
            || !std::isfinite(longitudeValue) || !std::isfinite(altitudeValue)
            || !std::isfinite(fix) || qRound(fix) < 3 || !std::isfinite(ax)
            || !std::isfinite(ay) || !std::isfinite(az) || !std::isfinite(attitude[0])
            || !std::isfinite(attitude[1]) || !std::isfinite(attitude[2])) {
            return {};
        }

        ExportRow row;
        row.automatic = _automaticFlag(qRound(_heldValue(navState, timestamp, 0.0)));
        row.waypoint = qRound(_heldValue(missionSequence, timestamp, -1.0));
        row.gpst = _gpst(utcUsec / 1e6);
        row.latitude = latitudeValue;
        row.longitude = longitudeValue;
        row.altitude = altitudeValue;
        row.ax = ax;
        row.ay = ay;
        row.az = az;
        row.roll = attitude[0];
        row.pitch = attitude[1];
        row.yaw = attitude[2];
        rows.append(row);
    }
    return rows;
}

double CompetitionDataController::_nearestValue(const QVector<QPointF> &samples, double timestamp) const
{
    if (samples.isEmpty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto after = std::lower_bound(samples.cbegin(), samples.cend(), timestamp,
                                        [](const QPointF &sample, double time) {
                                            return sample.x() < time;
                                        });
    auto nearest = after;
    if (after == samples.cend()) {
        nearest = samples.cend() - 1;
    } else if (after != samples.cbegin()) {
        const auto before = after - 1;
        if (std::abs(before->x() - timestamp) <= std::abs(after->x() - timestamp)) {
            nearest = before;
        }
    }
    if (std::abs(nearest->x() - timestamp) > kMaximumAuxiliaryAgeSeconds) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return nearest->y();
}

double CompetitionDataController::_heldValue(const QVector<QPointF> &samples, double timestamp, double fallback) const
{
    if (samples.isEmpty()) {
        return fallback;
    }
    const auto next = std::upper_bound(samples.cbegin(), samples.cend(), timestamp,
                                       [](double time, const QPointF &sample) {
                                           return time < sample.x();
                                       });
    return next == samples.cbegin() ? fallback : (next - 1)->y();
}

double CompetitionDataController::_gpst(double utcSeconds) const
{
    return utcSeconds - static_cast<double>(kGpsEpochUnixSeconds) + kGpsUtcOffsetSeconds;
}

std::array<double, 3> CompetitionDataController::_eulerDegrees(double timestamp) const
{
    const double w = _nearestValue(_samples(_attitudeFields.w), timestamp);
    const double x = _nearestValue(_samples(_attitudeFields.x), timestamp);
    const double y = _nearestValue(_samples(_attitudeFields.y), timestamp);
    const double z = _nearestValue(_samples(_attitudeFields.z), timestamp);
    if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan};
    }

    const double norm = std::sqrt((w * w) + (x * x) + (y * y) + (z * z));
    if (norm <= std::numeric_limits<double>::epsilon()) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan};
    }

    const double qw = w / norm;
    const double qx = x / norm;
    const double qy = y / norm;
    const double qz = z / norm;
    const double roll = std::atan2(2.0 * ((qw * qx) + (qy * qz)),
                                   1.0 - (2.0 * ((qx * qx) + (qy * qy))));
    const double pitchInput = std::clamp(2.0 * ((qw * qy) - (qz * qx)), -1.0, 1.0);
    const double pitch = std::asin(pitchInput);
    const double yaw = std::atan2(2.0 * ((qw * qz) + (qx * qy)),
                                  1.0 - (2.0 * ((qy * qy) + (qz * qz))));
    constexpr double radiansToDegrees = 180.0 / std::numbers::pi;
    return {roll * radiansToDegrees, pitch * radiansToDegrees, yaw * radiansToDegrees};
}

int CompetitionDataController::_automaticFlag(int navState) const
{
    switch (navState) {
    case 0:  // Manual
    case 1:  // Altitude
    case 2:  // Position
    case 6:  // Position Slow
    case 8:  // Altitude Cruise
    case 9:  // Rattitude (older PX4)
    case 10: // Acro
    case 15: // Stabilized
        return 0;
    default:
        return 1;
    }
}

QString CompetitionDataController::_eventLabel(const QString &eventName, int number, double timestamp) const
{
    const int totalSeconds = std::max(0, qRound(timestamp));
    const int hours = totalSeconds / 3600;
    const int minutes = (totalSeconds % 3600) / 60;
    const int seconds = totalSeconds % 60;
    return tr("%1 %2 — %3:%4:%5 (log %6 s)")
        .arg(eventName)
        .arg(number)
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0'))
        .arg(timestamp, 0, 'f', 3);
}

QString CompetitionDataController::_localPath(const QString &filePath) const
{
    const QUrl url(filePath);
    return url.isLocalFile() ? url.toLocalFile() : filePath;
}

QString CompetitionDataController::_rowText(const ExportRow &row) const
{
    return QStringLiteral("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\t%9\t%10\t%11\t%12")
        .arg(row.automatic)
        .arg(row.waypoint)
        .arg(row.gpst, 0, 'f', 3)
        .arg(row.latitude, 0, 'f', 6)
        .arg(row.longitude, 0, 'f', 6)
        .arg(row.altitude, 0, 'f', 1)
        .arg(row.ax, 0, 'f', 3)
        .arg(row.ay, 0, 'f', 3)
        .arg(row.az, 0, 'f', 3)
        .arg(row.roll, 0, 'f', 2)
        .arg(row.pitch, 0, 'f', 2)
        .arg(row.yaw, 0, 'f', 2);
}

void CompetitionDataController::_setError(const QString &message)
{
    _ready = false;
    _errorMessage = message;
    emit resultChanged();
}
