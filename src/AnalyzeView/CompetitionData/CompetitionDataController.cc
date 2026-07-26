#include "CompetitionDataController.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QSaveFile>
#include <QtCore/QTextStream>
#include <QtCore/QTimeZone>
#include <QtCore/QUrl>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr double kOutputIntervalSeconds = 0.1;
constexpr double kGpsWeekSeconds = 604800.0;
constexpr qint64 kGpsEpochUnixSeconds = 315964800;
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

    QSaveFile outputFile(localPath);
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        const QString message = tr("Failed to open the output file: %1").arg(outputFile.errorString());
        emit exportFinished(localPath, false, message);
        return false;
    }

    QTextStream stream(&outputFile);
    stream.setEncoding(QStringConverter::Utf8);
    stream << "AUTO_MANUAL,WAYPOINT,GPS_TIME,LATITUDE,LONGITUDE,ALTITUDE\n";
    const QVector<ExportRow> rows = _generateRows();
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
    _exportRange = {};
    _ready = false;
    _sourcePath.clear();
    _errorMessage.clear();
    _warningMessage.clear();
    _gpsSource.clear();
    _suggestedOutputPath.clear();
    _sourceRateHz = 0.0;
    _durationSeconds = 0.0;
    _rowCount = 0;
    _previewRows.clear();
    emit resultChanged();
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
    _gpsFields = _selectGpsFields();
    if (_gpsFields.latitude.isEmpty()) {
        _setError(tr("No supported raw GPS topic was found. Expected sensor_gps or vehicle_gps_position."));
        return;
    }
    _gpsSource = _gpsFields.latitude.section(QLatin1Char('.'), 0, 0);

    _exportRange = _gpsRange(_gpsFields);
    if (!_exportRange.valid) {
        _setError(tr("The log does not contain a usable GPS time range."));
        return;
    }

    QStringList warnings;
    const TimeRange armed = _armedRange();
    if (armed.valid) {
        _exportRange.start = std::max(_exportRange.start, armed.start);
        _exportRange.end = std::min(_exportRange.end, armed.end);
        _exportRange.valid = _exportRange.end >= _exportRange.start;
    } else {
        warnings.append(tr("No armed interval was found; the full valid GPS range will be exported."));
    }
    if (!_exportRange.valid) {
        _setError(tr("The armed interval does not overlap valid GPS data."));
        return;
    }

    const QVector<QPointF> &latitudeSamples = _samples(_gpsFields.latitude);
    if (latitudeSamples.size() > 1) {
        double intervalSum = 0.0;
        double maximumInterval = 0.0;
        int intervalCount = 0;
        for (int index = 1; index < latitudeSamples.size(); ++index) {
            const double interval = latitudeSamples[index].x() - latitudeSamples[index - 1].x();
            if (interval > 0.0) {
                intervalSum += interval;
                maximumInterval = std::max(maximumInterval, interval);
                ++intervalCount;
            }
        }
        if (intervalCount > 0) {
            _sourceRateHz = 1.0 / (intervalSum / intervalCount);
        }
        if (maximumInterval > 0.5) {
            warnings.append(tr("The GPS log contains a gap of %1 seconds; interpolated rows cross this gap.")
                                .arg(maximumInterval, 0, 'f', 2));
        } else if (_sourceRateHz < 9.5) {
            warnings.append(tr("The source GPS rate is %1 Hz; position is linearly interpolated to 10 Hz.")
                                .arg(_sourceRateHz, 0, 'f', 2));
        }
    }

    if (_samples(kNavStateField).isEmpty()) {
        warnings.append(tr("vehicle_status.nav_state is missing; AUTO_MANUAL defaults to manual (0)."));
    }
    if (_samples(kMissionSequenceField).isEmpty()) {
        warnings.append(tr("mission_result.seq_current is missing; WAYPOINT defaults to -1."));
    }

    const double firstOutput = std::ceil((_exportRange.start - 1e-9) / kOutputIntervalSeconds)
            * kOutputIntervalSeconds;
    const double lastOutput = std::floor((_exportRange.end + 1e-9) / kOutputIntervalSeconds)
            * kOutputIntervalSeconds;
    if (lastOutput < firstOutput) {
        _setError(tr("The valid flight interval is shorter than one 10 Hz sample."));
        return;
    }

    _durationSeconds = lastOutput - firstOutput;
    _rowCount = static_cast<int>(std::floor((_durationSeconds / kOutputIntervalSeconds) + 0.5)) + 1;
    const QVector<ExportRow> preview = _generateRows(kPreviewRowCount);
    for (const ExportRow &row : preview) {
        const QStringList columns = _rowText(row).split(QLatin1Char(','));
        QVariantMap previewRow;
        previewRow[QStringLiteral("automatic")] = columns.value(0);
        previewRow[QStringLiteral("waypoint")] = columns.value(1);
        previewRow[QStringLiteral("gpsTime")] = columns.value(2);
        previewRow[QStringLiteral("latitude")] = columns.value(3);
        previewRow[QStringLiteral("longitude")] = columns.value(4);
        previewRow[QStringLiteral("altitude")] = columns.value(5);
        _previewRows.append(previewRow);
    }

    const QFileInfo sourceInfo(_sourcePath);
    _suggestedOutputPath = sourceInfo.dir().filePath(sourceInfo.completeBaseName()
                                                     + QStringLiteral("_competition.csv"));
    _warningMessage = warnings.join(QLatin1Char('\n'));
    _ready = true;
    emit resultChanged();
}

CompetitionDataController::FieldSelection CompetitionDataController::_selectGpsFields() const
{
    struct Candidate {
        const char *topic;
        const char *latitude;
        const char *longitude;
        const char *altitude;
        const char *utcTime;
        const char *fixType;
    };

    static const Candidate candidates[] = {
        {"sensor_gps", "sensor_gps.latitude_deg", "sensor_gps.longitude_deg",
         "sensor_gps.altitude_msl_m", "sensor_gps.time_utc_usec", "sensor_gps.fix_type"},
        {"sensor_gps[0]", "sensor_gps[0].latitude_deg", "sensor_gps[0].longitude_deg",
         "sensor_gps[0].altitude_msl_m", "sensor_gps[0].time_utc_usec", "sensor_gps[0].fix_type"},
        {"vehicle_gps_position", "vehicle_gps_position.latitude_deg", "vehicle_gps_position.longitude_deg",
         "vehicle_gps_position.altitude_msl_m", "vehicle_gps_position.time_utc_usec", "vehicle_gps_position.fix_type"},
        {"vehicle_gps_position[0]", "vehicle_gps_position[0].latitude_deg", "vehicle_gps_position[0].longitude_deg",
         "vehicle_gps_position[0].altitude_msl_m", "vehicle_gps_position[0].time_utc_usec", "vehicle_gps_position[0].fix_type"},
    };

    for (const Candidate &candidate : candidates) {
        if (!_samples(QLatin1String(candidate.latitude)).isEmpty()
                && !_samples(QLatin1String(candidate.longitude)).isEmpty()
                && !_samples(QLatin1String(candidate.altitude)).isEmpty()) {
            FieldSelection fields;
            fields.latitude = QLatin1String(candidate.latitude);
            fields.longitude = QLatin1String(candidate.longitude);
            fields.altitude = QLatin1String(candidate.altitude);
            fields.utcTime = QLatin1String(candidate.utcTime);
            fields.fixType = QLatin1String(candidate.fixType);
            return fields;
        }
    }
    return {};
}

const QVector<QPointF> &CompetitionDataController::_samples(const QString &fieldName) const
{
    return _parser.fieldSamplesVector(fieldName);
}

CompetitionDataController::TimeRange CompetitionDataController::_armedRange() const
{
    const QStringList candidates = {
        QStringLiteral("vehicle_status.arming_state"),
        QStringLiteral("actuator_armed.armed"),
    };

    for (const QString &field : candidates) {
        const QVector<QPointF> &samples = _samples(field);
        if (samples.isEmpty()) {
            continue;
        }

        TimeRange longest;
        double intervalStart = 0.0;
        bool armed = false;
        for (const QPointF &sample : samples) {
            const bool sampleArmed = field.endsWith(QStringLiteral("arming_state"))
                    ? qRound(sample.y()) == 2
                    : sample.y() > 0.5;
            if (sampleArmed && !armed) {
                intervalStart = sample.x();
            } else if (!sampleArmed && armed) {
                const double duration = sample.x() - intervalStart;
                if (!longest.valid || duration > longest.end - longest.start) {
                    longest = {intervalStart, sample.x(), true};
                }
            }
            armed = sampleArmed;
        }
        if (armed) {
            const double intervalEnd = samples.constLast().x();
            if (!longest.valid || intervalEnd - intervalStart > longest.end - longest.start) {
                longest = {intervalStart, intervalEnd, true};
            }
        }
        if (longest.valid) {
            return longest;
        }
    }
    return {};
}

CompetitionDataController::TimeRange CompetitionDataController::_gpsRange(const FieldSelection &fields) const
{
    const QVector<QPointF> &latitude = _samples(fields.latitude);
    const QVector<QPointF> &longitude = _samples(fields.longitude);
    const QVector<QPointF> &altitude = _samples(fields.altitude);
    if (latitude.isEmpty() || longitude.isEmpty() || altitude.isEmpty()) {
        return {};
    }

    double start = std::max({latitude.constFirst().x(), longitude.constFirst().x(), altitude.constFirst().x()});
    double end = std::min({latitude.constLast().x(), longitude.constLast().x(), altitude.constLast().x()});
    const QVector<QPointF> &utcTime = _samples(fields.utcTime);
    if (!utcTime.isEmpty()) {
        start = std::max(start, utcTime.constFirst().x());
        end = std::min(end, utcTime.constLast().x());
    } else if (_parser.startTime().isNull()) {
        return {};
    }
    return {start, end, end >= start};
}

QVector<CompetitionDataController::ExportRow> CompetitionDataController::_generateRows(int previewLimit) const
{
    QVector<ExportRow> rows;
    if (!_exportRange.valid) {
        return rows;
    }

    const double firstOutput = std::ceil((_exportRange.start - 1e-9) / kOutputIntervalSeconds)
            * kOutputIntervalSeconds;
    const double lastOutput = std::floor((_exportRange.end + 1e-9) / kOutputIntervalSeconds)
            * kOutputIntervalSeconds;
    const int maximumRows = previewLimit >= 0 ? previewLimit : std::numeric_limits<int>::max();
    rows.reserve(previewLimit >= 0 ? previewLimit : _rowCount);

    const QVector<QPointF> &latitude = _samples(_gpsFields.latitude);
    const QVector<QPointF> &longitude = _samples(_gpsFields.longitude);
    const QVector<QPointF> &altitude = _samples(_gpsFields.altitude);
    const QVector<QPointF> &utcTime = _samples(_gpsFields.utcTime);
    const QVector<QPointF> &navState = _samples(kNavStateField);
    const QVector<QPointF> &missionSequence = _samples(kMissionSequenceField);

    for (double timestamp = firstOutput;
         timestamp <= lastOutput + 1e-6 && rows.size() < maximumRows;
         timestamp += kOutputIntervalSeconds) {
        const double utcSeconds = utcTime.isEmpty()
                ? static_cast<double>(_parser.startTime().toMSecsSinceEpoch()) / 1000.0 + timestamp
                : _interpolatedValue(utcTime, timestamp) / 1e6;
        ExportRow row;
        row.automatic = _automaticFlag(qRound(_heldValue(navState, timestamp, 0.0)));
        row.waypoint = qRound(_heldValue(missionSequence, timestamp, -1.0));
        row.gpsTow = _gpsTow(utcSeconds);
        row.latitude = _interpolatedValue(latitude, timestamp);
        row.longitude = _interpolatedValue(longitude, timestamp);
        row.altitude = _interpolatedValue(altitude, timestamp);
        rows.append(row);
    }
    return rows;
}

double CompetitionDataController::_interpolatedValue(const QVector<QPointF> &samples, double timestamp) const
{
    if (samples.isEmpty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto next = std::lower_bound(samples.cbegin(), samples.cend(), timestamp,
                                       [](const QPointF &sample, double time) {
                                           return sample.x() < time;
                                       });
    if (next == samples.cbegin()) {
        return next->y();
    }
    if (next == samples.cend()) {
        return samples.constLast().y();
    }
    const QPointF &before = *(next - 1);
    const double interval = next->x() - before.x();
    if (interval <= 0.0) {
        return before.y();
    }
    const double fraction = (timestamp - before.x()) / interval;
    return before.y() + ((next->y() - before.y()) * fraction);
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

double CompetitionDataController::_gpsTow(double utcSeconds) const
{
    // GPS is ahead of UTC by 18 seconds for all contemporary competition logs.
    // Keep this conversion isolated so the organizer's final GPS Time definition
    // can be applied without changing the resampling pipeline.
    constexpr double gpsUtcOffsetSeconds = 18.0;
    double tow = std::fmod(utcSeconds - static_cast<double>(kGpsEpochUnixSeconds)
                           + gpsUtcOffsetSeconds, kGpsWeekSeconds);
    if (tow < 0.0) {
        tow += kGpsWeekSeconds;
    }
    return tow;
}

int CompetitionDataController::_automaticFlag(int navState) const
{
    // PX4 modes in which the pilot directly commands motion through RC sticks.
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

QString CompetitionDataController::_localPath(const QString &filePath) const
{
    const QUrl url(filePath);
    return url.isLocalFile() ? url.toLocalFile() : filePath;
}

QString CompetitionDataController::_rowText(const ExportRow &row) const
{
    return QStringLiteral("%1,%2,%3,%4,%5,%6")
            .arg(row.automatic)
            .arg(row.waypoint)
            .arg(row.gpsTow, 0, 'f', 3)
            .arg(row.latitude, 0, 'f', 6)
            .arg(row.longitude, 0, 'f', 6)
            .arg(row.altitude, 0, 'f', 1);
}

void CompetitionDataController::_setError(const QString &message)
{
    _ready = false;
    _errorMessage = message;
    emit resultChanged();
}
