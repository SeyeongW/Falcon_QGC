#pragma once

#include <QtCore/QObject>
#include <QtCore/QPointF>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QVariantList>
#include <QtCore/QVector>
#include <QtQmlIntegration/QtQmlIntegration>

#include <array>

#include "LogFileParser.h"

class CompetitionDataController : public QObject
{
    Q_OBJECT
    QML_ELEMENT

    Q_PROPERTY(bool parsing READ parsing NOTIFY parsingChanged)
    Q_PROPERTY(float parseProgress READ parseProgress NOTIFY parseProgressChanged)
    Q_PROPERTY(bool ready READ ready NOTIFY resultChanged)
    Q_PROPERTY(QString sourcePath READ sourcePath NOTIFY resultChanged)
    Q_PROPERTY(QString errorMessage READ errorMessage NOTIFY resultChanged)
    Q_PROPERTY(QString warningMessage READ warningMessage NOTIFY resultChanged)
    Q_PROPERTY(QString gpsSource READ gpsSource NOTIFY resultChanged)
    Q_PROPERTY(QVariantList gpsSourceOptions READ gpsSourceOptions NOTIFY resultChanged)
    Q_PROPERTY(int selectedGpsSourceIndex READ selectedGpsSourceIndex WRITE setSelectedGpsSourceIndex NOTIFY resultChanged)
    Q_PROPERTY(QString imuSource READ imuSource NOTIFY resultChanged)
    Q_PROPERTY(QString attitudeSource READ attitudeSource NOTIFY resultChanged)
    Q_PROPERTY(double sourceRateHz READ sourceRateHz NOTIFY resultChanged)
    Q_PROPERTY(double durationSeconds READ durationSeconds NOTIFY resultChanged)
    Q_PROPERTY(int rowCount READ rowCount NOTIFY resultChanged)
    Q_PROPERTY(QVariantList previewRows READ previewRows NOTIFY resultChanged)
    Q_PROPERTY(QString suggestedOutputPath READ suggestedOutputPath NOTIFY resultChanged)
    Q_PROPERTY(QVariantList armEvents READ armEvents NOTIFY resultChanged)
    Q_PROPERTY(QVariantList disarmEvents READ disarmEvents NOTIFY resultChanged)
    Q_PROPERTY(int selectedArmIndex READ selectedArmIndex WRITE setSelectedArmIndex NOTIFY resultChanged)
    Q_PROPERTY(int selectedDisarmIndex READ selectedDisarmIndex WRITE setSelectedDisarmIndex NOTIFY resultChanged)

public:
    explicit CompetitionDataController(QObject *parent = nullptr);
    ~CompetitionDataController() override;

    bool parsing() const { return _parser.parsing(); }
    float parseProgress() const { return _parser.parseProgress(); }
    bool ready() const { return _ready; }
    QString sourcePath() const { return _sourcePath; }
    QString errorMessage() const { return _errorMessage; }
    QString warningMessage() const { return _warningMessage; }
    QString gpsSource() const { return _gpsSource; }
    QVariantList gpsSourceOptions() const { return _gpsSourceOptions; }
    int selectedGpsSourceIndex() const { return _selectedGpsSourceIndex; }
    QString imuSource() const { return _imuSource; }
    QString attitudeSource() const { return _attitudeSource; }
    double sourceRateHz() const { return _sourceRateHz; }
    double durationSeconds() const { return _durationSeconds; }
    int rowCount() const { return _rowCount; }
    QVariantList previewRows() const { return _previewRows; }
    QString suggestedOutputPath() const { return _suggestedOutputPath; }
    QVariantList armEvents() const { return _armEvents; }
    QVariantList disarmEvents() const { return _disarmEvents; }
    int selectedArmIndex() const { return _selectedArmIndex; }
    int selectedDisarmIndex() const { return _selectedDisarmIndex; }

    Q_INVOKABLE void loadULog(const QString &filePath);
    Q_INVOKABLE bool exportAscii(const QString &filePath);
    Q_INVOKABLE void clear();

    void setSelectedArmIndex(int index);
    void setSelectedDisarmIndex(int index);
    void setSelectedGpsSourceIndex(int index);

signals:
    void parsingChanged();
    void parseProgressChanged();
    void resultChanged();
    void exportFinished(const QString &filePath, bool ok, const QString &errorMessage);

private:
    struct FieldSelection {
        QString latitude;
        QString longitude;
        QString altitude;
        QString utcTime;
        QString fixType;
    };

    struct TimeRange {
        double start = 0.0;
        double end = 0.0;
        bool valid = false;
    };

    struct AxisSelection {
        QString x;
        QString y;
        QString z;
    };

    struct QuaternionSelection {
        QString w;
        QString x;
        QString y;
        QString z;
    };

    struct ExportRow {
        int automatic = 0;
        int waypoint = -1;
        double gpst = 0.0;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    void _parseFinished(const QString &filePath, bool ok, const QString &errorMessage);
    void _prepareResult();
    void _updateSelectedRange();
    void _detectArmDisarmEvents();
    FieldSelection _selectGpsFields() const;
    QVector<FieldSelection> _availableGpsFields() const;
    double _fieldRateHz(const FieldSelection &fields) const;
    AxisSelection _selectAccelerationFields() const;
    QuaternionSelection _selectAttitudeFields() const;
    const QVector<QPointF> &_samples(const QString &fieldName) const;
    TimeRange _gpsRange(const FieldSelection &fields) const;
    QVector<int> _selectedGpsIndices() const;
    QVector<ExportRow> _generateRows(int previewLimit = -1) const;
    double _nearestValue(const QVector<QPointF> &samples, double timestamp) const;
    double _heldValue(const QVector<QPointF> &samples, double timestamp, double fallback) const;
    double _gpst(double utcSeconds) const;
    std::array<double, 3> _eulerDegrees(double timestamp) const;
    int _automaticFlag(int navState) const;
    QString _eventLabel(const QString &eventName, int number, double timestamp) const;
    QString _localPath(const QString &filePath) const;
    QString _rowText(const ExportRow &row) const;
    void _setError(const QString &message);

    LogFileParser _parser;
    FieldSelection _gpsFields;
    AxisSelection _accelerationFields;
    QuaternionSelection _attitudeFields;
    TimeRange _exportRange;
    QVector<double> _armTimes;
    QVector<double> _disarmTimes;
    QVariantList _armEvents;
    QVariantList _disarmEvents;
    int _selectedArmIndex = -1;
    int _selectedDisarmIndex = -1;
    bool _dataPrepared = false;
    bool _ready = false;
    QString _sourcePath;
    QString _errorMessage;
    QString _warningMessage;
    QString _gpsSource;
    QVariantList _gpsSourceOptions;
    QVector<FieldSelection> _gpsCandidates;
    int _selectedGpsSourceIndex = -1;
    QString _imuSource;
    QString _attitudeSource;
    QString _suggestedOutputPath;
    double _sourceRateHz = 0.0;
    double _durationSeconds = 0.0;
    int _rowCount = 0;
    QVariantList _previewRows;
};
