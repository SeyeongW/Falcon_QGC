#pragma once

#include <QtCore/QObject>
#include <QtCore/QPointF>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QVariantList>
#include <QtCore/QVector>
#include <QtQmlIntegration/QtQmlIntegration>

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
    Q_PROPERTY(double sourceRateHz READ sourceRateHz NOTIFY resultChanged)
    Q_PROPERTY(double durationSeconds READ durationSeconds NOTIFY resultChanged)
    Q_PROPERTY(int rowCount READ rowCount NOTIFY resultChanged)
    Q_PROPERTY(QVariantList previewRows READ previewRows NOTIFY resultChanged)
    Q_PROPERTY(QString suggestedOutputPath READ suggestedOutputPath NOTIFY resultChanged)

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
    double sourceRateHz() const { return _sourceRateHz; }
    double durationSeconds() const { return _durationSeconds; }
    int rowCount() const { return _rowCount; }
    QVariantList previewRows() const { return _previewRows; }
    QString suggestedOutputPath() const { return _suggestedOutputPath; }

    Q_INVOKABLE void loadULog(const QString &filePath);
    Q_INVOKABLE bool exportAscii(const QString &filePath);
    Q_INVOKABLE void clear();

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

    struct ExportRow {
        int automatic = 0;
        int waypoint = -1;
        double gpsTow = 0.0;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
    };

    void _parseFinished(const QString &filePath, bool ok, const QString &errorMessage);
    void _prepareResult();
    FieldSelection _selectGpsFields() const;
    const QVector<QPointF> &_samples(const QString &fieldName) const;
    TimeRange _armedRange() const;
    TimeRange _gpsRange(const FieldSelection &fields) const;
    QVector<ExportRow> _generateRows(int previewLimit = -1) const;
    double _interpolatedValue(const QVector<QPointF> &samples, double timestamp) const;
    double _heldValue(const QVector<QPointF> &samples, double timestamp, double fallback) const;
    double _gpsTow(double utcSeconds) const;
    int _automaticFlag(int navState) const;
    QString _localPath(const QString &filePath) const;
    QString _rowText(const ExportRow &row) const;
    void _setError(const QString &message);

    LogFileParser _parser;
    FieldSelection _gpsFields;
    TimeRange _exportRange;
    bool _ready = false;
    QString _sourcePath;
    QString _errorMessage;
    QString _warningMessage;
    QString _gpsSource;
    QString _suggestedOutputPath;
    double _sourceRateHz = 0.0;
    double _durationSeconds = 0.0;
    int _rowCount = 0;
    QVariantList _previewRows;
};
