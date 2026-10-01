#include "CompetitionDataControllerTest.h"

#include "CompetitionDataController.h"

#include <QtCore/QFile>
#include <QtCore/QTemporaryDir>

#include <cstring>
#include <vector>

#include <ulog_cpp/messages.hpp>
#include <ulog_cpp/writer.hpp>

namespace {

template<typename T>
void appendValue(std::vector<uint8_t> &payload, const T &value)
{
    const size_t offset = payload.size();
    payload.resize(offset + sizeof(T));
    memcpy(payload.data() + offset, &value, sizeof(T));
}

QByteArray competitionULog(uint64_t gpsIntervalUsec = 100000ULL)
{
    std::vector<uint8_t> buffer;
    ulog_cpp::Writer writer([&](const uint8_t *data, int length) {
        buffer.insert(buffer.end(), data, data + length);
    });

    writer.fileHeader(ulog_cpp::FileHeader{});
    writer.messageFormat(ulog_cpp::MessageFormat{
        "sensor_gps",
        {ulog_cpp::Field{"uint64_t", "timestamp"},
         ulog_cpp::Field{"double", "latitude_deg"},
         ulog_cpp::Field{"double", "longitude_deg"},
         ulog_cpp::Field{"float", "altitude_msl_m"},
         ulog_cpp::Field{"uint64_t", "time_utc_usec"},
         ulog_cpp::Field{"uint8_t", "fix_type"}}
    });
    writer.messageFormat(ulog_cpp::MessageFormat{
        "sensor_combined",
        {ulog_cpp::Field{"uint64_t", "timestamp"},
         ulog_cpp::Field{"float[3]", "accelerometer_m_s2"}}
    });
    writer.messageFormat(ulog_cpp::MessageFormat{
        "vehicle_attitude",
        {ulog_cpp::Field{"uint64_t", "timestamp"},
         ulog_cpp::Field{"float[4]", "q"}}
    });
    writer.messageFormat(ulog_cpp::MessageFormat{
        "vehicle_status",
        {ulog_cpp::Field{"uint64_t", "timestamp"},
         ulog_cpp::Field{"uint8_t", "arming_state"},
         ulog_cpp::Field{"uint8_t", "nav_state"}}
    });
    writer.messageFormat(ulog_cpp::MessageFormat{
        "mission_result",
        {ulog_cpp::Field{"uint64_t", "timestamp"},
         ulog_cpp::Field{"int32_t", "seq_current"}}
    });
    writer.headerComplete();

    writer.addLoggedMessage(ulog_cpp::AddLoggedMessage{0, 1, "sensor_gps"});
    writer.addLoggedMessage(ulog_cpp::AddLoggedMessage{0, 2, "sensor_combined"});
    writer.addLoggedMessage(ulog_cpp::AddLoggedMessage{0, 3, "vehicle_attitude"});
    writer.addLoggedMessage(ulog_cpp::AddLoggedMessage{0, 4, "vehicle_status"});
    writer.addLoggedMessage(ulog_cpp::AddLoggedMessage{0, 5, "mission_result"});

    constexpr uint64_t utcBaseUsec = 1700000000000000ULL;
    for (int index = 0; ; ++index) {
        const uint64_t timestamp = 1000000ULL + (static_cast<uint64_t>(index) * gpsIntervalUsec);
        if (timestamp > 4000000ULL) {
            break;
        }

        std::vector<uint8_t> gps;
        appendValue(gps, timestamp);
        appendValue(gps, 37.600000 + (index * 0.000001));
        appendValue(gps, 126.800000 + (index * 0.000001));
        appendValue(gps, 180.0F);
        appendValue(gps, utcBaseUsec + timestamp);
        appendValue(gps, static_cast<uint8_t>(3));
        writer.data(ulog_cpp::Data{1, gps});

        std::vector<uint8_t> acceleration;
        appendValue(acceleration, timestamp);
        appendValue(acceleration, 1.0F);
        appendValue(acceleration, 2.0F);
        appendValue(acceleration, 9.81F);
        writer.data(ulog_cpp::Data{2, acceleration});

        std::vector<uint8_t> attitude;
        appendValue(attitude, timestamp);
        appendValue(attitude, 1.0F);
        appendValue(attitude, 0.0F);
        appendValue(attitude, 0.0F);
        appendValue(attitude, 0.0F);
        writer.data(ulog_cpp::Data{3, attitude});
    }

    const auto writeVehicleStatus = [&writer](uint64_t timestamp, uint8_t armingState) {
        std::vector<uint8_t> status;
        appendValue(status, timestamp);
        appendValue(status, armingState);
        appendValue(status, static_cast<uint8_t>(4));
        writer.data(ulog_cpp::Data{4, status});
    };
    writeVehicleStatus(900000ULL, 1);
    writeVehicleStatus(1000000ULL, 2);
    writeVehicleStatus(2000000ULL, 1);
    writeVehicleStatus(3000000ULL, 2);
    writeVehicleStatus(4000000ULL, 1);

    std::vector<uint8_t> mission;
    appendValue(mission, 900000ULL);
    appendValue(mission, static_cast<int32_t>(2));
    writer.data(ulog_cpp::Data{5, mission});

    return QByteArray(reinterpret_cast<const char *>(buffer.data()), static_cast<int>(buffer.size()));
}

} // namespace

void CompetitionDataControllerTest::_selectedArmDisarmRangeTest()
{
    QTemporaryDir temporaryDirectory;
    QVERIFY(temporaryDirectory.isValid());

    const QString logPath = temporaryDirectory.filePath(QStringLiteral("competition.ulg"));
    QFile logFile(logPath);
    QVERIFY(logFile.open(QIODevice::WriteOnly));
    const QByteArray logBytes = competitionULog();
    QCOMPARE(logFile.write(logBytes), logBytes.size());
    logFile.close();

    CompetitionDataController controller;
    controller.loadULog(logPath);
    QTRY_VERIFY_WITH_TIMEOUT(!controller.parsing(), 10000);

    QVERIFY2(controller.ready(), qPrintable(controller.errorMessage()));
    QCOMPARE(controller.armEvents().size(), 2);
    QCOMPARE(controller.disarmEvents().size(), 2);
    QCOMPARE(controller.selectedArmIndex(), 0);
    QCOMPARE(controller.selectedDisarmIndex(), 1);
    QCOMPARE(controller.rowCount(), 30);

    controller.setSelectedArmIndex(1);
    controller.setSelectedDisarmIndex(0);
    QVERIFY(!controller.ready());

    controller.setSelectedDisarmIndex(1);
    QVERIFY2(controller.ready(), qPrintable(controller.errorMessage()));
    QCOMPARE(controller.rowCount(), 10);

    const QString outputPath = temporaryDirectory.filePath(QStringLiteral("competition.txt"));
    QVERIFY(controller.exportAscii(outputPath));
    QFile outputFile(outputPath);
    QVERIFY(outputFile.open(QIODevice::ReadOnly | QIODevice::Text));
    QCOMPARE(outputFile.readLine().trimmed(),
             QByteArray("AUTO_MANUAL\tEVENT\tGPST\tLATITUDE\tLONGITUDE\tALTITUDE\tAX\tAY\tAZ\tROLL\tPITCH\tYAW"));
    QCOMPARE(outputFile.readLine().trimmed().split('\t').size(), 12);
}

void CompetitionDataControllerTest::_lowRateRejectedTest()
{
    QTemporaryDir temporaryDirectory;
    QVERIFY(temporaryDirectory.isValid());

    const QString logPath = temporaryDirectory.filePath(QStringLiteral("low_rate.ulg"));
    QFile logFile(logPath);
    QVERIFY(logFile.open(QIODevice::WriteOnly));
    const QByteArray logBytes = competitionULog(125000ULL);
    QCOMPARE(logFile.write(logBytes), logBytes.size());
    logFile.close();

    CompetitionDataController controller;
    controller.loadULog(logPath);
    QTRY_VERIFY_WITH_TIMEOUT(!controller.parsing(), 10000);

    QVERIFY(!controller.ready());
    QVERIFY(controller.errorMessage().contains(QStringLiteral("native GPS data at 10 Hz")));
}

UT_REGISTER_TEST(CompetitionDataControllerTest, TestLabel::Unit, TestLabel::AnalyzeView)
