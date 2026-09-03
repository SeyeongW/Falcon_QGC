#include "RosBridge.h"

#include <cmath>
#include <limits>

#include <QtCore/QDateTime>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QLoggingCategory>
#include <QtCore/QSet>
#include <QtCore/QVariantMap>

Q_LOGGING_CATEGORY(RosBridgeLog, "Custom.RosBridge")

namespace {
constexpr const char *kNodeName = "vtol_gcs";
constexpr const char *kImageType = "sensor_msgs/msg/Image";
constexpr const char *kCompressedType = "sensor_msgs/msg/CompressedImage";
constexpr qint64 kActuatorStaleMs = 1500;   ///< no RCOut for this long -> go static
constexpr qint64 kPhaseStaleMs = 3000;      ///< no command/status for this long -> link down
constexpr const char *kRunPhaseTopic = "command/run_phase";
constexpr const char *kResetPhaseTopic = "command/reset_phase";
constexpr const char *kCatalogTopic = "command/catalog";
constexpr const char *kStatusTopic = "command/status";
constexpr const char *kAbortTopic = "command/abort";
constexpr const char *kPhaseResponseTopic = "command/phase_response";
constexpr const char *kActionTopic = "command/run_action";
constexpr int kCatalogVersion = 1;
}

RosBridge::RosBridge(QObject *parent)
    : QObject(parent)
{
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
        _ownsContext = true;
    }

    _node = std::make_shared<rclcpp::Node>(kNodeName);
    _rosOk = true;
    emit rosOkChanged();

    // Spin the node on the GUI thread. ~30 Hz is plenty for a monitoring feed and
    // keeps GUI-thread wakeups low so the QGC UI stays smooth; spin_some drains
    // only ready work. (If a high-rate video feed ever needs it, move the spin to
    // a worker thread behind queued signals instead of raising this rate.)
    connect(&_spinTimer, &QTimer::timeout, this, &RosBridge::_spinOnce);
    _spinTimer.start(33);

    connect(&_fpsTimer, &QTimer::timeout, this, &RosBridge::_updateFps);
    _fpsTimer.start(1000);

    // Topic discovery is comparatively expensive; a slow re-scan is fine.
    connect(&_discoveryTimer, &QTimer::timeout, this, &RosBridge::refreshTopics);
    _discoveryTimer.start(5000);

    refreshTopics();
    setActuatorTopic(_actuatorTopic);   // subscribe to the default MAVROS RCOut topic

    // Mission phase orchestrator: publish run requests and subscribe to the
    // onboard MC's dynamic catalog plus live execution status.
    _runPhasePub = _node->create_publisher<std_msgs::msg::Int32>(kRunPhaseTopic, 10);
    _resetPhasePub = _node->create_publisher<std_msgs::msg::Int32>(kResetPhaseTopic, 10);
    _abortPub = _node->create_publisher<std_msgs::msg::Empty>(kAbortTopic, 10);
    _phaseResponsePub = _node->create_publisher<std_msgs::msg::String>(kPhaseResponseTopic, 10);
    _actionPub = _node->create_publisher<std_msgs::msg::String>(kActionTopic, 10);
    _subscribePhaseTopics();

    qCDebug(RosBridgeLog) << "RosBridge up, node" << kNodeName;
}

RosBridge::~RosBridge()
{
    _spinTimer.stop();
    _fpsTimer.stop();
    _discoveryTimer.stop();
    _imageSub.reset();
    _compressedSub.reset();
    _actuatorSub.reset();
    _phaseCatalogSub.reset();
    _phaseStatusSub.reset();
    _runPhasePub.reset();
    _resetPhasePub.reset();
    _abortPub.reset();
    _phaseResponsePub.reset();
    _actionPub.reset();
    _node.reset();
    if (_ownsContext && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

RosBridge *RosBridge::instance()
{
    static RosBridge *s_instance = new RosBridge();
    return s_instance;
}

void RosBridge::_spinOnce()
{
    if (rclcpp::ok() && _node) {
        rclcpp::spin_some(_node);
    }
}

void RosBridge::_updateFps()
{
    if (_frameCounter != _imageFps) {
        _imageFps = _frameCounter;
        emit imageFpsChanged();
    }
    if (_frameCounter > 0) {
        qCDebug(RosBridgeLog) << _imageTopic << _imageFps << "fps";
    }
    _frameCounter = 0;

    // Expire actuator data so the control-surface widget goes static when MAVROS
    // stops publishing (rather than freezing on the last values).
    if (_haveActuator && (QDateTime::currentMSecsSinceEpoch() - _lastActuatorMs) > kActuatorStaleMs) {
        _haveActuator = false;
        emit servoChannelsChanged();
    }

    // Mark the orchestrator link down if command/status has gone quiet (the
    // orchestrator republishes at ~2 Hz, so 3 s of silence means it's gone).
    if (_phaseLinkOk && (QDateTime::currentMSecsSinceEpoch() - _lastPhaseMs) > kPhaseStaleMs) {
        _phaseLinkOk = false;
        emit phaseStatusChanged();
    }
}

void RosBridge::refreshTopics()
{
    if (!_node) {
        return;
    }

    QStringList found;
    const auto topics = _node->get_topic_names_and_types();
    for (const auto &[name, types] : topics) {
        for (const auto &type : types) {
            if (type == kImageType || type == kCompressedType) {
                found.append(QString::fromStdString(name));
                break;
            }
        }
    }
    found.sort();

    if (found != _imageTopics) {
        _imageTopics = found;
        emit imageTopicsChanged();
    }
}

void RosBridge::setImageTopic(const QString &topic)
{
    if (topic == _imageTopic) {
        return;
    }
    _imageTopic = topic;
    emit imageTopicChanged();

    _imageSub.reset();
    _compressedSub.reset();
    _frameCounter = 0;

    if (!_node || topic.isEmpty()) {
        return;
    }

    // A CompressedImage topic (e.g. `/cam/compressed`, published by
    // image_transport republish) is preferred over the network since it dodges
    // the fragmentation loss that kills large raw frames. Decide from the graph
    // which message type this topic carries, then subscribe accordingly.
    bool compressed = false;
    const auto topics = _node->get_topic_names_and_types();
    const auto it = topics.find(topic.toStdString());
    if (it != topics.end()) {
        for (const auto &type : it->second) {
            if (type == kCompressedType) {
                compressed = true;
                break;
            }
        }
    }

    // Sensor data QoS (best-effort): compatible with both reliable and
    // best-effort publishers, and fine for a monitoring feed.
    if (compressed) {
        _compressedSub = _node->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic.toStdString(), rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) { _onCompressedImage(msg); });
        qCDebug(RosBridgeLog) << "subscribed (compressed) to" << topic;
    } else {
        _imageSub = _node->create_subscription<sensor_msgs::msg::Image>(
            topic.toStdString(), rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { _onImage(msg); });
        qCDebug(RosBridgeLog) << "subscribed (raw) to" << topic;
    }
}

void RosBridge::_onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    ++_frameCounter;
    const QImage image = toQImage(*msg);
    if (!image.isNull()) {
        emit frameReceived(image);
    }
}

void RosBridge::_onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
{
    ++_frameCounter;
    // CompressedImage carries an encoded blob (jpeg/png). QImage::loadFromData
    // auto-detects the format via the bundled Qt image plugins.
    QImage image;
    if (image.loadFromData(msg->data.data(), static_cast<int>(msg->data.size()))) {
        emit frameReceived(image);
    } else {
        qCWarning(RosBridgeLog) << "failed to decode compressed image (" << msg->format.c_str() << ")";
    }
}

void RosBridge::setActuatorTopic(const QString &topic)
{
    const bool topicChanged = (topic != _actuatorTopic);
    _actuatorTopic = topic;
    if (topicChanged) {
        emit actuatorTopicChanged();
    }

    _actuatorSub.reset();
    if (_haveActuator) {
        _haveActuator = false;
        emit servoChannelsChanged();
    }

    if (!_node || topic.isEmpty()) {
        return;
    }

    _actuatorSub = _node->create_subscription<mavros_msgs::msg::RCOut>(
        topic.toStdString(), rclcpp::SensorDataQoS(),
        [this](const mavros_msgs::msg::RCOut::ConstSharedPtr &msg) { _onActuator(msg); });

    qCDebug(RosBridgeLog) << "actuator subscribed to" << topic;
}

void RosBridge::_onActuator(const mavros_msgs::msg::RCOut::ConstSharedPtr &msg)
{
    QVariantList channels;
    channels.reserve(static_cast<int>(msg->channels.size()));
    for (const uint16_t pwm : msg->channels) {
        channels.append(static_cast<int>(pwm));
    }
    _servoChannels = channels;
    _lastActuatorMs = QDateTime::currentMSecsSinceEpoch();
    _haveActuator = !channels.isEmpty();
    emit servoChannelsChanged();
}

void RosBridge::runPhase(int n)
{
    if (!_runPhasePub) {
        qCWarning(RosBridgeLog) << "runPhase" << n << "ignored: no publisher";
        return;
    }
    std_msgs::msg::Int32 msg;
    msg.data = n;
    _runPhasePub->publish(msg);
    qCDebug(RosBridgeLog) << "runPhase" << n << "published";
}

void RosBridge::resetPhase(int n)
{
    if (!_resetPhasePub) {
        qCWarning(RosBridgeLog) << "resetPhase" << n << "ignored: no publisher";
        return;
    }
    std_msgs::msg::Int32 msg;
    msg.data = n;
    _resetPhasePub->publish(msg);
    qCDebug(RosBridgeLog) << "resetPhase" << n << "published";
}

void RosBridge::respondPhase(const QString &response)
{
    const QString normalizedResponse = response.trimmed().toLower();
    if (normalizedResponse != QStringLiteral("ok") && normalizedResponse != QStringLiteral("no")
        && normalizedResponse != QStringLiteral("position")
        && normalizedResponse != QStringLiteral("again")) {
        qCWarning(RosBridgeLog) << "invalid phase response" << response;
        return;
    }
    if (!_phaseResponsePub) {
        qCWarning(RosBridgeLog) << "phase response ignored: no publisher";
        return;
    }

    std_msgs::msg::String msg;
    msg.data = normalizedResponse.toStdString();
    _phaseResponsePub->publish(msg);
    qCDebug(RosBridgeLog) << "phase response" << normalizedResponse << "published";
}

void RosBridge::setCameraEnabled(bool enabled)
{
    if (!_actionPub) {
        qCWarning(RosBridgeLog) << "camera action ignored: no publisher";
        return;
    }

    std_msgs::msg::String msg;
    msg.data = enabled ? "camera:on" : "camera:off";
    _actionPub->publish(msg);
    qCDebug(RosBridgeLog) << QString::fromStdString(msg.data) << "published";
}

void RosBridge::runGripper(const QString &action)
{
    const QString normalizedAction = action.trimmed().toLower();
    if (normalizedAction != QStringLiteral("open") && normalizedAction != QStringLiteral("close")
        && normalizedAction != QStringLiteral("stop")) {
        qCWarning(RosBridgeLog) << "invalid gripper action" << action;
        return;
    }
    if (!_actionPub) {
        qCWarning(RosBridgeLog) << "gripper action ignored: no publisher";
        return;
    }

    std_msgs::msg::String msg;
    msg.data = QStringLiteral("gripper:%1").arg(normalizedAction).toStdString();
    _actionPub->publish(msg);
    qCDebug(RosBridgeLog) << QString::fromStdString(msg.data) << "published";
}

void RosBridge::runFailsafe()
{
    if (!_actionPub) {
        qCWarning(RosBridgeLog) << "failsafe action ignored: no publisher";
        return;
    }

    std_msgs::msg::String msg;
    msg.data = "failsafe:run";
    _actionPub->publish(msg);
    qCDebug(RosBridgeLog) << "failsafe:run published";
}

void RosBridge::abortMission()
{
    if (!_abortPub) {
        qCWarning(RosBridgeLog) << "abortMission ignored: no publisher";
        return;
    }
    _abortPub->publish(std_msgs::msg::Empty{});
    qCDebug(RosBridgeLog) << "abortMission (command/abort) published";
}

void RosBridge::_subscribePhaseTopics()
{
    _phaseCatalogSub.reset();
    _phaseStatusSub.reset();

    // The MC publishes the catalog with transient-local durability. Request the
    // same QoS so QGC receives the latest catalog even when it starts later.
    const auto catalogQos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    _phaseCatalogSub = _node->create_subscription<std_msgs::msg::String>(
        kCatalogTopic, catalogQos,
        [this](const std_msgs::msg::String::ConstSharedPtr &msg) { _onPhaseCatalog(msg); });
    _phaseStatusSub = _node->create_subscription<std_msgs::msg::String>(
        kStatusTopic, 10,
        [this](const std_msgs::msg::String::ConstSharedPtr &msg) { _onPhaseStatus(msg); });
}

void RosBridge::retryPhaseLink()
{
    if (!_node) {
        return;
    }
    // Drop and re-create both subscriptions so DDS re-discovers the onboard MC.
    _subscribePhaseTopics();

    if (_phaseLinkOk) {
        _phaseLinkOk = false;
        emit phaseStatusChanged();
    }
    qCDebug(RosBridgeLog) << "phase link retry: re-subscribed to"
                         << kCatalogTopic << "and" << kStatusTopic;
}

void RosBridge::_onPhaseCatalog(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if (!doc.isObject()) {
        qCWarning(RosBridgeLog) << "bad command/catalog payload: root is not an object";
        return;
    }

    const QJsonObject root = doc.object();
    if (root.value(QStringLiteral("version")).toInt(-1) != kCatalogVersion) {
        qCWarning(RosBridgeLog) << "unsupported command/catalog version"
                                << root.value(QStringLiteral("version"));
        return;
    }

    const QJsonValue phasesValue = root.value(QStringLiteral("phases"));
    if (!phasesValue.isArray()) {
        qCWarning(RosBridgeLog) << "bad command/catalog payload: phases is not an array";
        return;
    }

    QVariantList catalog;
    QSet<int> phaseIds;
    for (const QJsonValue &value : phasesValue.toArray()) {
        if (!value.isObject()) {
            qCWarning(RosBridgeLog) << "bad command/catalog payload: phase is not an object";
            return;
        }

        const QJsonObject phaseObject = value.toObject();
        const QJsonValue idValue = phaseObject.value(QStringLiteral("id"));
        const double idNumber = idValue.toDouble(-1.0);
        if (!idValue.isDouble() || !std::isfinite(idNumber) || std::floor(idNumber) != idNumber
            || idNumber < 0.0 || idNumber > std::numeric_limits<int>::max()) {
            qCWarning(RosBridgeLog) << "bad command/catalog phase id" << idValue;
            return;
        }

        const int phaseId = static_cast<int>(idNumber);
        if (phaseIds.contains(phaseId)) {
            qCWarning(RosBridgeLog) << "duplicate command/catalog phase id" << phaseId;
            return;
        }

        const QJsonValue titleValue = phaseObject.value(QStringLiteral("title"));
        if (!titleValue.isString() || titleValue.toString().trimmed().isEmpty()) {
            qCWarning(RosBridgeLog) << "bad command/catalog title for phase" << phaseId;
            return;
        }

        const QJsonValue descValue = phaseObject.value(QStringLiteral("desc"));
        if (!descValue.isUndefined() && !descValue.isString()) {
            qCWarning(RosBridgeLog) << "bad command/catalog description for phase" << phaseId;
            return;
        }

        const QJsonValue independentValue = phaseObject.value(QStringLiteral("independent"));
        if (!independentValue.isUndefined() && !independentValue.isBool()) {
            qCWarning(RosBridgeLog) << "bad command/catalog independent flag for phase" << phaseId;
            return;
        }

        const QJsonValue availableValue = phaseObject.value(QStringLiteral("available"));
        if (!availableValue.isUndefined() && !availableValue.isBool()) {
            qCWarning(RosBridgeLog) << "bad command/catalog available flag for phase" << phaseId;
            return;
        }

        QVariantMap phase;
        phase.insert(QStringLiteral("id"), phaseId);
        phase.insert(QStringLiteral("title"), titleValue.toString().trimmed());
        phase.insert(QStringLiteral("desc"), descValue.toString());
        phase.insert(QStringLiteral("independent"), independentValue.toBool(true));
        phase.insert(QStringLiteral("available"), availableValue.toBool(true));
        catalog.append(phase);
        phaseIds.insert(phaseId);
    }

    if (_phaseCatalog != catalog) {
        _phaseCatalog = catalog;
        emit phaseCatalogChanged();
    }
}

void RosBridge::_onPhaseStatus(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if (!doc.isObject()) {
        qCWarning(RosBridgeLog) << "bad command/status payload" << QString::fromStdString(msg->data);
        return;
    }
    const QJsonObject obj = doc.object();

    _phase = obj.value(QStringLiteral("phase")).toInt(-1);
    _phaseState = obj.value(QStringLiteral("state")).toString(QStringLiteral("idle"));
    _phaseMsg = obj.value(QStringLiteral("msg")).toString();
    _phasePrompt = obj.value(QStringLiteral("prompt")).toString();
    _phaseProgress = obj.value(QStringLiteral("progress")).toDouble(-1.0);

    QVariantList done;
    for (const QJsonValue &v : obj.value(QStringLiteral("done")).toArray()) {
        done.append(v.toInt());
    }
    _phaseDone = done;

    const QJsonObject actions = obj.value(QStringLiteral("actions")).toObject();
    _cameraAvailable = actions.value(QStringLiteral("camera_available")).toBool(false);
    _cameraRunning = actions.value(QStringLiteral("camera_running")).toBool(false);
    _gripperOpenAvailable = actions.value(QStringLiteral("gripper_open_available")).toBool(false);
    _gripperCloseAvailable = actions.value(QStringLiteral("gripper_close_available")).toBool(false);
    _gripperBusy = actions.value(QStringLiteral("gripper_busy")).toBool(false);
    _gripperState = actions.value(QStringLiteral("gripper_state")).toString(QStringLiteral("unknown"));
    _failsafeAvailable = actions.value(QStringLiteral("failsafe_available")).toBool(false);
    _failsafeRunning = actions.value(QStringLiteral("failsafe_running")).toBool(false);
    _actionMsg = actions.value(QStringLiteral("msg")).toString();

    _lastPhaseMs = QDateTime::currentMSecsSinceEpoch();
    _phaseLinkOk = true;
    emit phaseStatusChanged();
}

QImage RosBridge::toQImage(const sensor_msgs::msg::Image &msg)
{
    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);
    if (w <= 0 || h <= 0 || msg.data.empty()) {
        return QImage();
    }

    const auto wrap = [&](QImage::Format fmt) {
        // QImage does not take ownership of msg.data, so return a deep copy that
        // outlives the message. bytesPerLine preserves any row padding (step).
        return QImage(msg.data.data(), w, h, static_cast<qsizetype>(msg.step), fmt).copy();
    };

    const std::string &enc = msg.encoding;
    if (enc == "rgb8") {
        return wrap(QImage::Format_RGB888);
    }
    if (enc == "bgr8") {
        return wrap(QImage::Format_BGR888);
    }
    if (enc == "rgba8") {
        return wrap(QImage::Format_RGBA8888);
    }
    if (enc == "bgra8") {
        return wrap(QImage::Format_RGBA8888).rgbSwapped();
    }
    if (enc == "mono8" || enc == "8UC1") {
        return wrap(QImage::Format_Grayscale8);
    }
    if (enc == "mono16" || enc == "16UC1") {
        return wrap(QImage::Format_Grayscale16);
    }

    qCWarning(RosBridgeLog) << "unsupported image encoding" << QString::fromStdString(enc);
    return QImage();
}
