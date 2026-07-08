#include "RosBridge.h"

#include <QtCore/QDateTime>
#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(RosBridgeLog, "Custom.RosBridge")

namespace {
constexpr const char *kNodeName = "vtol_gcs";
constexpr const char *kImageType = "sensor_msgs/msg/Image";
constexpr qint64 kActuatorStaleMs = 1500;   ///< no RCOut for this long -> go static
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

    qCDebug(RosBridgeLog) << "RosBridge up, node" << kNodeName;
}

RosBridge::~RosBridge()
{
    _spinTimer.stop();
    _fpsTimer.stop();
    _discoveryTimer.stop();
    _imageSub.reset();
    _actuatorSub.reset();
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
            if (type == kImageType) {
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
    _frameCounter = 0;

    if (!_node || topic.isEmpty()) {
        return;
    }

    // Sensor data QoS: best-effort matches typical camera publishers.
    _imageSub = _node->create_subscription<sensor_msgs::msg::Image>(
        topic.toStdString(), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { _onImage(msg); });

    qCDebug(RosBridgeLog) << "subscribed to" << topic;
}

void RosBridge::_onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    ++_frameCounter;
    const QImage image = toQImage(*msg);
    if (!image.isNull()) {
        emit frameReceived(image);
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
