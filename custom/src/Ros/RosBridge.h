#pragma once

#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtGui/QImage>

#include <memory>

#include <QtCore/QVariantList>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mavros_msgs/msg/rc_out.hpp>

/// Bridge between ROS2 and the VTOL-GCS QML layer.
///
/// Owns a single rclcpp node that is spun on the GUI thread via `spin_some`
/// (driven by a QTimer), so every callback and every property update happens
/// on the Qt thread — no cross-thread marshaling is required and QML bindings
/// stay correct by construction. This is adequate for a monitoring feed; if
/// image decode ever becomes a bottleneck the spin can be moved to its own
/// thread behind queued signals.
///
/// Phase 5 scope: image-topic discovery (rqt_image_view style), subscribing to
/// a selectable `sensor_msgs/Image` topic, converting frames to QImage, and
/// reporting the received frame rate. Mission-state / gripper topics are added
/// on top of this same node in later phases.
class RosBridge : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool rosOk READ rosOk NOTIFY rosOkChanged)
    Q_PROPERTY(QStringList imageTopics READ imageTopics NOTIFY imageTopicsChanged)
    Q_PROPERTY(QString imageTopic READ imageTopic WRITE setImageTopic NOTIFY imageTopicChanged)
    Q_PROPERTY(int imageFps READ imageFps NOTIFY imageFpsChanged)
    // Live actuator/servo outputs from MAVROS (mavros_msgs/RCOut, PWM per channel).
    // Drives the control-surface widget with the aircraft's real behavior.
    Q_PROPERTY(QString actuatorTopic READ actuatorTopic WRITE setActuatorTopic NOTIFY actuatorTopicChanged)
    Q_PROPERTY(bool haveActuator READ haveActuator NOTIFY servoChannelsChanged)
    Q_PROPERTY(QVariantList servoChannels READ servoChannels NOTIFY servoChannelsChanged)

public:
    explicit RosBridge(QObject *parent = nullptr);
    ~RosBridge() override;

    /// Process-wide singleton (registered with QML as `RosBridge`).
    static RosBridge *instance();

    bool rosOk() const { return _rosOk; }
    QStringList imageTopics() const { return _imageTopics; }
    QString imageTopic() const { return _imageTopic; }
    int imageFps() const { return _imageFps; }
    QString actuatorTopic() const { return _actuatorTopic; }
    bool haveActuator() const { return _haveActuator; }
    QVariantList servoChannels() const { return _servoChannels; }

    /// Convert a raw sensor_msgs/Image to QImage. Supports rgb8/bgr8/rgba8/
    /// bgra8/mono8/mono16; returns a null QImage for unsupported encodings.
    /// The returned image owns a deep copy of the message data.
    static QImage toQImage(const sensor_msgs::msg::Image &msg);

public slots:
    /// Re-scan the ROS graph for `sensor_msgs/msg/Image` publishers.
    void refreshTopics();
    /// Switch the active image subscription to `topic` (empty = unsubscribe).
    void setImageTopic(const QString &topic);
    /// Switch the MAVROS actuator/servo subscription (mavros_msgs/RCOut).
    void setActuatorTopic(const QString &topic);

signals:
    void rosOkChanged();
    void imageTopicsChanged();
    void imageTopicChanged();
    void imageFpsChanged();
    void actuatorTopicChanged();
    /// Emitted when actuator channels update or go stale (see haveActuator).
    void servoChannelsChanged();
    /// Emitted (on the GUI thread) each time a frame is decoded.
    void frameReceived(const QImage &image);

private:
    void _spinOnce();
    void _updateFps();
    void _onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void _onActuator(const mavros_msgs::msg::RCOut::ConstSharedPtr &msg);

    bool _rosOk = false;
    bool _ownsContext = false;                  ///< true if we called rclcpp::init

    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _imageSub;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr _actuatorSub;

    QTimer _spinTimer;                          ///< drives rclcpp::spin_some
    QTimer _fpsTimer;                            ///< 1 Hz frame-rate accounting
    QTimer _discoveryTimer;                      ///< periodic topic re-scan

    QString _imageTopic;
    QStringList _imageTopics;
    int _frameCounter = 0;
    int _imageFps = 0;

    QString _actuatorTopic = QStringLiteral("/mavros/rc/out");
    QVariantList _servoChannels;        ///< latest RCOut PWM channels
    bool _haveActuator = false;
    qint64 _lastActuatorMs = 0;         ///< monotonic ms of last actuator msg
};
