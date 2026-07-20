#pragma once

#include <QtCore/QObject>
#include <QtCore/QStringList>
#include <QtCore/QTimer>
#include <QtGui/QImage>

#include <memory>

#include <QtCore/QVariantList>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <mavros_msgs/msg/rc_out.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

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
    // Mission phase orchestrator (command/run_phase <-> command/status). The
    // orchestrator runs command/phaseN.py scripts sequentially and streams live
    // status here so the mission panel can gate buttons and show progress.
    Q_PROPERTY(bool phaseLinkOk READ phaseLinkOk NOTIFY phaseStatusChanged)
    Q_PROPERTY(int phase READ phase NOTIFY phaseStatusChanged)
    Q_PROPERTY(QString phaseState READ phaseState NOTIFY phaseStatusChanged)
    Q_PROPERTY(QString phaseMsg READ phaseMsg NOTIFY phaseStatusChanged)
    Q_PROPERTY(double phaseProgress READ phaseProgress NOTIFY phaseStatusChanged)
    Q_PROPERTY(QVariantList phaseDone READ phaseDone NOTIFY phaseStatusChanged)

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
    bool phaseLinkOk() const { return _phaseLinkOk; }
    int phase() const { return _phase; }
    QString phaseState() const { return _phaseState; }
    QString phaseMsg() const { return _phaseMsg; }
    double phaseProgress() const { return _phaseProgress; }
    QVariantList phaseDone() const { return _phaseDone; }

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
    /// Ask the orchestrator to run mission phase `n` (publishes command/run_phase).
    Q_INVOKABLE void runPhase(int n);
    /// Take control back from the orchestrator: publishes command/abort so it
    /// terminates the running phase script and hands the vehicle to the GCS
    /// (the orchestrator switches PX4 to HOLD / hover-in-place).
    Q_INVOKABLE void abortMission();
    /// Re-create the command/status subscription to force fresh discovery of the
    /// orchestrator (used by the panel's "retry" button after a link timeout).
    Q_INVOKABLE void retryPhaseLink();

signals:
    void rosOkChanged();
    void imageTopicsChanged();
    void imageTopicChanged();
    void imageFpsChanged();
    void actuatorTopicChanged();
    /// Emitted when actuator channels update or go stale (see haveActuator).
    void servoChannelsChanged();
    /// Emitted when a command/status message arrives or the link goes stale.
    void phaseStatusChanged();
    /// Emitted (on the GUI thread) each time a frame is decoded.
    void frameReceived(const QImage &image);

private:
    void _spinOnce();
    void _updateFps();
    void _onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void _onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg);
    void _onActuator(const mavros_msgs::msg::RCOut::ConstSharedPtr &msg);
    void _onPhaseStatus(const std_msgs::msg::String::ConstSharedPtr &msg);

    bool _rosOk = false;
    bool _ownsContext = false;                  ///< true if we called rclcpp::init

    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _imageSub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _compressedSub;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr _actuatorSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _phaseStatusSub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _runPhasePub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _abortPub;

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

    // Mission phase orchestrator status (parsed from command/status JSON).
    bool _phaseLinkOk = false;          ///< orchestrator seen recently
    int _phase = -1;
    QString _phaseState = QStringLiteral("idle");
    QString _phaseMsg;
    double _phaseProgress = -1.0;
    QVariantList _phaseDone;            ///< completed phase ids
    qint64 _lastPhaseMs = 0;            ///< ms of last command/status msg
};
