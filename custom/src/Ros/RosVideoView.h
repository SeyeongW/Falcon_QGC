#pragma once

#include <QtGui/QImage>
#include <QtQuick/QQuickPaintedItem>

/// QML item that paints the latest frame delivered by the `RosBridge` singleton.
///
/// It carries no topic state of its own: topic selection lives on `RosBridge`
/// (so the rqt-style dropdown drives the subscription), and this view simply
/// renders whatever `RosBridge::frameReceived` emits, letterboxed to fit.
/// Registered with QML as `RosVideoView` under `Custom.Ros`.
class RosVideoView : public QQuickPaintedItem
{
    Q_OBJECT

public:
    explicit RosVideoView(QQuickItem *parent = nullptr);

    void paint(QPainter *painter) override;

private:
    void _onFrame(const QImage &image);

    QImage _frame;
};
