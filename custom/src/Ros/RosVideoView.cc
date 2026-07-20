#include "RosVideoView.h"
#include "RosBridge.h"

#include <QtGui/QPainter>

RosVideoView::RosVideoView(QQuickItem *parent)
    : QQuickPaintedItem(parent)
{
    setFillColor(Qt::black);
    // RosBridge lives on the GUI thread and emits frameReceived there, so a
    // direct connection is safe.
    connect(RosBridge::instance(), &RosBridge::frameReceived, this, &RosVideoView::_onFrame);
}

void RosVideoView::_onFrame(const QImage &image)
{
    _frame = image;
    update();
}

void RosVideoView::paint(QPainter *painter)
{
    if (_frame.isNull()) {
        return;
    }

    // Letterbox: preserve aspect ratio, centered in the item.
    const QSizeF scaled = QSizeF(_frame.size()).scaled(boundingRect().size(), Qt::KeepAspectRatio);
    QRectF target(QPointF(0, 0), scaled);
    target.moveCenter(boundingRect().center());

    painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter->drawImage(target, _frame);
}
