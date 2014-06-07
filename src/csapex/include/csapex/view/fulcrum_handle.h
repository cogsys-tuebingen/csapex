#ifndef FULCRUM_HANDLE_H
#define FULCRUM_HANDLE_H

/// SYSTEM
#include <QGraphicsItem>

namespace csapex
{

class FulcrumHandle: public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT

public:
    FulcrumHandle(const QPointF &pos, QGraphicsItem *parent = 0);

    bool sceneEvent(QEvent *event);

Q_SIGNALS:
    void moved(bool dropped);
};

}

#endif // FULCRUM_HANDLE_H
