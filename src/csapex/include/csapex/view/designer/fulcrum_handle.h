#ifndef FULCRUM_HANDLE_H
#define FULCRUM_HANDLE_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// SYSTEM
#include <QGraphicsItem>

namespace csapex
{

class CSAPEX_QT_EXPORT FulcrumHandle: public QObject, public QGraphicsEllipseItem
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
