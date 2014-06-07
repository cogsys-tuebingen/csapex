#ifndef FULCRUM_WIDGET_H
#define FULCRUM_WIDGET_H

/// PROJECT
#include <csapex/model/connection.h>

/// SYSTEM
#include <QGraphicsItem>

namespace csapex
{

class FulcrumWidget : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT

public:
    FulcrumWidget(Fulcrum* fulcrum, QGraphicsItem *parent = 0);

    bool sceneEvent(QEvent *event);

public Q_SLOTS:
    void moved();

private:
    Fulcrum* fulcrum_;

    QPointF half_size_;
};

}

#endif // FULCRUM_WIDGET_H
