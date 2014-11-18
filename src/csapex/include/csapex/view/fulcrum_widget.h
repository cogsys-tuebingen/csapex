#ifndef FULCRUM_WIDGET_H
#define FULCRUM_WIDGET_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QGraphicsItem>

namespace csapex
{

class FulcrumHandle;

class FulcrumWidget : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT

public:
    FulcrumWidget(Fulcrum* fulcrum, CommandDispatcher *dispatcher, QGraphicsItem *parent = 0);

    void contextMenuEvent(QGraphicsSceneContextMenuEvent* e);
    bool sceneEvent(QEvent *event);

public Q_SLOTS:
    void moved();

    void updateHandles(Fulcrum* f, bool dropped, int which);
    void updateHandleIn(bool dropped);
    void updateHandleOut(bool dropped);


private:
    void updateHandlesHelper(FulcrumHandle& a, QGraphicsLineItem *linea, FulcrumHandle& b, QGraphicsLineItem *lineb, bool dropped);

private:
    Fulcrum* fulcrum_;
    CommandDispatcher* cmd_dispatcher_;

    QPointF half_size_;

    FulcrumHandle *handle_in_, *handle_out_;
    QGraphicsLineItem *line_in, *line_out;
};

}

#endif // FULCRUM_WIDGET_H
