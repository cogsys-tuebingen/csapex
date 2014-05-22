#ifndef MOVABLE_GRAPHICS_PROXY_WIDGET_H
#define MOVABLE_GRAPHICS_PROXY_WIDGET_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QGraphicsProxyWidget>

namespace csapex
{

class MovableGraphicsProxyWidget : public QGraphicsProxyWidget
{
    Q_OBJECT

public:
    MovableGraphicsProxyWidget(NodeBox* box, QGraphicsItem *parent = 0, Qt::WindowFlags wFlags = 0);

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

    void dragEnterEvent(QGraphicsSceneDragDropEvent *event);
    void dragMoveEvent(QGraphicsSceneDragDropEvent *event);
    void dropEvent(QGraphicsSceneDragDropEvent *event);
    void dragLeaveEvent(QGraphicsSceneDragDropEvent *event);

    void contextMenuEvent(QGraphicsSceneContextMenuEvent* event);

    NodeBox* getBox();

private:
    NodeBox* box_;
};

}

#endif // MOVABLE_GRAPHICS_PROXY_WIDGET_H
