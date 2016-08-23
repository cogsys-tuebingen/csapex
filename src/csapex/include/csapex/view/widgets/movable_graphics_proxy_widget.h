#ifndef MOVABLE_GRAPHICS_PROXY_WIDGET_H
#define MOVABLE_GRAPHICS_PROXY_WIDGET_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QGraphicsProxyWidget>

namespace csapex
{

class CSAPEX_QT_EXPORT MovableGraphicsProxyWidget : public QGraphicsProxyWidget
{
    Q_OBJECT

public:
    MovableGraphicsProxyWidget(NodeBox* box, GraphView* view, CsApexViewCore& view_core, QGraphicsItem *parent = 0, Qt::WindowFlags wFlags = 0);
    ~MovableGraphicsProxyWidget();

    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);

    void dragEnterEvent(QGraphicsSceneDragDropEvent *event);
    void dragMoveEvent(QGraphicsSceneDragDropEvent *event);
    void dropEvent(QGraphicsSceneDragDropEvent *event);
    void dragLeaveEvent(QGraphicsSceneDragDropEvent *event);

    void contextMenuEvent(QGraphicsSceneContextMenuEvent* event);

    NodeBox* getBox();

Q_SIGNALS:
    void moving(double dx, double dy);
    void moved(double dx, double dy);

private:
    void signalMoved(const QPointF& delta);
    void signalMoving(const QPointF& delta);

private:
    QPointF before_;
    QPointF last_signaled_;
    NodeBox* box_;
    GraphView* view_;
    CsApexViewCore& view_core_;

    bool relay_;
    QPointF clone_start_;
    bool clone_started_;
    bool clone_allowed_;

    static long next_box_z;
    static long next_note_z;
};

}

#endif // MOVABLE_GRAPHICS_PROXY_WIDGET_H
