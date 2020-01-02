#ifndef MOVABLE_GRAPHICS_PROXY_WIDGET_H
#define MOVABLE_GRAPHICS_PROXY_WIDGET_H

/// COMPONENT
#include <csapex_qt/export.h>
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QGraphicsProxyWidget>

namespace csapex
{
class CSAPEX_QT_EXPORT MovableGraphicsProxyWidget : public QGraphicsProxyWidget
{
    Q_OBJECT

public:
    MovableGraphicsProxyWidget(NodeBox* box, GraphView* view, CsApexViewCore& view_core, QGraphicsItem* parent = 0, Qt::WindowFlags wFlags = 0);
    ~MovableGraphicsProxyWidget() override;

    QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;

    void dragEnterEvent(QGraphicsSceneDragDropEvent* event) override;
    void dragMoveEvent(QGraphicsSceneDragDropEvent* event) override;
    void dropEvent(QGraphicsSceneDragDropEvent* event) override;
    void dragLeaveEvent(QGraphicsSceneDragDropEvent* event) override;

    void contextMenuEvent(QGraphicsSceneContextMenuEvent* event) override;

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

}  // namespace csapex

#endif  // MOVABLE_GRAPHICS_PROXY_WIDGET_H
