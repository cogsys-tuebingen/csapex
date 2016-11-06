#ifndef MINIMAP_WIDGET_H
#define MINIMAP_WIDGET_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/view/view_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

class CSAPEX_QT_EXPORT MinimapWidget : public QWidget
{
    Q_OBJECT

public:
    MinimapWidget();

Q_SIGNALS:
    void resizeRequest(QSize);
    void positionRequest(QPointF);
    void zoomRequest(QPointF at, double factor);

public Q_SLOTS:
    void doResize();
    void display(GraphView* view);

    void disconnectView();

protected:
    void mousePressEvent(QMouseEvent* me);
    void mouseReleaseEvent(QMouseEvent* me);
    void mouseMoveEvent(QMouseEvent* me);

    void wheelEvent(QWheelEvent* e);

    void paintEvent(QPaintEvent* e);

private:
    void emitPositionRequest(QMouseEvent *me);

private:
    GraphView* view_;
    DesignerScene* scene_;

    bool dragging_;
    QPoint dragging_start_;
    QPoint dragging_offset_;

    QSize size_screen_;
    QTransform scene_to_minimap_;
};

}

#endif // MINIMAP_WIDGET_H

