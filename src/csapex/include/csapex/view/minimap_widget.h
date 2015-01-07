#ifndef MINIMAP_WIDGET_H
#define MINIMAP_WIDGET_H

/// PROJECT
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

class MinimapWidget : public QWidget
{
    Q_OBJECT

public:
    MinimapWidget(DesignerView* view, DesignerScene* scene);

Q_SIGNALS:
    void resizeRequest(QSize);
    void positionRequest(QPointF);
    void zoomRequest(QPointF at, double factor);

public Q_SLOTS:
    void doResize();

protected:
    void mousePressEvent(QMouseEvent* me);
    void mouseReleaseEvent(QMouseEvent* me);
    void mouseMoveEvent(QMouseEvent* me);

    void wheelEvent(QWheelEvent* e);

    void paintEvent(QPaintEvent* e);

private:
    void emitPositionRequest(QMouseEvent *me);

private:
    DesignerView* view_;
    DesignerScene* scene_;

    bool dragging_;
    QPoint dragging_start_;
    QPoint dragging_offset_;

    QSize size_screen_;
    QTransform scene_to_minimap_;
};

}

#endif // MINIMAP_WIDGET_H

