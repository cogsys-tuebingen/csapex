#ifndef FULCRUM_WIDGET_H
#define FULCRUM_WIDGET_H

/// COMPONENT
#include <csapex/view/csapex_qt_export.h>

/// PROJECT
#include <csapex/model/model_fwd.h>

/// SYSTEM
#include <QGraphicsItem>

namespace csapex
{

class FulcrumHandle;

class CSAPEX_QT_EXPORT FulcrumWidget : public QObject, public QGraphicsEllipseItem
{
    Q_OBJECT

public:
    FulcrumWidget(Fulcrum* fulcrum, QGraphicsItem *parent = 0);

    void contextMenuEvent(QGraphicsSceneContextMenuEvent* e);
    bool sceneEvent(QEvent *event);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);

Q_SIGNALS:
    void movedEvent();
    void movedHandlesEvent(Fulcrum* f, bool dropped, int which);

    void deleteRequest(Fulcrum* f);
    void modifyRequest(Fulcrum* f, int type);

public Q_SLOTS:
    void moved();

    void updateHandles(Fulcrum* f, bool dropped, int which);
    void updateHandleIn(bool dropped);
    void updateHandleOut(bool dropped);


private:
    void updateHandlesHelper(FulcrumHandle& a, QGraphicsLineItem *linea, FulcrumHandle& b, QGraphicsLineItem *lineb, bool dropped);

private:
    Fulcrum* fulcrum_;

    QPointF half_size_;

    FulcrumHandle *handle_in_, *handle_out_;
    QGraphicsLineItem *line_in, *line_out;
};

}

#endif // FULCRUM_WIDGET_H
