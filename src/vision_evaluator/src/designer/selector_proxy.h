#ifndef SELECTOR_PROXY_H
#define SELECTOR_PROXY_H

/// COMPONENT
#include "box.h"

/// SYSTEM
#include <QGraphicsView>
#include <iostream>
#include <QMouseEvent>
#include <typeinfo>

namespace vision_evaluator
{


class SelectorProxy : public QGraphicsView
{
public:
    SelectorProxy(QWidget* parent = 0);
    virtual ~SelectorProxy();

    virtual void mousePressEvent(QMouseEvent* event) = 0;
    virtual void spawnObject(QWidget* parent, const QPoint& pos) = 0;
};

template <class T>
class SelectorProxyImp : public SelectorProxy
{
public:
    SelectorProxyImp(QWidget* parent = 0)
        : SelectorProxy(parent), box_(new T) {
        setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        QSize size(80, 80);

        setScene(new QGraphicsScene(QRectF(QPoint(), size)));
        scene()->addPixmap(box_->makePixmap());
    }

    virtual ~SelectorProxyImp() {
        delete box_;
    }

    virtual void spawnObject(QWidget* parent, const QPoint& pos) {
        T* object(new T(parent));
        object->setGeometry(pos.x(), pos.y(), 100, 100);
        object->show();
    }

    virtual void mousePressEvent(QMouseEvent* event) {
        if(event->button() == Qt::LeftButton) {
            QDrag* drag = new QDrag(this);
            QMimeData* mimeData = new QMimeData;
            mimeData->setText(Box::MIME);
            mimeData->setParent(this);
            drag->setMimeData(mimeData);
            drag->setPixmap(box_->makePixmap());
            drag->exec();
        }
    }


private:
    T* box_;
};

}
#endif // SELECTOR_PROXY_H
