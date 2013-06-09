#ifndef SELECTOR_PROXY_H
#define SELECTOR_PROXY_H

/// COMPONENT
#include "box.h"

/// PROJECT
#include <evaluator/constructor.hpp>

/// SYSTEM
#include <QGraphicsView>
#include <iostream>
#include <QMouseEvent>
#include <typeinfo>
#include <boost/function.hpp>

namespace vision_evaluator
{

class SelectorProxy : public QGraphicsView
{
public:
    struct ProxyConstructor : public Constructor {
        boost::function<SelectorProxy* (const std::string)> constructor;

        SelectorProxy* operator()() {
            SelectorProxy* res(constructor(name));
            assert(res != NULL);
            return res;
        }
    };

    static void registerProxy(ProxyConstructor c);

public:
    SelectorProxy(const std::string& name, QWidget* parent = 0);
    virtual ~SelectorProxy();

    virtual void mousePressEvent(QMouseEvent* event) = 0;
    virtual void spawnObject(QWidget* parent, const QPoint& pos) = 0;

protected:
    std::string name_;
};

template <class T>
class SelectorProxyImp : public SelectorProxy
{
public:
    SelectorProxyImp(const std::string& name, QWidget *parent = 0)
        : SelectorProxy(name, parent), box_(new vision_evaluator::Box)/*, element_(new T)*/
    {
        setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        QSize size(80, 80);

        setScene(new QGraphicsScene(QRectF(QPoint(), size)));
        scene()->addPixmap(box_->makePixmap(name_));
    }

    virtual ~SelectorProxyImp() {
        delete box_;
//        delete element_;
    }

    virtual void spawnObject(QWidget* parent, const QPoint& pos) {
        vision_evaluator::Box* object(new vision_evaluator::Box(parent));
        object->setGeometry(pos.x(), pos.y(), 100, 100);
        object->setObjectName(name_.c_str());
        object->show();
    }

    virtual void mousePressEvent(QMouseEvent* event) {
        if(event->button() == Qt::LeftButton) {
            QDrag* drag = new QDrag(this);
            QMimeData* mimeData = new QMimeData;
            mimeData->setText(Box::MIME);
            mimeData->setParent(this);
            drag->setMimeData(mimeData);
            drag->setPixmap(box_->makePixmap(name_));
            drag->exec();
        }
    }


private:
    vision_evaluator::Box* box_;
//    T* element_;
};

}
#endif // SELECTOR_PROXY_H
