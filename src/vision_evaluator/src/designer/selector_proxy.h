#ifndef SELECTOR_PROXY_H
#define SELECTOR_PROXY_H

/// COMPONENT
#include "boxed_object.h"

/// PROJECT
#include <utils/constructor.hpp>

/// SYSTEM
#include <QGraphicsView>
#include <QGroupBox>
#include <QLayout>
#include <iostream>
#include <QMouseEvent>
#include <typeinfo>
#include <boost/function.hpp>

namespace vision_evaluator
{

class Box;

class SelectorProxy : public QGraphicsView
{
public:
    struct ProxyConstructor : public Constructor {
        typedef boost::function<SelectorProxy* (const std::string)> Call;

        SelectorProxy* operator()() {
            SelectorProxy* res(constructor(name));
            assert(res != NULL);
            return res;
        }

        void setConstructor(Call c) {
            constructor = c;
            has_constructor = true;
        }

    private:
        Call constructor;
    };

    static void registerProxy(ProxyConstructor c);

public:
    SelectorProxy(const std::string& name, BoxedObject* content, QWidget* parent = 0);
    virtual ~SelectorProxy();

    virtual void mousePressEvent(QMouseEvent* event);
    virtual void spawnObject(QWidget* parent, const QPoint& pos);

protected:
    virtual BoxedObject* makeContent() = 0;

protected:
    std::string name_;
    boost::shared_ptr<vision_evaluator::Box> box_;
};

template <class T>
class SelectorProxyImp : public SelectorProxy
{
public:
    SelectorProxyImp(const std::string& name, QWidget* parent = 0)
        : SelectorProxy(name, new T, parent)
    {}

    virtual ~SelectorProxyImp()
    {}

    virtual BoxedObject* makeContent() {
        return new T;
    }
};

}
#endif // SELECTOR_PROXY_H
