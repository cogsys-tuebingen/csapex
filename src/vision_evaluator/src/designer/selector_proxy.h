#ifndef SELECTOR_PROXY_H
#define SELECTOR_PROXY_H

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
class BoxedObject;

namespace command {
class AddBox;
}

class SelectorProxy : public QGraphicsView
{
    friend class command::AddBox;

public:
    typedef boost::shared_ptr<SelectorProxy> Ptr;

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
    static void registerProxy(SelectorProxy::Ptr prototype);

public:
    SelectorProxy(const std::string& name, BoxedObject* prototype, QWidget* parent = 0);
    virtual ~SelectorProxy();

    virtual SelectorProxy* clone() = 0;

    virtual void mousePressEvent(QMouseEvent* event);
    std::string name();

private:
    virtual vision_evaluator::Box* spawnObject(QWidget* parent, const QPoint& pos, const std::string &uuid);

protected:
    virtual BoxedObject* makeContent() = 0;

protected:
    std::string name_;
    boost::shared_ptr<vision_evaluator::Box> prototype_box_;
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

    virtual SelectorProxy* clone() {
        assert(false);
        return NULL;
    }
};

class SelectorProxyDynamic : public SelectorProxy
{
public:
    typedef boost::function<BoxedObject*()> Make;

public:
    SelectorProxyDynamic(const std::string& name, Make c, QWidget* parent = 0)
        : SelectorProxy(name, c(), parent), c(c)
    {}

    virtual ~SelectorProxyDynamic()
    {}

    virtual BoxedObject* makeContent() {
        return c();
    }

    virtual SelectorProxy* clone() {
        return new SelectorProxyDynamic(name(), c);
    }

protected:
    Make c;
};


}
#endif // SELECTOR_PROXY_H
