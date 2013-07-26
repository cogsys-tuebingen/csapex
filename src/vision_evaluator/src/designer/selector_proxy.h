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

namespace command
{
class AddBox;
}

class SelectorProxy : public QGraphicsView
{
    friend class command::AddBox;
    friend class BoxManager;

public:
    typedef boost::shared_ptr<SelectorProxy> Ptr;

public:
    struct ProxyConstructor : public Constructor {
        typedef boost::function<SelectorProxy* (const std::string)> Call;

        SelectorProxy* operator()() {
            SelectorProxy* res(constructor(type));
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
    SelectorProxy(const std::string& type, BoxedObject* prototype, QWidget* parent = 0);
    virtual ~SelectorProxy();

    virtual SelectorProxy* clone() = 0;

    virtual void mousePressEvent(QMouseEvent* event);
    std::string getType();
    std::string getCategory();

    void startObjectPositioning(const QPoint &offset = QPoint(0,0));

private:
    /// PRIVATE: Use command to spawn objects (undoable)
    virtual vision_evaluator::Box* spawnObject(QWidget* parent, const QPoint& pos, const std::string &type, const std::string& uuid);

protected:
    virtual BoxedObject* makeContent() = 0;

protected:
    std::string type_;
    boost::shared_ptr<vision_evaluator::Box> prototype_box_;
};

template <class T>
class SelectorProxyImp : public SelectorProxy
{
public:
    SelectorProxyImp(const std::string& type, QWidget* parent = 0)
        : SelectorProxy(type, new T, parent)
    {}

    virtual ~SelectorProxyImp()
    {}

    virtual BoxedObject* makeContent() {
        return new T;
    }

    virtual SelectorProxy* clone() {
        return new SelectorProxyImp<T>(type_, parentWidget());
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
        return new SelectorProxyDynamic(getType(), c);
    }

protected:
    Make c;
};


}
#endif // SELECTOR_PROXY_H
