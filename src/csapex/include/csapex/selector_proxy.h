#ifndef SELECTOR_PROXY_H
#define SELECTOR_PROXY_H

/// COMPONENT
#include <utils_plugin/constructor.hpp>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QGraphicsView>
#include <QGroupBox>
#include <QLayout>
#include <iostream>
#include <QMouseEvent>
#include <typeinfo>
#include <boost/function.hpp>
#include <QIcon>

namespace csapex
{

class SelectorProxy/* : public QGraphicsView*/
{
    friend class command::AddBox;
    friend class BoxManager;

public:
    typedef boost::shared_ptr<SelectorProxy> Ptr;
    static const Ptr NullPtr;

public:
    struct ProxyConstructor : public Constructor {
        typedef boost::function<SelectorProxy::Ptr (const std::string)> Call;

        SelectorProxy::Ptr operator()() {
            SelectorProxy::Ptr res(constructor(type));
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

    static void registerProxy(SelectorProxy::Ptr prototype);

public:
    SelectorProxy(const std::string& type, const std::string &description, boost::shared_ptr<BoxedObject> prototype);
    virtual ~SelectorProxy();

    virtual SelectorProxy* clone() = 0;

    virtual void mousePressEvent(QMouseEvent* event);
    std::string getType();
    std::vector<Tag> getTags();
    QIcon getIcon();
    std::string getDescription();

    void startObjectPositioning(QWidget *parent, const QPoint &offset = QPoint(0,0), const std::string& template_ = "");

private:
    /// PRIVATE: Use command to spawn objects (undoable)
    virtual BoxPtr create(const QPoint& pos, const std::string &type, const std::string& uuid);

protected:
    virtual BoxedObjectPtr makeContent() = 0;

protected:
    std::string type_;
    std::string descr_;
    QIcon icon;
    std::vector<Tag> cat;
};

template <class T>
class SelectorProxyImp : public SelectorProxy
{
public:
    SelectorProxyImp(const std::string& type, const std::string& description)
        : SelectorProxy(type, description, boost::shared_ptr<BoxedObject>(new T))
    {}

    virtual ~SelectorProxyImp()
    {}

    virtual boost::shared_ptr<BoxedObject> makeContent() {
        return boost::shared_ptr<BoxedObject>(new T);
    }

    virtual SelectorProxy* clone() {
        return new SelectorProxyImp<T>(type_, descr_);
    }
};

class SelectorProxyDynamic : public SelectorProxy
{
public:
    typedef boost::function< boost::shared_ptr<BoxedObject>()> Make;

public:
    SelectorProxyDynamic(const std::string& name, const std::string& description, Make c)
        : SelectorProxy(name, description, c()), c(c)
    {}

    virtual ~SelectorProxyDynamic()
    {}

    virtual boost::shared_ptr<BoxedObject> makeContent() {
        return c();
    }

    virtual SelectorProxy* clone() {
        return new SelectorProxyDynamic(getType(), getDescription(), c);
    }

protected:
    Make c;
};


}
#endif // SELECTOR_PROXY_H
