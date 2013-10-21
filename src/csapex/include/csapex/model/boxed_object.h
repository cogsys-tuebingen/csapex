#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/model/node.h>
#include <csapex/view/displayable.h>
#include <csapex/view/node_adapter.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>
#include <QObject>
#include <QMutex>

namespace csapex
{

class BoxedObject : public Node, public Displayable, public NodeAdapter
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<BoxedObject> Ptr;
    static const Ptr NullPtr;

protected:
    BoxedObject();

public:
    virtual ~BoxedObject();

    virtual void updateModel();


public Q_SLOTS:
    virtual void tick();
    virtual void disable();

Q_SIGNALS:
    void modelChanged();
    void guiChanged();

protected:
    void errorEvent(bool error, ErrorLevel level);

protected:
};



class NullBoxedObject : public BoxedObject {
    Q_OBJECT

public:
    NullBoxedObject(const std::string& type)
    {
        type_ = type;
    }
};

}

#endif // BOXED_OBJECT_H
