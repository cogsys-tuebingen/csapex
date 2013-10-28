#ifndef BOXED_OBJECT_H
#define BOXED_OBJECT_H

/// COMPONENT
#include <csapex/model/memento.h>
#include <csapex/model/node.h>
#include <csapex/view/node_adapter.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <string>
#include <QObject>
#include <QMutex>

namespace csapex
{

class BoxedObject : public Node, public NodeAdapter
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<BoxedObject> Ptr;

    virtual void setup();

protected:
    virtual void setupUi(QBoxLayout* layout);

protected:
    BoxedObject(const std::string& uuid = "");
    virtual void fill(QBoxLayout* layout);

public:
    virtual ~BoxedObject();
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
