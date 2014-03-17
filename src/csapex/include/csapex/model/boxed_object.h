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

#warning "BoxedObject should not be used any more!"

class BoxedObject : public Node, public NodeAdapter
{
    Q_OBJECT

public:
    typedef boost::shared_ptr<BoxedObject> Ptr;

    virtual void setup();

protected:
    virtual void setupUi(QBoxLayout* layout);

protected:
    BoxedObject(const UUID &uuid = UUID::NONE);
    virtual void fill(QBoxLayout* layout);

public:
    virtual ~BoxedObject();
};
}

#endif // BOXED_OBJECT_H
