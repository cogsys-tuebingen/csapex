#ifndef SUB_GRAPH_H
#define SUB_GRAPH_H

/// COMPONENT
#include <csapex/graph.h>
#include <csapex/boxed_object.h>

namespace csapex
{

class SubGraph : public BoxedObject
{
    Q_OBJECT

private Q_SLOTS:
    void messageArrived(ConnectorIn* source);
};

}

#endif // SUB_GRAPH_H
