#ifndef TICKABLE_NODE_H
#define TICKABLE_NODE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{

class TickableNode : public Node
{
public:
    virtual bool canTick();
    virtual void tick() = 0;
};

}

#endif // TICKABLE_NODE_H
