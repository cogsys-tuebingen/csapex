#ifndef GENERATOR_NODE_H
#define GENERATOR_NODE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{

class GeneratorNode : public Node
{
public:
    virtual void notifyMessagesProcessed();

public:
    csapex::slim_signal::Signal<void()> updated;

protected:
    GeneratorNode();
};

}

#endif // GENERATOR_NODE_H
