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

    virtual void getProperties(std::vector<std::string>& properties) const override;

public:
    csapex::slim_signal::Signal<void()> updated;

protected:
    GeneratorNode();
};

}

#endif // GENERATOR_NODE_H
