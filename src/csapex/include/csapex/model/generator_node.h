#ifndef GENERATOR_NODE_H
#define GENERATOR_NODE_H

/// COMPONENT
#include <csapex/model/node.h>

namespace csapex
{

class CSAPEX_EXPORT GeneratorNode : public Node
{
public:
    virtual void notifyMessagesProcessed();

    virtual void getProperties(std::vector<std::string>& properties) const override;

    virtual bool canProcess() const;
    virtual bool isDoneProcessing() const = 0;

public:
    slim_signal::Signal<void()> updated;
    slim_signal::Signal<void()> finished;

protected:
    GeneratorNode();
};

}

#endif // GENERATOR_NODE_H
