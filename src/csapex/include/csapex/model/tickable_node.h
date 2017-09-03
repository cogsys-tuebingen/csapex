#ifndef TICKABLE_NODE_H
#define TICKABLE_NODE_H

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/utility/rate.h>
#include <csapex/utility/ticker.h>

#ifndef IGNORE_TICKABLE_NODE_WARNING
#pragma message "TickableNode is deprecated and may not work correctly anymore!"
#pragma message "  - if you used it to implement a source, just derive from Node directly and implement process()"
#pragma message "  - if you used it to implement a frequency limit source, just derive from ThrottledNode directly and implement process()"
#pragma message "  - if you used it to get a tick() callback periodically, use multiple inheritance and derive from Node and Ticker"
#endif

namespace csapex
{

class CSAPEX_EXPORT TickableNode : public Node, public Ticker
{
public:
    virtual void setup(NodeModifier& modifier);

    virtual void getProperties(std::vector<std::string>& properties) const override;

protected:
    // API:
    virtual bool canTick() const;
    virtual void tick() = 0;

private:
    bool doTick();
    virtual void tickEvent() final override;

protected:
    TickableNode();
    ~TickableNode();
};

}

#endif // TICKABLE_NODE_H
