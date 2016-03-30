#ifndef TICKABLE_NODE_H
#define TICKABLE_NODE_H

/// COMPONENT
#include <csapex/model/generator_node.h>

namespace csapex
{

class TickableNode : public GeneratorNode
{
public:
    bool doTick(NodeModifier &nm, Parameterizable &p);
    virtual bool canTick();

    bool isTickEnabled() const;
    void setTickEnabled(bool tick);

    double getTickFrequency() const;
    void setTickFrequency(double f);

    bool isImmediate() const;

protected:
    virtual bool tick(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters);
    virtual void tick();

protected:
    TickableNode();

private:
    bool tick_enabled_;
    double tick_frequency_;
    bool tick_immediate_;
};

}

#endif // TICKABLE_NODE_H
