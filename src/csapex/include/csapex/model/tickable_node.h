#ifndef TICKABLE_NODE_H
#define TICKABLE_NODE_H

/// COMPONENT
#include <csapex/model/generator_node.h>
#include <csapex/utility/rate.h>

namespace csapex
{

class CSAPEX_EXPORT TickableNode : public GeneratorNode
{
public:
    bool doTick(NodeModifier &nm, Parameterizable &p);
    virtual bool canTick();

    bool isTickEnabled() const;
    void setTickEnabled(bool tick);

    void setTickFrequency(double f);
    double getTickFrequency() const;

    void setTickImmediate(bool immediate);
    bool isImmediate() const;

    void keepUpRate();

    virtual bool isDoneProcessing() const override;

    virtual void getProperties(std::vector<std::string>& properties) const override;

protected:
    virtual bool tick(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters);
    virtual void tick();

protected:
    TickableNode();

private:
    bool tick_enabled_;

    Rate tick_rate_;
};

}

#endif // TICKABLE_NODE_H
