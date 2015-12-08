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

    bool isTickEnabled() const;
    void setTickEnabled(bool tick);

    double getTickFrequency() const;
    void setTickFrequency(double f);

    bool isImmediate() const;

protected:
    TickableNode();

private:
    bool tick_enabled_;
    double tick_frequency_;
    bool tick_immediate_;
};

}

#endif // TICKABLE_NODE_H
