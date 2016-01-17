/// HEADER
#include <csapex/model/tickable_node.h>

using namespace csapex;

TickableNode::TickableNode()
    : tick_enabled_(true),
      tick_frequency_(30.0),
      tick_immediate_(false)
{

}

bool TickableNode::canTick()
{
    return true;
}

bool TickableNode::isTickEnabled() const
{
    return tick_enabled_;
}

void TickableNode::setTickEnabled(bool tick)
{
    tick_enabled_ = tick;
}

double TickableNode::getTickFrequency() const
{
    return tick_frequency_;
}
void TickableNode::setTickFrequency(double f)
{
    tick_frequency_ = f;
}

bool TickableNode::isImmediate() const
{
    return tick_immediate_;
}

void TickableNode::tick(NodeModifier &node_modifier, Parameterizable &)
{
    tick();
}

void TickableNode::tick()
{
}
