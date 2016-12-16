/// HEADER
#include <csapex/model/tickable_node.h>

/// PROJECT
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/model/node_handle.h>

using namespace csapex;

TickableNode::TickableNode()
    : tick_enabled_(true),
      tick_rate_(30.0, false)
{

}

bool TickableNode::isDoneProcessing() const
{
    return true;
}

void TickableNode::getProperties(std::vector<std::string>& properties) const
{
    GeneratorNode::getProperties(properties);
    properties.push_back("ticking");
}

bool TickableNode::doTick(NodeModifier &nm, Parameterizable &p)
{
    bool res;
    {
        INTERLUDE("tick");
        try {
            res = tick(nm, p);
        } catch(const std::exception& e) {
            aerr << "tick failed: " << e.what() << std::endl;
            res = false;
        }
    }

    if(res) {
        updated();
    }

    return res;
}

bool TickableNode::canTick()
{
    return true;
}

bool TickableNode::canProcess() const
{
    if(node_handle_->isSource()) {
        // legacy: sources deriving from tickable sould not process
        // TODO: re-implement all tickables!
        return false;

    } else {
        // the node is no source -> process as usual
        return true;
    }
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
    return tick_rate_.getFrequency();
}
void TickableNode::setTickFrequency(double f)
{
    tick_rate_.setFrequency(f);
}

void TickableNode::setTickImmediate(bool immediate)
{
    tick_rate_.setImmediate(immediate);
}

bool TickableNode::isImmediate() const
{
    return tick_rate_.isImmediate();
}

bool TickableNode::tick(NodeModifier &nm, Parameterizable &p)
{
    tick();
    return true;
}

void TickableNode::tick()
{
}

void TickableNode::keepUpRate()
{
    tick_rate_.keepUp();
}
