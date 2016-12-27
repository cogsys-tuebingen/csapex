/// HEADER
#include <csapex/model/tickable_node.h>

/// PROJECT
#include <csapex/profiling/timer.h>
#include <csapex/profiling/interlude.hpp>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>

using namespace csapex;

TickableNode::TickableNode()
{
}

TickableNode::~TickableNode()
{
}

void TickableNode::setup(NodeModifier& modifier)
{
    startTickThread();
}

void TickableNode::getProperties(std::vector<std::string>& properties) const
{
    properties.push_back("ticking");
}

bool TickableNode::doTick()
{
    apex_assert_hard(node_handle_->getNodeWorker()->canSend());

    bool res = false;
    if(canTick()){
        INTERLUDE("tick");
        try {
            tick();
            yield();
            res = true;
        } catch(const std::exception& e) {
            aerr << "tick failed: " << e.what() << std::endl;
        }
    }

    return res;
}

bool TickableNode::canTick() const
{
    return true;
}

void TickableNode::tickEvent()
{
    node_handle_->execution_requested([this](){
        if(node_handle_->getNodeWorker()->canSend()) {
            doTick();
        }
    });
}
