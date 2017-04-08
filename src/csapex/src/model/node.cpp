/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/assert.h>
#include <csapex/core/settings.h>
#include <csapex/model/generic_state.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/input_transition.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Node::Node()
    : adebug(std::cout, ""), ainfo(std::cout, ""), awarn(std::cout, ""), aerr(std::cerr, ""),
      node_handle_(nullptr),
      guard_(-1)
{
}

Node::~Node()
{
    apex_assert_hard(guard_ == -1);

    guard_ = 0xDEADBEEF;
}

NodeHandle* Node::getNodeHandle() const
{
    return node_handle_.get();
}

void Node::initialize(NodeHandlePtr node_handle)
{
    node_modifier_ = node_handle.get();
    node_handle_ = node_handle;
    parameters_ = this;

    UUID uuid = node_handle->getUUID();

    parameter_state_->setParentUUID(uuid);

    std::string p = uuid.getFullName();
    adebug.setPrefix(p);
    ainfo.setPrefix(p);
    awarn.setPrefix(p);
    aerr.setPrefix(p);
}

void Node::detach()
{
    stopObserving();
    if(node_handle_) {
        apex_assert_hard(node_handle_->guard_ == -1);
    }
    node_handle_.reset();
}

UUID Node::getUUID() const
{
    return node_handle_ ? node_handle_->getUUID() : UUID::NONE;
}

void Node::setupParameters(Parameterizable& )
{

}

void Node::stateChanged()
{

}

bool Node::isAsynchronous() const
{
    return false;
}
bool Node::isIsolated() const
{
    return false;
}

bool Node::canProcess() const
{
    if(!node_handle_->getOutputTransition()->hasConnection() && !node_handle_->getInputTransition()->hasConnection()) {
        // by default, nodes without any synchronous ports should not be processed.
        return false;
    } else {
        return true;
    }
}

void Node::yield() const
{
    apex_assert(node_handle_);
    node_handle_->might_be_enabled();
}

void Node::process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters, Continuation continuation)
{
    process(node_modifier, parameters);
    continuation([](csapex::NodeModifier&, csapex::Parameterizable&){});
}

void Node::process(csapex::NodeModifier& /*node_modifier*/, csapex::Parameterizable& /*parameters*/)
{
    process();
}


void Node::process()
{
    // default: do nothing, clients overwrite this
}

void Node::finishSetup()
{

}

void Node::tearDown()
{

}

void Node::reset()
{
}

bool Node::processMessageMarkers() const
{
    return false;
}

void Node::processMarker(const csapex::connection_types::MessageConstPtr &marker)
{

}

void Node::activation()
{

}

void Node::deactivation()
{

}

void Node::getProperties(std::vector<std::string> &/*properties*/) const
{
}
