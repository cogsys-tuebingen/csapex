/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>
#include <csapex/core/settings.h>

/// SYSTEM


using namespace csapex;

Node::Node()
    : adebug(std::cout, ""), ainfo(std::cout, ""), awarn(std::cout, ""), aerr(std::cerr, ""),
      modifier_(nullptr)
{
}

Node::~Node()
{
}

void Node::initialize(const UUID& uuid, NodeModifier *node_modifier)
{
    modifier_ = node_modifier;

    parameter_state_->setParentUUID(uuid);

    std::string p = uuid.getFullName();
    adebug.setPrefix(p);
    ainfo.setPrefix(p);
    awarn.setPrefix(p);
    aerr.setPrefix(p);
}

void Node::doSetup()
{
    setupParameters(*this);

    try {
        setup(*modifier_);
    } catch(std::runtime_error& e) {
        aerr << "setup failed: " << e.what() << std::endl;
    }
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

void Node::process(csapex::Parameterizable& parameters)
{
    // default to deprecated style
    process();
}


void Node::process(csapex::Parameterizable& parameters, std::function<void(std::function<void ()>)> continuation)
{
    process(parameters);
    continuation([](){});
}

void Node::process()
{
    // default: do nothing, clients overwrite this
}


void Node::abort()
{
}
