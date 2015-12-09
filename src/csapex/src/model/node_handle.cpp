/// HEADER
#include <csapex/model/node_handle.h>

/// COMPONENT
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>

using namespace csapex;

NodeHandle::NodeHandle(const std::string &type, const UUID& uuid, NodePtr node)
    : Unique(uuid),
      node_(node),
      node_type_(type),
      transition_in_(std::make_shared<InputTransition>(std::bind(&NodeHandle::triggerCheckTransitions, this))),
      transition_out_(std::make_shared<OutputTransition>(std::bind(&NodeHandle::triggerCheckTransitions, this)))
{

}

NodeHandle::~NodeHandle()
{

}

std::string NodeHandle::getType() const
{
    return node_type_;
}

NodeWeakPtr NodeHandle::getNode() const
{
    return node_;
}

InputTransition* NodeHandle::getInputTransition() const
{
    return transition_in_.get();
}

OutputTransition* NodeHandle::getOutputTransition() const
{
    return transition_out_.get();
}
