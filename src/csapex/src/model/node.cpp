/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

Node::Node(const UUID &uuid)
    : Unique(uuid),
      modifier_(new NodeModifier(this)), settings_(NULL),
      ainfo(std::cout, uuid.getFullName()), awarn(std::cout, uuid.getFullName()), aerr(std::cerr, uuid.getFullName()), alog(std::clog, uuid.getFullName()),
      worker_(NULL),
      node_state_(new NodeState(this))
{
}

Node::~Node()
{
    delete worker_;
    delete modifier_;
}

void Node::initialize(const std::string& type, const UUID& uuid,
                   NodeWorker* node_worker, Settings* settings)
{
    type_ = type;
    worker_ = node_worker;
    settings_ = settings;

    setUUID(uuid);

    std::string p = uuid.getFullName();
    ainfo.setPrefix(p);
    awarn.setPrefix(p);
    aerr.setPrefix(p);
    alog.setPrefix(p);


    setupParameters();
    worker_->makeParametersConnectable();

    try {
        setup();
    } catch(std::runtime_error& e) {
        aerr << "setup failed: " << e.what() << std::endl;
    }
}

std::string Node::getType() const
{
    return type_;
}

void Node::messageArrived(Input *)
{

}
void Node::setupParameters()
{

}

void Node::stateChanged()
{

}

void Node::process()
{
}

NodeState::Ptr Node::getNodeStateCopy() const
{
    apex_assert_hard(node_state_);

    NodeState::Ptr memento(new NodeState(this));
    *memento = *node_state_;

    memento->setParameterState(getParameterState());

    return memento;
}

NodeState::Ptr Node::getNodeState()
{
    apex_assert_hard(node_state_);

    return node_state_;
}

void Node::setNodeState(NodeState::Ptr memento)
{
    boost::shared_ptr<NodeState> m = boost::dynamic_pointer_cast<NodeState> (memento);
    apex_assert_hard(m.get());

    UUID old_uuid = getUUID();
    std::string old_label = node_state_->getLabel();

    *node_state_ = *m;

    if(getUUID().empty()) {
        setUUID(old_uuid);
    }
    if(node_state_->getLabel().empty()) {
        if(old_label.empty()) {
            node_state_->setLabel(getUUID().getShortName());
        } else {
            node_state_->setLabel(old_label);
        }
    }

    node_state_->setParent(this);
    if(m->getParameterState()) {
        setParameterState(m->getParameterState());
    }

    Q_EMIT worker_->nodeStateChanged();

    stateChanged();
}

Memento::Ptr Node::getParameterState() const
{
    return parameter_state_.clone();
}

void Node::setParameterState(Memento::Ptr memento)
{
    boost::shared_ptr<GenericState> m = boost::dynamic_pointer_cast<GenericState> (memento);
    apex_assert_hard(m.get());

    parameter_state_.setFrom(*m);

    triggerModelChanged();
}


void Node::triggerModelChanged()
{
    Q_EMIT worker_->nodeModelChanged();
}

void Node::tick()
{
}

void Node::abort()
{
}

NodeWorker* Node::getNodeWorker() const
{
    return worker_;
}


void Node::errorEvent(bool error, const std::string& /*msg*/, ErrorLevel level)
{
    if(node_state_->isEnabled() && error && level == EL_ERROR) {
        worker_->setIOError(true);
    } else {
        worker_->setIOError(false);
    }
}



Input* Node::addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async)
{
    return worker_->addInput(type, label, optional, async);
}

Output* Node::addOutput(ConnectionTypePtr type, const std::string& label)
{
    return worker_->addOutput(type, label);
}

void Node::removeInput(const UUID &uuid)
{
    worker_->removeInput(getInput(uuid));
}

void Node::removeOutput(const UUID &uuid)
{
    worker_->removeOutput(getOutput(uuid));
}

Input* Node::getInput(const UUID& uuid) const
{
    BOOST_FOREACH(Input* in, worker_->inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }
    BOOST_FOREACH(Input* in, worker_->managed_inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

Output* Node::getOutput(const UUID& uuid) const
{
    BOOST_FOREACH(Output* out, worker_->outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }
    BOOST_FOREACH(Output* out, worker_->managed_outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }

    return NULL;
}

std::vector<Input*> Node::getAllInputs() const
{
    std::vector<Input*> result;
    result = worker_->inputs_;
    result.insert(result.end(), worker_->managed_inputs_.begin(), worker_->managed_inputs_.end());
    return result;
}

std::vector<Output*> Node::getAllOutputs() const
{
    std::vector<Output*> result;
    result = worker_->outputs_;
    result.insert(result.end(), worker_->managed_outputs_.begin(), worker_->managed_outputs_.end());
    return result;
}

std::vector<Input*> Node::getMessageInputs() const
{
    return worker_->inputs_;
}

std::vector<Output*> Node::getMessageOutputs() const
{
    return worker_->outputs_;
}

std::vector<Input*> Node::getManagedInputs() const
{
    return worker_->managed_inputs_;
}

std::vector<Output*> Node::getManagedOutputs() const
{
    return worker_->managed_outputs_;
}
