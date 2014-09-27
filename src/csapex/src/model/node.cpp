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
#include <csapex/core/settings.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

Node::Node(const UUID &uuid)
    : Unique(uuid),
      ainfo(std::cout, uuid.getFullName()), awarn(std::cout, uuid.getFullName()), aerr(std::cerr, uuid.getFullName()), alog(std::clog, uuid.getFullName()),
      modifier_(NULL), settings_(NULL),
      worker_(NULL),
      node_state_(new NodeState(this))
{
}

Node::~Node()
{
    delete modifier_;
}

void Node::initialize(const std::string& type, const UUID& uuid,
                   NodeWorker* node_worker, Settings* settings)
{
    type_ = type;
    worker_ = node_worker;
    modifier_ = new NodeModifier(node_worker),
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

    memento->setParameterState(getParameterStateClone());

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


void Node::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    if(settings_->get<bool>("headless")){
        aerr << msg << std::endl;
    }
    if(node_state_->isEnabled() && error && level == EL_ERROR) {
        worker_->setIOError(true);
    } else {
        worker_->setIOError(false);
    }
}



