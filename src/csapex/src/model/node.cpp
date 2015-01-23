/// HEADER
#include <csapex/model/node.h>

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/assert.h>
#include <csapex/core/settings.h>

/// SYSTEM


using namespace csapex;

Node::Node()
    : adebug(std::cout, ""), ainfo(std::cout, ""), awarn(std::cout, ""), aerr(std::cerr, ""),
      modifier_(nullptr), worker_(nullptr)
{
}

Node::~Node()
{
    delete modifier_;
}

void Node::initialize(const std::string& /*type*/, const UUID& uuid,
                   NodeWorker* node_worker)
{
    worker_ = node_worker;
    modifier_ = new NodeModifier(node_worker);

    parameter_state_->setParentUUID(uuid);

    std::string p = uuid.getFullName();
    adebug.setPrefix(p);
    ainfo.setPrefix(p);
    awarn.setPrefix(p);
    aerr.setPrefix(p);
}

void Node::doSetup()
{
    setupParameters();

    try {
        setup();
    } catch(std::runtime_error& e) {
        aerr << "setup failed: " << e.what() << std::endl;
    }
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



void Node::triggerModelChanged()
{
    Q_EMIT worker_->nodeModelChanged();
}

bool Node::canTick()
{
    return true;
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
    aerr << msg << std::endl;

    if(error && level == EL_ERROR) {
        worker_->setIOError(true);
    } else {
        worker_->setIOError(false);
    }
}
