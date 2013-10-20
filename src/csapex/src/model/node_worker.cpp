/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/box.h>
#include <csapex/utility/timer.h>

using namespace csapex;

NodeWorker::NodeWorker(Box *parent)
    : parent_(parent)
{
    node_ = parent_->getContent();
    assert(node_);
}

void NodeWorker::forwardMessage(Connector *s)
{
    ConnectorIn* source = dynamic_cast<ConnectorIn*> (s);
    assert(source);

    if(node_->isEnabled()) {
        if(parent_->synchronized_inputs_) {
            forwardMessageSynchronized(source);
        } else {
            forwardMessageDirectly(source);
        }
    }


    if(!node_->isError() || node_->errorLevel() != Displayable::EL_ERROR) {
        node_->setError(false);
    }
}

void NodeWorker::forwardMessageDirectly(ConnectorIn *source)
{
    Timer t;
    node_->messageArrived(source);
    parent_->timer_history_.push_back(t.elapsedMs());

    std::cout << "warning: using deprecated message forwarding in " << parent_->getType() << std::endl;
    parent_->messageProcessed();
}

void NodeWorker::forwardMessageSynchronized(ConnectorIn *source)
{
    parent_->has_msg[source] = true;

    typedef std::pair<ConnectorIn*, bool> PAIR;
    foreach(const PAIR& pair, parent_->has_msg) {
        ConnectorIn* c = pair.first;
        if(!pair.second) {
            // connector doesn't have a message
            if(c->isOptional()) {
                if(c->isConnected()) {
                    // c is optional and connected, so we have to wait for a message
                    return;
                } else {
                    // c is optinal and not connected, so we can proceed
                    /* do nothing */
                }
            } else {
                // c is mandatory, so we have to wait for a message
                return;
            }
        }
    }

    Timer t;
    node_->allConnectorsArrived();
    parent_->timer_history_.push_back(t.elapsedMs());

    parent_->messageProcessed();


    foreach(const PAIR& pair, parent_->has_msg) {
        parent_->has_msg[pair.first] = false;
    }
}

void NodeWorker::tick()
{
    if(node_->isEnabled()) {
        node_->tick();
    }
}
void NodeWorker::eventGuiChanged()
{
    if(node_->isEnabled()) {
        node_->updateModel();
    }
}

void NodeWorker::triggerError(bool e, const std::string &what)
{
    node_->setError(e, what);
}

