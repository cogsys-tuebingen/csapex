/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/timer.h>

using namespace csapex;

const unsigned NodeWorker::timer_history_length_ = 30;

NodeWorker::NodeWorker(Node::Ptr node)
    : node_(node), synchronized_inputs_(false)
{
    assert(node_);
}

void NodeWorker::setSynchronizedInputs(bool s)
{
    synchronized_inputs_ = s;
}

void NodeWorker::forwardMessage(Connector *s)
{
    ConnectorIn* source = dynamic_cast<ConnectorIn*> (s);
    assert(source);

    if(node_->isEnabled()) {
        if(synchronized_inputs_) {
            forwardMessageSynchronized(source);
        } else {
            forwardMessageDirectly(source);
        }
    }


    if(!node_->isError() || node_->errorLevel() != ErrorState::EL_ERROR) {
        node_->setError(false);
    }
}

void NodeWorker::forwardMessageDirectly(ConnectorIn *source)
{
    Timer t;
    node_->messageArrived(source);
    timer_history_.push_back(t.elapsedMs());

    Q_EMIT messageProcessed();
}

void NodeWorker::forwardMessageSynchronized(ConnectorIn *source)
{
    has_msg_[source] = true;

    typedef std::pair<ConnectorIn*, bool> PAIR;
    foreach(const PAIR& pair, has_msg_) {
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
    timer_history_.push_back(t.elapsedMs());

    Q_EMIT messageProcessed();


    foreach(const PAIR& pair, has_msg_) {
        has_msg_[pair.first] = false;
    }
}

void NodeWorker::tick()
{
    if(node_->isEnabled()) {
        node_->tick();
    }

    while(timer_history_.size() > timer_history_length_) {
        timer_history_.pop_front();
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

