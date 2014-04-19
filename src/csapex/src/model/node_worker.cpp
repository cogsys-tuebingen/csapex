/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/thread.h>
#include <csapex/core/settings.h>

using namespace csapex;

NodeWorker::NodeWorker(Node* node)
    : node_(node), synchronized_inputs_(false), thread_initialized_(false), is_processing_(false)
{
    assert(node_);
}

NodeWorker::~NodeWorker()
{
    for(unsigned i = 0; i < connections_.size(); ++i) {
        connections_[i].disconnect();
    }
}

bool NodeWorker::isProcessing()
{
    return is_processing_;
}

void NodeWorker::addParameter(param::Parameter* param)
{
    connections_.push_back(parameter_changed(*param).connect(boost::bind(&NodeWorker::parameterChanged, this, _1)));
}

void NodeWorker::addParameterCallback(param::Parameter* param, boost::function<void(param::Parameter *)> cb)
{
    connections_.push_back(parameter_changed(*param).connect(boost::bind(&NodeWorker::parameterChanged, this, _1, cb)));
}

void NodeWorker::addParameterCondition(param::Parameter* param, boost::function<bool ()> enable_condition)
{
    conditions_[param] = enable_condition;
}

void NodeWorker::parameterChanged(param::Parameter *)
{
    if(!conditions_.empty()) {
        checkConditions();
    }
}

void NodeWorker::checkConditions()
{
    for(std::map<param::Parameter*, boost::function<bool()> >::iterator it = conditions_.begin(); it != conditions_.end(); ++it) {
        it->first->setEnabled(it->second());
    }
}

void NodeWorker::parameterChanged(param::Parameter *param, boost::function<void(param::Parameter *)> cb)
{
   // QMutexLocker lock(&changed_params_mutex_);
    changed_params_.push_back(std::make_pair(param, cb));
}

void NodeWorker::setSynchronizedInputs(bool s)
{
    synchronized_inputs_ = s;

    typedef std::pair<ConnectorIn*, bool> PAIR;
    Q_FOREACH(const PAIR& pair, has_msg_) {
        pair.first->setLegacy(!synchronized_inputs_);
    }
}

bool NodeWorker::isSynchronizedInputs() const
{
    return synchronized_inputs_;
}

void NodeWorker::forwardMessage(Connectable *s)
{
    ConnectorIn* source = dynamic_cast<ConnectorIn*> (s);
    assert(source);

//    if(node_->isError()) {
//        return;
//    }

    if(node_->isEnabled()) {
        if(synchronized_inputs_) {
            forwardMessageSynchronized(source);
        } else {
            forwardMessageDirectly(source);
        }
    }


//    if(!node_->isError() || node_->errorLevel() != ErrorState::EL_ERROR) {
//        node_->setError(false);
//    }
}

void NodeWorker::forwardMessageDirectly(ConnectorIn *source)
{
    assert(!is_processing_);

    Timer::Ptr t(new Timer(node_->getUUID()));
    is_processing_ = true;

    node_->useTimer(t.get());
    node_->messageArrived(source);

    t->finish();

    timer_history_.push_back(t);

    is_processing_ = false;
    Q_EMIT messageProcessed();
}

void NodeWorker::setProcessing(bool p)
{
    is_processing_ = p;
}

bool NodeWorker::isProcessing() const
{
    return is_processing_;
}

void NodeWorker::addInput(ConnectorIn *source)
{
    has_msg_[source] = false;
    source->setLegacy(!synchronized_inputs_);
}

void NodeWorker::removeInput(ConnectorIn *source)
{
    std::map<ConnectorIn*, bool>::iterator it = has_msg_.find(source);

    if(it != has_msg_.end()) {
        has_msg_.erase(it);
    }
}

void NodeWorker::forwardMessageSynchronized(ConnectorIn *source)
{
    assert(!is_processing_);

    has_msg_[source] = true;

    typedef std::pair<ConnectorIn*, bool> PAIR;

    Q_FOREACH(const PAIR& pair, has_msg_) {
        ConnectorIn* c = pair.first;
        if(!pair.second) {
            // connector doesn't have a message
            if(c->isAsync()) {
                // do not wait for this input
            } else if(c->isOptional()) {
                if(c->isConnected()) {
                    // c is optional and connected, so we have to wait for a message
                    return;
                } else {
                    // c is optional and not connected, so we can proceed
                    /* do nothing */
                }
            } else {
                // c is mandatory, so we have to wait for a message
                return;
            }
        }
    }

    is_processing_ = true;

    Timer::Ptr t(new Timer(node_->getUUID()));
    node_->useTimer(t.get());
    node_->process();
    t->finish();
    timer_history_.push_back(t);

    // reset all edges
    Q_FOREACH(const PAIR& pair, has_msg_) {
        ConnectorIn* cin = pair.first;
        if(cin->isConnected() && cin->getSource()->isAsync()) {
            continue;
        }

        has_msg_[cin] = false;
    }

    is_processing_ = false;
    Q_EMIT messageProcessed();
}

void NodeWorker::tick()
{
    if(!thread_initialized_) {
        thread::set_name(node_->getUUID().getShortName().c_str());
        thread_initialized_ = true;
    }

    if(node_->isEnabled()) {
        node_->tick();
    }

    {
        QMutexLocker lock(&changed_params_mutex_);
        typedef std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > cbs;
        for(cbs::iterator it = changed_params_.begin(); it != changed_params_.end();) {
            try {
                it->second(it->first);

            } catch(const std::exception& e) {
                it = changed_params_.erase(it);
                throw;
            }

            it = changed_params_.erase(it);
        }

        changed_params_.clear();
    }

    while(timer_history_.size() > Settings::timer_history_length_) {
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

