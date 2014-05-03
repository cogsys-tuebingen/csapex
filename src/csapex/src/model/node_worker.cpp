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
    : node_(node), thread_initialized_(false), paused_(false)
{
    assert(node_);

    timer_ = new QTimer();
    setTickFrequency(DEFAULT_FREQUENCY);

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(tick()));
}

NodeWorker::~NodeWorker()
{
}

void NodeWorker::pause(bool pause)
{
    QMutexLocker lock(&pause_mutex_);
    paused_ = pause;
    continue_.wakeAll();
}

void NodeWorker::forwardMessage(Connectable *s)
{
    {
        pause_mutex_.lock();
        while(paused_) {
            continue_.wait(&pause_mutex_);
        }
        pause_mutex_.unlock();
    }

    ConnectorIn* source = dynamic_cast<ConnectorIn*> (s);
    assert(source);

    if(node_->isEnabled()) {
        Q_EMIT messagesReceived();

        forwardMessageSynchronized(source);
    }
}

void NodeWorker::addInput(ConnectorIn *source)
{
    has_msg_[source] = false;
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
    assert(!has_msg_[source]);
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

    Timer::Ptr t(new Timer(node_->getUUID()));
    node_->useTimer(t.get());
    {
        boost::shared_ptr<QMutexLocker> lock = node_->getParamLock();
        node_->process();
    }
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


    Q_FOREACH(const PAIR& pair, has_msg_) {
        ConnectorIn* cin = pair.first;
        cin->free();
    }

    Q_EMIT messageProcessed();
}

void NodeWorker::setTickFrequency(double f)
{
    if(timer_->isActive()) {
        timer_->stop();
    }
    if(f == 0.0) {
        return;
    }
    timer_->setInterval(1000. / f);
    timer_->setSingleShot(false);
    timer_->start();
}

void NodeWorker::tick()
{
    if(!thread_initialized_) {
        thread::set_name(node_->getUUID().getShortName().c_str());
        thread_initialized_ = true;
    }

    if(node_->isEnabled()) {
        boost::shared_ptr<QMutexLocker> lock = node_->getParamLock();
        node_->tick();
    }
    while(timer_history_.size() > Settings::timer_history_length_) {
        timer_history_.pop_front();
    }
}

void NodeWorker::checkParameters()
{
    Parameterizable::ChangedParameterList changed_params = node_->getChangedParameters();

    if(!changed_params.empty()) {
        for(Parameterizable::ChangedParameterList::iterator it = changed_params.begin(); it != changed_params.end();) {
            try {
                it->second(it->first);

            } catch(const std::exception& e) {
                it = changed_params.erase(it);
                throw;
            }

            it = changed_params.erase(it);
        }
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

