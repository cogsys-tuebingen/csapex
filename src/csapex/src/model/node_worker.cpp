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
    for(unsigned i = 0; i < connections_.size(); ++i) {
        connections_[i].disconnect();
    }
}

void NodeWorker::addParameter(param::Parameter* param)
{
    connections_.push_back(parameter_changed(*param).connect(boost::bind(&NodeWorker::parameterChanged, this, _1)));
    connections_.push_back(parameter_enabled(*param).connect(boost::bind(&NodeWorker::parameterEnabled, this, _1, _2)));
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
        checkConditions(false);
    }
}

void NodeWorker::checkConditions(bool silent)
{
    bool change = false;
    node_->setParameterSetSilence(true);
    for(std::map<param::Parameter*, boost::function<bool()> >::iterator it = conditions_.begin(); it != conditions_.end(); ++it) {
        param::Parameter* p = it->first;
        bool should_be_enabled = it->second();
        if(should_be_enabled != p->isEnabled()) {
            it->first->setEnabled(should_be_enabled);
            change = true;
        }
    }
    node_->setParameterSetSilence(false);

    if(change && !silent) {
        node_->triggerParameterSetChanged();
    }
}

void NodeWorker::pause(bool pause)
{
    QMutexLocker lock(&pause_mutex_);
    paused_ = pause;
    continue_.wakeAll();
}

void NodeWorker::parameterChanged(param::Parameter *param, boost::function<void(param::Parameter *)> cb)
{
    QMutexLocker lock(&changed_params_mutex_);
    changed_params_.push_back(std::make_pair(param, cb));
}

void NodeWorker::parameterEnabled(param::Parameter */*param*/, bool /*enabled*/)
{
    node_->triggerParameterSetChanged();
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
        QMutexLocker lock(&changed_params_mutex_);
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
        QMutexLocker lock(&changed_params_mutex_);
        node_->tick();
    }
    while(timer_history_.size() > Settings::timer_history_length_) {
        timer_history_.pop_front();
    }
}

void NodeWorker::checkParameters()
{
    std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > changed_params;
    {
        QMutexLocker lock(&changed_params_mutex_);
        changed_params = changed_params_;
        changed_params_.clear();
    }

    if(!changed_params.empty()) {
        typedef std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > cbs;
        for(cbs::iterator it = changed_params.begin(); it != changed_params.end();) {
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

