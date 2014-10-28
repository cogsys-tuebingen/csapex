/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/thread.h>
#include <csapex/core/settings.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/q_signal_relay.h>

/// SYSTEM
#include <QThread>
#include <QApplication>

using namespace csapex;

static const int DEFAULT_HISTORY_LENGTH = 15;

NodeWorker::NodeWorker(const std::string& type, const UUID& uuid, Settings& settings, Node::Ptr node)
    : settings_(settings), node_(node),
      tick_enabled_(false), ticks_(0),
      source_(false), sink_(false),
      sync(QMutex::Recursive),
      messages_waiting_to_be_sent(false),
      timer_history_pos_(-1),
      thread_initialized_(false), paused_(false), stop_(false)
{
    std::size_t timer_history_length = settings_.get<int>("timer_history_length", DEFAULT_HISTORY_LENGTH);
    timer_history_.resize(timer_history_length);
    apex_assert_hard(timer_history_.size() == timer_history_length);
    apex_assert_hard(timer_history_.capacity() == timer_history_length);

    apex_assert_hard(node_);


    tick_timer_ = new QTimer();
    setTickFrequency(DEFAULT_FREQUENCY);

    QObject::connect(this, SIGNAL(threadSwitchRequested(QThread*, int)), this, SLOT(switchThread(QThread*, int)));

    node->getParameterState()->parameter_added->connect(boost::bind(&NodeWorker::makeParameterConnectable, this, _1));

    node_->initialize(type, uuid, this, &settings_);
}

NodeWorker::~NodeWorker()
{
    tick_immediate_ = false;

    waitUntilFinished();

    QObject::disconnect(this);

    while(!inputs_.empty()) {
        removeInput(*inputs_.begin());
    }
    while(!outputs_.empty()) {
        removeOutput(*outputs_.begin());
    }
    while(!parameter_inputs_.empty()) {
        removeInput(*parameter_inputs_.begin());
    }
    while(!parameter_outputs_.empty()) {
        removeOutput(*parameter_outputs_.begin());
    }

    Q_FOREACH(QObject* cb, callbacks) {
        qt_helper::Call* call = dynamic_cast<qt_helper::Call*>(cb);
        if(call) {
            call->disconnect();
        }
        cb->deleteLater();
    }
    callbacks.clear();
}

Node* NodeWorker::getNode()
{
    return node_.get();
}

UUID NodeWorker::getNodeUUID() const
{
    return node_->getUUID();
}

std::string NodeWorker::getType() const
{
    return node_->getType();
}

bool NodeWorker::isEnabled() const
{
    return node_->getNodeState()->isEnabled();
}

void NodeWorker::setEnabled(bool e)
{
    node_->getNodeState()->setEnabled(e);

    if(!e) {
        node_->setError(false);
    }

    checkIO();
    Q_EMIT enabled(e);
}

bool NodeWorker::canReceive()
{
    bool can_receive = true;
    Q_FOREACH(Input* i, inputs_) {
        if(!i->isConnected() && !i->isOptional()) {
            can_receive = false;
        } /*else if(i->isConnected() && !i->getSource()->isEnabled()) {
            can_receive = false;
        }*/
    }

    return can_receive;
}

template <typename T>
void NodeWorker::makeParameterConnectableImpl(param::Parameter *p)
{
    {
        Input* cin = new Input(settings_, UUID::make_sub(node_->getUUID(), p->name() + "_in"));
        cin->setEnabled(true);
        cin->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());

        parameter_inputs_.push_back(cin);
        connectConnector(cin);
        cin->moveToThread(thread());

        QObject::connect(cin, SIGNAL(messageArrived(Connectable*)), this, SLOT(parameterMessageArrived(Connectable*)));

        param_2_input_[p->name()] = cin;
        input_2_param_[cin] = p;
    }
    {
        Output* cout = new Output(settings_, UUID::make_sub(node_->getUUID(), p->name() + "_out"));
        cout->setEnabled(true);
        cout->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());

        parameter_outputs_.push_back(cout);
        connectConnector(cout);
        cout->moveToThread(thread());

        param_2_output_[p->name()] = cout;
        output_2_param_[cout] = p;
    }
}

void NodeWorker::makeParameterConnectable(param::Parameter* p)
{
    if(p->is<int>()) {
        makeParameterConnectableImpl<int>(p);
    } else if(p->is<double>()) {
        makeParameterConnectableImpl<double>(p);
    } else if(p->is<std::string>()) {
        makeParameterConnectableImpl<std::string>(p);
    } else if(p->is<bool>()) {
        makeParameterConnectableImpl<bool>(p);
    } else if(p->is<std::pair<int, int> >()) {
        makeParameterConnectableImpl<std::pair<int, int> >(p);
    } else if(p->is<std::pair<double, double> >()) {
        makeParameterConnectableImpl<std::pair<double, double> >(p);
    }
    // else: do nothing and ignore the parameter
}

void NodeWorker::makeParametersConnectable()
{
    GenericState::Ptr state = node_->getParameterState();
    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = state->params.begin(), end = state->params.end(); it != end; ++it ) {
        makeParameterConnectable(it->second.get());
    }
}


void NodeWorker::triggerSwitchThreadRequest(QThread* thread, int id)
{
    Q_EMIT threadSwitchRequested(thread, id);
}

void NodeWorker::triggerPanic()
{
    Q_EMIT panic();
}

void NodeWorker::switchThread(QThread *thread, int id)
{
    QObject::disconnect(tick_timer_);

    assert(thread);
    assert(thread != QApplication::instance()->thread());

    moveToThread(thread);

    node_->getNodeState()->setThread(id);

    Q_EMIT threadChanged();

    QObject::connect(tick_timer_, SIGNAL(timeout()), this, SLOT(tick()), Qt::QueuedConnection);
}

void NodeWorker::connectConnector(Connectable *c)
{
    QObject::connect(c, SIGNAL(connectionInProgress(Connectable*,Connectable*)), this, SIGNAL(connectionInProgress(Connectable*,Connectable*)));
    QObject::connect(c, SIGNAL(connectionStart(Connectable*)), this, SIGNAL(connectionStart(Connectable*)));
    QObject::connect(c, SIGNAL(connectionDone(Connectable*)), this, SIGNAL(connectionDone(Connectable*)));
    QObject::connect(c, SIGNAL(connectionDone(Connectable*)), this, SLOT(checkIO()));
    QObject::connect(c, SIGNAL(connectionEnabled(bool)), this, SLOT(checkIO()));
    QObject::connect(c, SIGNAL(connectionRemoved(Connectable*)), this, SLOT(checkIO()));
}


void NodeWorker::disconnectConnector(Connectable */*c*/)
{
}


void NodeWorker::stop()
{
    QObject::disconnect(tick_timer_);

    assert(this->thread() != QApplication::instance()->thread());
    node_->abort();

    QMutexLocker lock(&stop_mutex_);
    stop_ = true;

    Q_FOREACH(Output* i, outputs_) {
        i->stop();
    }
    Q_FOREACH(Input* i, inputs_) {
        i->stop();
    }

    Q_FOREACH(Input* i, inputs_) {
        disconnectConnector(i);
    }
    Q_FOREACH(Output* i, outputs_) {
        disconnectConnector(i);
    }

    QObject::disconnect(this);

    pause(false);
}

void NodeWorker::waitUntilFinished()
{
    QMutexLocker stop_lock(&stop_mutex_);
}

void NodeWorker::reset()
{
    assert(this->thread() != QApplication::instance()->thread());
    node_->abort();
    node_->setError(false);
    Q_FOREACH(Input* i, inputs_) {
        i->reset();
    }
    Q_FOREACH(Output* i, outputs_) {
        i->reset();
        is_ready_[i] = true;
    }
}

bool NodeWorker::isPaused() const
{
    return paused_;
}

void NodeWorker::pause(bool pause)
{
    QMutexLocker lock(&pause_mutex_);
    paused_ = pause;
    continue_.wakeAll();
}

void NodeWorker::killExecution()
{
    // TODO: implement
}

void NodeWorker::messageArrived(Connectable *s)
{
    {
        pause_mutex_.lock();
        while(paused_) {
            continue_.wait(&pause_mutex_);
        }
        pause_mutex_.unlock();
    }

    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);

    QMutexLocker lock(&sync);


    checkIfInputsCanBeProcessed();
}

void NodeWorker::parameterMessageArrived(Connectable *s)
{
    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);

    param::Parameter* p = input_2_param_.at(source);

    if(source->isValue<int>()) {
        p->set<int>(source->getValue<int>());
    } else if(source->isValue<double>()) {
        p->set<double>(source->getValue<double>());
    } else if(source->isValue<std::string>()) {
        p->set<std::string>(source->getValue<std::string>());
    } else {
        node_->ainfo << "parameter " << p->name() << " got a message of unsupported type" << std::endl;
    }

    source->free();
    source->notifyMessageProcessed();
}

void NodeWorker::checkIfInputsCanBeProcessed()
{
    if(isEnabled() && areAllInputsAvailable()) {
        processMessages();
    }
}


void NodeWorker::checkIfOutputIsReady(Connectable* c)
{
    Output* output = dynamic_cast<Output*>(c);
    assert(output);

    QMutexLocker lock(&sync);
    bool every_input_ready = true;
    foreach(Input* input, output->getTargets()) {
        bool blocked = input->isEnabled() && input->isBlocked();
        every_input_ready &= !blocked;
    }

    is_ready_[output] = every_input_ready;

    trySendMessages();
}

void NodeWorker::processMessages()
{
    // everything has a message here

    assert(this->thread() != QApplication::instance()->thread());

    QMutexLocker stop_lock(&stop_mutex_);
    if(stop_) {
        return;
    }

    bool all_inputs_are_present = true;

    {
        QMutexLocker lock(&sync);

        // check for old messages
        int highest_seq_no = -1;
        UUID highest = UUID::NONE;
        foreach(Input* cin, inputs_) {
            if(cin->hasReceived()) {
                int s = cin->sequenceNumber();
                if(s > highest_seq_no) {
                    highest_seq_no = s;
                    highest = cin->getUUID();
                }
            }
        }

        // if a message was dropped we can already return
        //        if(had_old_message) {
        //            return;
        //        }

        // now all sequence numbers must be equal!
        //        Q_FOREACH(const PAIR& pair, has_msg_) {
        //            Input* cin = pair.first;

        //            if(has_msg_[cin]) {
        //                if(highest_seq_no != cin->sequenceNumber()) {
        //                    std::cerr << "input @" << cin->getUUID().getFullName() <<
        //                                 ": assertion failed: highest_seq_no (" << highest_seq_no << ") == cin->seq_no (" << cin->sequenceNumber() << ")" << std::endl;
        //                    apex_assert_hard(false);
        //                }
        //            }
        //        }

        // check if one is "NoMessage";
        Q_FOREACH(Input* cin, inputs_) {
            if(!cin->isOptional() && cin->isMessage<connection_types::NoMessage>()) {
                all_inputs_are_present = false;
            }
        }

        // set output sequence numbers
        for(std::size_t i = 0; i < outputs_.size(); ++i) {
            Output* out = outputs_[i];
            out->setSequenceNumber(highest_seq_no);
        }
    }

    processInputs(all_inputs_are_present);

    stop_lock.unlock();

    if(!isSink()) {
        // send the messages
        trySendMessages();
    } else {
        resetInputs();
    }

    Q_EMIT messageProcessed();
}

bool NodeWorker::areAllInputsAvailable()
{
    QMutexLocker lock(&sync);

    // check if all inputs have messages
    foreach(Input* cin, inputs_) {
        if(!cin->hasReceived()) {
            // connector doesn't have a message
            if(cin->isOptional()) {
                if(cin->isConnected()) {
                    // c is optional and connected, so we have to wait for a message
                    return false;
                } else {
                    // c is optional and not connected, so we can proceed
                    /* do nothing */
                }
            } else {
                // c is mandatory, so we have to wait for a message
                return false;
            }
        }
    }

    return true;
}

void NodeWorker::processInputs(bool all_inputs_are_present)
{
    QMutexLocker lock_in(&sync);

    Timer::Ptr t(new Timer(node_->getUUID()));
    node_->useTimer(t.get());
    if(all_inputs_are_present){
        try {
            node_->process();

        }  catch(const std::exception& e) {
            node_->setError(true, e.what());
        } catch(const std::string& s) {
            node_->setError(true, "Uncatched exception (string) exception: " + s);
        } catch(...) {
            throw;
        }
    }
    t->finish();

    messages_waiting_to_be_sent = true;
    Q_EMIT messagesWaitingToBeSent(true);

    if(++timer_history_pos_ >= (int) timer_history_.size()) {
        timer_history_pos_ = 0;
    }
    timer_history_[timer_history_pos_] = t;
}


Input* NodeWorker::addInput(ConnectionTypePtr type, const std::string& label, bool optional)
{
    int id = inputs_.size();
    Input* c = new Input(settings_, node_.get(), id);
    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);

    registerInput(c);

    return c;
}

Output* NodeWorker::addOutput(ConnectionTypePtr type, const std::string& label)
{
    int id = outputs_.size();
    Output* c = new Output(settings_, node_.get(), id);
    c->setLabel(label);
    c->setType(type);

    registerOutput(c);
    return c;
}

Input* NodeWorker::getParameterInput(const std::string &name) const
{
    std::map<std::string, Input*>::const_iterator it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}

Output* NodeWorker::getParameterOutput(const std::string &name) const
{
    std::map<std::string, Output*>::const_iterator it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return NULL;
    } else {
        return it->second;
    }
}


void NodeWorker::removeInput(Input *in)
{
    std::vector<Input*>::iterator it;
    it = std::find(inputs_.begin(), inputs_.end(), in);

    if(it != inputs_.end()) {
        inputs_.erase(it);
    } else {
        it = std::find(parameter_inputs_.begin(), parameter_inputs_.end(), in);
        if(it != parameter_inputs_.end()) {
            parameter_inputs_.erase(it);
        } else {
            std::cerr << "ERROR: cannot remove input " << in->getUUID().getFullName() << std::endl;
        }
    }

    in->deleteLater();

    disconnectConnector(in);
    Q_EMIT connectorRemoved(in);
}

void NodeWorker::removeOutput(Output *out)
{
    {
        QMutexLocker lock(&sync);
        std::map<Output*, bool>::iterator it = is_ready_.find(out);

        if(it != is_ready_.end()) {
            is_ready_.erase(it);
        }
    }

    std::vector<Output*>::iterator it;
    it = std::find(outputs_.begin(), outputs_.end(), out);

    if(it != outputs_.end()) {
        outputs_.erase(it);
    } else {
        it = std::find(parameter_outputs_.begin(), parameter_outputs_.end(), out);
        if(it != parameter_outputs_.end()) {
            parameter_outputs_.erase(it);
        } else {
            std::cerr << "ERROR: cannot remove output " << out->getUUID().getFullName() << std::endl;
        }
    }

    out->deleteLater();

    disconnectConnector(out);
    Q_EMIT connectorRemoved(out);
}

void NodeWorker::registerInput(Input* in)
{
    inputs_.push_back(in);

    in->moveToThread(thread());

    connectConnector(in);
    QObject::connect(in, SIGNAL(messageArrived(Connectable*)), this, SLOT(messageArrived(Connectable*)));
    QObject::connect(in, SIGNAL(connectionDone(Connectable*)), this, SLOT(trySendMessages()));

    Q_EMIT connectorCreated(in);
}

void NodeWorker::registerOutput(Output* out)
{
    outputs_.push_back(out);

    out->moveToThread(thread());

    is_ready_[out] = true;

    connectConnector(out);
    QObject::connect(out, SIGNAL(messageProcessed(Connectable*)), this, SLOT(checkIfOutputIsReady(Connectable*)));
    QObject::connect(out, SIGNAL(connectionRemoved(Connectable*)), this, SLOT(checkIfOutputIsReady(Connectable*)));
    QObject::connect(out, SIGNAL(connectionDone(Connectable*)), this, SLOT(checkIfOutputIsReady(Connectable*)));

    Q_EMIT connectorCreated(out);
}

Input* NodeWorker::getInput(const UUID& uuid) const
{
    foreach(Input* in, inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }
    foreach(Input* in, parameter_inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }

    return NULL;
}

Output* NodeWorker::getOutput(const UUID& uuid) const
{
    foreach(Output* out, outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }
    foreach(Output* out, parameter_outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }

    return NULL;
}

void NodeWorker::removeInput(const UUID &uuid)
{
    removeInput(getInput(uuid));
}

void NodeWorker::removeOutput(const UUID &uuid)
{
    removeOutput(getOutput(uuid));
}

std::vector<Input*> NodeWorker::getAllInputs() const
{
    std::vector<Input*> result;
    result = inputs_;
    result.insert(result.end(), parameter_inputs_.begin(), parameter_inputs_.end());
    return result;
}

std::vector<Output*> NodeWorker::getAllOutputs() const
{
    std::vector<Output*> result;
    result = outputs_;
    result.insert(result.end(), parameter_outputs_.begin(), parameter_outputs_.end());
    return result;
}

std::vector<Input*> NodeWorker::getMessageInputs() const
{
    return inputs_;
}

std::vector<Output*> NodeWorker::getMessageOutputs() const
{
    return outputs_;
}

std::vector<Input*> NodeWorker::getParameterInputs() const
{
    return parameter_inputs_;
}

std::vector<Output*> NodeWorker::getParameterOutputs() const
{
    return parameter_outputs_;
}

bool NodeWorker::canSendMessages()
{
    QMutexLocker lock(&sync);

    for(std::size_t i = 0; i < outputs_.size(); ++i) {
        Output* out = outputs_[i];
        if(!is_ready_[out]) {
            //            node_->ainfo << "cannot send: " << out->getUUID() << " is not ready" << std::endl;
            return false;
        }
    }

    return true;
}

void NodeWorker::resetInputs()
{
    QMutexLocker lock(&sync);
    Q_FOREACH(Input* cin, inputs_) {
        cin->free();
    }
    Q_FOREACH(Input* cin, inputs_) {
        cin->notifyMessageProcessed();
    }
}

void NodeWorker::trySendMessages()
{
    QMutexLocker lock(&sync);

    if(!messages_waiting_to_be_sent) {
        return;
    }

    if(!canSendMessages()) {
        return;
    }

    messages_waiting_to_be_sent = false;
    Q_EMIT messagesWaitingToBeSent(false);

    for(std::size_t i = 0; i < outputs_.size(); ++i) {
        Output* out = outputs_[i];
        if(out->isConnected()) {
            bool every_connected_input_disabled = false;
            foreach(Input* input, out->getTargets()) {
                bool disabled = !input->isEnabled();
                every_connected_input_disabled |= disabled;
            }

            if(every_connected_input_disabled) {
                is_ready_[out] = true;
            } else {
                is_ready_[out] = false;
            }

        } else {
            is_ready_[out] = true;
        }
    }

    for(std::size_t i = 0; i < parameter_outputs_.size(); ++i) {
        Output* out = parameter_outputs_[i];
        if(out->isConnected()) {
            param::Parameter* p = output_2_param_.at(out);
            if(p->is<int>())
                out->publish(p->as<int>());
            else if(p->is<double>())
                out->publish(p->as<double>());
            else if(p->is<std::string>())
                out->publish(p->as<std::string>());
            out->sendMessages();
        }
    }

    for(std::size_t i = 0; i < outputs_.size(); ++i) {
        Output* out = outputs_[i];
        out->sendMessages();
    }

    resetInputs();
}

void NodeWorker::setTickFrequency(double f)
{
    if(tick_timer_->isActive()) {
        tick_timer_->stop();
    }
    if(f == 0.0) {
        return;
    }

    tick_immediate_ = (f >= 999.9);

    if(tick_immediate_) {
        tick_timer_->setSingleShot(true);
    } else {
        tick_timer_->setInterval(1000. / f);
        tick_timer_->setSingleShot(false);
    }
    tick_timer_->start();
}

void NodeWorker::setTickEnabled(bool enabled)
{
    tick_enabled_ = enabled;
}

bool NodeWorker::isTickEnabled() const
{
    return tick_enabled_;
}


void NodeWorker::setIsSource(bool source)
{
    source_ = source;
}

bool NodeWorker::isSource() const
{
    if(source_) {
        return true;
    }

    // check if there are no (mandatory) inputs -> then it's a virtual source
    // TODO: remove and refactor old plugins
    if(!inputs_.empty()) {
        foreach(Input* in, inputs_) {
            if(!in->isOptional() || in->isConnected()) {
                return false;
            }
        }
    }

    return true;
}


void NodeWorker::setIsSink(bool sink)
{
    sink_ = sink;
}

bool NodeWorker::isSink() const
{
    return sink_ || outputs_.empty();
}

void NodeWorker::tick()
{
    while(true) {
        {
            pause_mutex_.lock();
            while(paused_) {
                continue_.wait(&pause_mutex_);
            }
            pause_mutex_.unlock();
        }

        QMutexLocker lock(&stop_mutex_);
        if(stop_) {
            return;
        }

        node_->checkConditions(false);
        checkParameters();

        if(!thread_initialized_) {
            thread::set_name(thread()->objectName().toStdString().c_str());
            thread_initialized_ = true;
        }


        if(isEnabled() && (isTickEnabled() || isSource())) {
            assert(this->thread() != QApplication::instance()->thread());

            if(!canSendMessages() /*|| ticks_ != 0*/) {
                return;
            }

            node_->tick();
            ++ticks_;

            processInputs(isSource());
            trySendMessages();
        }

        //            node_->tick();

        // if there is a message: send!
        //            bool has_msg = false;
        //            for(std::size_t i = 0; i < outputs_.size(); ++i) {
        //                Output* out = outputs_[i];
        //                if(out->hasMessage()) {
        //                    has_msg = true;
        //                }
        //            }
        //            if(has_msg) {
        //                trySendMessages();
        //            } else {
        //                node_->ainfo << "no message output" << std::endl;
        //            }
        //        }

        if(!tick_immediate_) {
            return;
        }
    }
}

void NodeWorker::checkParameters()
{
    // check if a parameter-connection has a message

    // check if a parameter was changed
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

void NodeWorker::checkIO()
{
    if(isEnabled()) {
        enableInput(canReceive());
        enableOutput(canReceive());
    } else {
        enableInput(false);
        enableOutput(false);
    }
}


void NodeWorker::enableIO(bool enable)
{
    enableInput(canReceive() && enable);
    enableOutput(enable);
}

void NodeWorker::enableInput (bool enable)
{
    Q_FOREACH(Input* i, inputs_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}


void NodeWorker::enableOutput (bool enable)
{
    Q_FOREACH(Output* o, outputs_) {
        if(enable) {
            o->enable();
        } else {
            o->disable();
        }
    }
}

void NodeWorker::triggerError(bool e, const std::string &what)
{
    node_->setError(e, what);
}



void NodeWorker::setIOError(bool error)
{
    Q_FOREACH(Input* i, inputs_) {
        i->setErrorSilent(error);
    }
    Q_FOREACH(Output* i, outputs_) {
        i->setErrorSilent(error);
    }
    enableIO(!error);
}

void NodeWorker::setMinimized(bool min)
{
    node_->getNodeState()->setMinimized(min);
}
