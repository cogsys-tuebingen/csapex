/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/utility/timer.h>
#include <csapex/utility/thread.h>
#include <csapex/core/settings.h>
#include <csapex/model/node_state.h>
#include <csapex/utility/q_signal_relay.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <utils_param/trigger_parameter.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/node_modifier.h>

/// SYSTEM
#include <QThread>
#include <QApplication>
#include <thread>

using namespace csapex;

const double NodeWorker::DEFAULT_FREQUENCY = 30.0;

NodeWorker::NodeWorker(const std::string& type, const UUID& uuid, Settings& settings, Node::Ptr node)
    : Unique(uuid),
      settings_(settings),
      node_type_(type), node_(node), node_state_(std::make_shared<NodeState>(this)),
      transition_in_(std::make_shared<InputTransition>(this)),
      transition_out_(std::make_shared<OutputTransition>(this)),
      is_setup_(false), state_(State::IDLE),
      trigger_tick_done_(nullptr), trigger_process_done_(nullptr),
      tick_enabled_(false), ticks_(0),
      source_(false), sink_(false), level_(0),
      thread_initialized_(false), paused_(false), stop_(false),
      profiling_(false)
{
    apex_assert_hard(node_);

    tick_timer_ = new QTimer();
    setTickFrequency(DEFAULT_FREQUENCY);

    QObject::connect(this, SIGNAL(threadSwitchRequested(QThread*, int)), this, SLOT(switchThread(QThread*, int)));

    node_state_->setLabel(uuid);
    modifier_ = std::make_shared<NodeModifier>(this);
    node_->initialize(uuid, modifier_.get());

    bool params_created_in_constructor = node_->getParameterCount() != 0;

    if(params_created_in_constructor) {
        makeParametersConnectable();
    }

    node_->getParameterState()->parameter_added->connect(std::bind(&NodeWorker::makeParameterConnectable, this, std::placeholders::_1));

    addSlot("enable", std::bind(&NodeWorker::setEnabled, this, true), true);
    addSlot("disable", std::bind(&NodeWorker::setEnabled, this, false), false);

    trigger_tick_done_ = addTrigger("ticked");
    trigger_process_done_ = addTrigger("inputs\nprocessed");

    node_->doSetup();

    if(isSource()) {
        setTickEnabled(true);
    }

    is_setup_ = true;

    if(params_created_in_constructor) {
        node_->awarn << "Node creates parameters in its constructor! Please implement 'setupParameters'" << std::endl;
    }
}

NodeWorker::~NodeWorker()
{
    tick_immediate_ = false;
    is_setup_ = false;

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
    while(!slots_.empty()) {
        removeSlot(*slots_.begin());
    }
    while(!triggers_.empty()) {
        removeTrigger(*triggers_.begin());
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

InputTransition* NodeWorker::getInputTransition() const
{
    return transition_in_.get();
}

OutputTransition* NodeWorker::getOutputTransition() const
{
    return transition_out_.get();
}

void NodeWorker::setNodeState(NodeStatePtr memento)
{
    std::string old_label = node_state_->getLabel();

    *node_state_ = *memento;

    node_state_->setParent(this);
    if(memento->getParameterState()) {
        node_->setParameterState(memento->getParameterState());
    }

    node_state_->enabled_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));
    node_state_->flipped_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));
    node_state_->label_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));
    node_state_->minimized_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));
    node_state_->parent_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));
    node_state_->pos_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));
    node_state_->thread_changed->connect(std::bind(&NodeWorker::triggerNodeStateChanged, this));

    triggerNodeStateChanged();

    node_->stateChanged();

    if(node_state_->getLabel().empty()) {
        if(old_label.empty()) {
            node_state_->setLabel(getUUID().getShortName());
        } else {
            node_state_->setLabel(old_label);
        }
    }
}

void NodeWorker::triggerNodeStateChanged()
{
    Q_EMIT nodeStateChanged();
}

NodeState::Ptr NodeWorker::getNodeStateCopy() const
{
    apex_assert_hard(node_state_);

    NodeState::Ptr memento(new NodeState(this));
    *memento = *node_state_;

    memento->setParameterState(node_->getParameterStateClone());

    return memento;
}

NodeState::Ptr NodeWorker::getNodeState()
{
    apex_assert_hard(node_state_);

    return node_state_;
}


Node* NodeWorker::getNode() const
{
    return node_.get();
}

NodeWorker::State NodeWorker::getState() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return state_;
}

void NodeWorker::setState(State state)
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    switch(state) {
    case State::IDLE:
        apex_assert_hard(state_ == State::WAITING_FOR_RESET);
        break;
    case State::FIRED:
        apex_assert_hard(state_ == State::ENABLED);
        break;
    case State::PROCESSING:
        apex_assert_hard(state_ == State::FIRED);
        break;

    default:
        break;
    }

    state_ = state;
}

std::string NodeWorker::getType() const
{
    return node_type_;
}

bool NodeWorker::isEnabled() const
{
    return node_state_->isEnabled();
}

void NodeWorker::setEnabled(bool e)
{
    node_state_->setEnabled(e);

    if(!e) {
        setError(false);
    }

    checkIO();
    Q_EMIT enabled(e);
}

bool NodeWorker::canProcess()
{
    return canReceive();
}

bool NodeWorker::canReceive()
{
    for(Input* i : inputs_) {
        if(!i->isConnected() && !i->isOptional()) {
            return false;
        }
    }
    return true;
}

template <typename T>
void NodeWorker::makeParameterConnectableImpl(param::Parameter *p)
{
    if(param_2_input_.find(p->name()) != param_2_input_.end()) {
        return;
    }

    {
        Input* cin = new Input(transition_in_.get(), UUID::make_sub(getUUID(), std::string("in_") + p->name()));
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
        Output* cout = new StaticOutput(transition_out_.get(), UUID::make_sub(getUUID(), std::string("out_") + p->name()));
        cout->setEnabled(true);
        cout->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());

        parameter_outputs_.push_back(cout);
        connectConnector(cout);
        cout->moveToThread(thread());

        p->parameter_changed.connect(std::bind(&NodeWorker::publishParameter, this, p));

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

    param::TriggerParameter* t = dynamic_cast<param::TriggerParameter*>(p);
    if(t) {
        Trigger* trigger = addTrigger(t->name());
        addSlot(t->name(), std::bind(&param::TriggerParameter::trigger, t), false);
        node_->addParameterCallback(t, std::bind(&Trigger::trigger, trigger));
    }
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

void NodeWorker::triggerProcess()
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    apex_assert_hard(state_ == State::ENABLED);

    apex_assert_hard(transition_out_->canSendMessages());
    setState(State::FIRED);
    Q_EMIT processRequested();
}

void NodeWorker::triggerCheckTransitions()
{
    Q_EMIT checkTransitionsRequested();
}

void NodeWorker::switchThread(QThread *thread, int id)
{
    QObject::disconnect(tick_timer_);

    assert(thread);

    foreach(Input* input, inputs_){
        input->moveToThread(thread);
    }
    foreach(Input* input, parameter_inputs_){
        input->moveToThread(thread);
    }
    foreach(Output* output, outputs_){
        output->moveToThread(thread);
    }
    foreach(Output* output, parameter_outputs_){
        output->moveToThread(thread);
    }
    foreach(Slot* slot, slots_){
        slot->moveToThread(thread);
    }
    foreach(Trigger* trigger, triggers_){
        trigger->moveToThread(thread);
    }
    moveToThread(thread);

    assertNotInGuiThread();

    node_state_->setThread(id);

    Q_EMIT threadChanged();

    QObject::connect(tick_timer_, SIGNAL(timeout()), this, SLOT(tick()));

    QObject::connect(this, SIGNAL(processRequested()), this, SLOT(processMessages()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(tickRequested()), this, SLOT(tick()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(messagesProcessed()), this, SLOT(prepareForNextProcess()), Qt::QueuedConnection);
    QObject::connect(this, SIGNAL(checkTransitionsRequested()), this, SLOT(checkTransitions()), Qt::QueuedConnection);
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


void NodeWorker::disconnectConnector(Connectable* c)
{
    disconnect(c);
}


void NodeWorker::stop()
{
    QObject::disconnect(tick_timer_);

    assertNotInGuiThread();
    node_->abort();

    std::lock_guard<std::recursive_mutex> lock(stop_mutex_);
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
    std::unique_lock<std::recursive_mutex> stop_lock(stop_mutex_);
}

void NodeWorker::reset()
{
    assertNotInGuiThread();

    node_->abort();
    setError(false);
    for(Input* i : inputs_) {
        i->reset();
    }
    for(Output* o : outputs_) {
        o->reset();
    }
}

bool NodeWorker::isPaused() const
{
    return paused_;
}

void NodeWorker::setProfiling(bool profiling)
{
    profiling_ = profiling;

    {
        std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
        timer_history_.clear();
    }

    if(profiling_) {
        Q_EMIT startProfiling(this);
    } else {
        Q_EMIT stopProfiling(this);
    }
}

bool NodeWorker::isProfiling() const
{
    return profiling_;
}

void NodeWorker::pause(bool pause)
{
    std::lock_guard<std::recursive_mutex> lock(pause_mutex_);
    paused_ = pause;
    continue_.notify_all();
}

void NodeWorker::killExecution()
{
    // TODO: implement
}

void NodeWorker::messageArrived(Connectable */*s*/)
{
    //    assertNotInGuiThread();

    //    {
    //        std::unique_lock<std::recursive_mutex> lock(pause_mutex_);
    //        while(paused_) {
    //            continue_.wait(lock);
    //        }
    //    }

    //    Input* source = dynamic_cast<Input*> (s);
    //    apex_assert_hard(source);

    //    std::lock_guard<std::recursive_mutex> lock(sync);

    //    checkIfInputsCanBeProcessed();
}

void NodeWorker::parameterMessageArrived(Connectable *s)
{
    assertNotInGuiThread();
    {
        std::unique_lock<std::recursive_mutex> pause_lock(pause_mutex_);
        while(paused_) {
            continue_.wait(pause_lock);
        }
    }

    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);

    param::Parameter* p = input_2_param_.at(source);

    if(msg::isValue<int>(source)) {
        p->set<int>(msg::getValue<int>(source));
    } else if(msg::isValue<double>(source)) {
        p->set<double>(msg::getValue<double>(source));
    } else if(msg::isValue<std::string>(source)) {
        p->set<std::string>(msg::getValue<std::string>(source));
    } else if(msg::hasMessage(source) && !msg::isMessage<connection_types::NoMessage>(source)) {
        node_->ainfo << "parameter " << p->name() << " got a message of unsupported type" << std::endl;
    }

    source->free();
    source->notifyMessageProcessed();

    publishParameter(p);
}

void NodeWorker::checkIfInputsCanBeProcessed()
{
    //    if(isEnabled() && areAllInputsAvailable()) {
    //        processMessages();
    //    }
}


void NodeWorker::outputConnectionChanged(Connectable*)
{
    std::lock_guard<std::recursive_mutex> lock(sync);

    if(state_ == State::WAITING_FOR_OUTPUTS) {
        transition_out_->updateOutputs();
    }

    triggerCheckTransitions();
}

void NodeWorker::processMessages()
{
    std::unique_lock<std::recursive_mutex> stop_lock(stop_mutex_);

    std::lock_guard<std::recursive_mutex> lock(sync);

    assertNotInGuiThread();

    if(stop_) {
        notifyMessagesProcessed();
        return;
    }

    apex_assert_hard(state_ == State::FIRED);
    apex_assert_hard(!transition_in_->hasUnestablishedConnection());
    apex_assert_hard(!transition_out_->hasUnestablishedConnection());
    apex_assert_hard(transition_out_->canSendMessages());
    apex_assert_hard(isEnabled());
    setState(State::PROCESSING);

    {
        std::unique_lock<std::recursive_mutex> pause_lock(pause_mutex_);
        while(paused_) {
            continue_.wait(pause_lock);
        }
    }

    // everything has a message here

    bool all_inputs_are_present = true;

    {
        std::lock_guard<std::recursive_mutex> lock(sync);

        // check if one is "NoMessage";
        for(Input* cin : inputs_) {
            apex_assert_hard(cin->hasReceived() || (cin->isOptional() && !cin->isConnected()));
            if(!cin->isOptional() && !msg::hasMessage(cin)) {
                all_inputs_are_present = false;
            }
        }
    }

    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
    transition_out_->setConnectionsReadyToReceive();
    transition_out_->clearOutputs();

    if(all_inputs_are_present) {
        std::lock_guard<std::recursive_mutex> lock(sync);


        Timer::Ptr t = nullptr;

        if(profiling_) {
            t.reset(new Timer(getUUID()));
            Q_EMIT timerStarted(this, PROCESS, t->startTimeMs());
        }
        node_->useTimer(t.get());

        try {
            //            node_->aerr << "processing" << std::endl;
            node_->process();

            if(trigger_process_done_->isConnected()) {
                t->step("trigger process done");
                trigger_process_done_->trigger();
            }

        }  catch(const std::exception& e) {
            setError(true, e.what());
        } catch(const std::string& s) {
            setError(true, "Uncatched exception (string) exception: " + s);
        } catch(...) {
            throw;
        }

        if(t) {
            finishTimer(t);
        }
    }

    sendMessages();
}

bool NodeWorker::areAllInputsAvailable()
{
    std::lock_guard<std::recursive_mutex> lock(sync);

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

void NodeWorker::finishTimer(Timer::Ptr t)
{
    t->finish();

    {
        std::lock_guard<std::recursive_mutex> lock(timer_mutex_);
        timer_history_.push_back(t);
    }
    Q_EMIT timerStopped(this, t->stopTimeMs());
}

std::vector<TimerPtr> NodeWorker::extractLatestTimers()
{
    std::lock_guard<std::recursive_mutex> lock(timer_mutex_);

    std::vector<TimerPtr> result = timer_history_;
    timer_history_.clear();
    return result;
}

Input* NodeWorker::addInput(ConnectionTypePtr type, const std::string& label, bool dynamic, bool optional)
{
    int id = inputs_.size();
    Input* c = nullptr;
    if(dynamic) {
        c = new DynamicInput(transition_in_.get(), this, id);
    } else {
        c = new Input(transition_in_.get(), this, id);
    }
    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);

    registerInput(c);

    return c;
}

Output* NodeWorker::addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic)
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    int id = outputs_.size();
    Output* c = nullptr;
    if(dynamic) {
        c = new DynamicOutput(transition_out_.get(), this, id);
    } else {
        c = new StaticOutput(transition_out_.get(), this, id);
    }
    c->setLabel(label);
    c->setType(type);

    registerOutput(c);
    return c;
}

Slot* NodeWorker::addSlot(const std::string& label, std::function<void()> callback, bool active)
{
    int id = slots_.size();
    Slot* slot = new Slot(callback, this, id, active);
    slot->setLabel(label);
    slot->setEnabled(true);

    registerSlot(slot);

    return slot;
}

Trigger* NodeWorker::addTrigger(const std::string& label)
{
    int id = triggers_.size();
    Trigger* trigger = new Trigger(this, id);
    trigger->setLabel(label);
    trigger->setEnabled(true);

    registerTrigger(trigger);

    return trigger;
}

Input* NodeWorker::getParameterInput(const std::string &name) const
{
    std::map<std::string, Input*>::const_iterator it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return nullptr;
    } else {
        return it->second;
    }
}

Output* NodeWorker::getParameterOutput(const std::string &name) const
{
    std::map<std::string, Output*>::const_iterator it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return nullptr;
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

    disconnectConnector(in);
    Q_EMIT connectorRemoved(in);

    in->deleteLater();
}

void NodeWorker::removeOutput(Output *out)
{
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

    disconnectConnector(out);
    Q_EMIT connectorRemoved(out);

    out->deleteLater();
}

void NodeWorker::removeSlot(Slot *s)
{
    std::vector<Slot*>::iterator it;
    it = std::find(slots_.begin(), slots_.end(), s);

    if(it != slots_.end()) {
        slots_.erase(it);
    }

    disconnectConnector(s);
    Q_EMIT connectorRemoved(s);

    s->deleteLater();
}

void NodeWorker::removeTrigger(Trigger *t)
{
    std::vector<Trigger*>::iterator it;
    it = std::find(triggers_.begin(), triggers_.end(), t);

    if(it != triggers_.end()) {
        triggers_.erase(it);
    }

    disconnectConnector(t);
    Q_EMIT connectorRemoved(t);

    t->deleteLater();
}

void NodeWorker::registerInput(Input* in)
{
    inputs_.push_back(in);

    in->moveToThread(thread());

    connectConnector(in);
    QObject::connect(in, SIGNAL(messageArrived(Connectable*)), this, SLOT(messageArrived(Connectable*)));
    //    QObject::connect(in, SIGNAL(connectionDone(Connectable*)), this, SLOT(trySendMessages()));

    Q_EMIT connectorCreated(in);
}

void NodeWorker::registerOutput(Output* out)
{
    outputs_.push_back(out);

    out->moveToThread(thread());

    connectConnector(out);
    QObject::connect(out, SIGNAL(messageProcessed(Connectable*)), this, SLOT(outputConnectionChanged(Connectable*)));
    QObject::connect(out, SIGNAL(connectionRemoved(Connectable*)), this, SLOT(outputConnectionChanged(Connectable*)));
    QObject::connect(out, SIGNAL(connectionDone(Connectable*)), this, SLOT(outputConnectionChanged(Connectable*)));

    Q_EMIT connectorCreated(out);
}

void NodeWorker::registerSlot(Slot* s)
{
    slots_.push_back(s);

    s->moveToThread(thread());

    connectConnector(s);
    QObject::connect(s, SIGNAL(messageArrived(Connectable*)), this, SLOT(messageArrived(Connectable*)));
    //    QObject::connect(s, SIGNAL(connectionDone(Connectable*)), this, SLOT(trySendMessages()));
    QObject::connect(s, SIGNAL(triggered()), s, SLOT(handleTrigger()), Qt::QueuedConnection);

    Q_EMIT connectorCreated(s);
}

void NodeWorker::registerTrigger(Trigger* t)
{
    triggers_.push_back(t);

    t->moveToThread(thread());

    connectConnector(t);
    //    QObject::connect(t, SIGNAL(messageProcessed(Connectable*)), this, SLOT(checkIfOutputIsReady(Connectable*)));
    //    QObject::connect(t, SIGNAL(connectionRemoved(Connectable*)), this, SLOT(checkIfOutputIsReady(Connectable*)));
    //    QObject::connect(t, SIGNAL(connectionDone(Connectable*)), this, SLOT(checkIfOutputIsReady(Connectable*)));

    Q_EMIT connectorCreated(t);
}

Connectable* NodeWorker::getConnector(const UUID &uuid) const
{
    std::string type = uuid.type();

    if(type == "in") {
        return getInput(uuid);
    } else if(type == "out") {
        return getOutput(uuid);
    } else if(type == "slot") {
        return getSlot(uuid);
    } else if(type == "trigger") {
        return getTrigger(uuid);
    } else {
        throw std::logic_error(std::string("the connector type '") + type + "' is unknown.");
    }
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

    return nullptr;
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

    return nullptr;
}


Slot* NodeWorker::getSlot(const UUID& uuid) const
{
    foreach(Slot* s, slots_) {
        if(s->getUUID() == uuid) {
            return s;
        }
    }

    return nullptr;
}


Trigger* NodeWorker::getTrigger(const UUID& uuid) const
{
    foreach(Trigger* t, triggers_) {
        if(t->getUUID() == uuid) {
            return t;
        }
    }

    return nullptr;
}

void NodeWorker::removeInput(const UUID &uuid)
{
    removeInput(getInput(uuid));
}

void NodeWorker::removeOutput(const UUID &uuid)
{
    removeOutput(getOutput(uuid));
}

void NodeWorker::removeSlot(const UUID &uuid)
{
    removeSlot(getSlot(uuid));
}

void NodeWorker::removeTrigger(const UUID &uuid)
{
    removeTrigger(getTrigger(uuid));
}

std::vector<Connectable*> NodeWorker::getAllConnectors() const
{
    std::vector<Connectable*> result;
    result.insert(result.end(), inputs_.begin(), inputs_.end());
    result.insert(result.end(), outputs_.begin(), outputs_.end());
    result.insert(result.end(), parameter_inputs_.begin(), parameter_inputs_.end());
    result.insert(result.end(), parameter_outputs_.begin(), parameter_outputs_.end());
    result.insert(result.end(), triggers_.begin(), triggers_.end());
    result.insert(result.end(), slots_.begin(), slots_.end());
    return result;
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

std::vector<Slot*> NodeWorker::getSlots() const
{
    return slots_;
}

std::vector<Trigger*> NodeWorker::getTriggers() const
{
    return triggers_;
}

std::vector<Input*> NodeWorker::getParameterInputs() const
{
    return parameter_inputs_;
}

std::vector<Output*> NodeWorker::getParameterOutputs() const
{
    return parameter_outputs_;
}

//bool NodeWorker::canSendMessages()
//{
//    std::lock_guard<std::recursive_mutex> lock(sync);

//    foreach(Output* out, outputs_) {
//        if(!out->canSendMessages()) {
//            return false;
//        }
//    }

//    //    foreach(Output* out, parameter_outputs_) {
//    //        if(!out->canSendMessages()) {
//    //            return false;
//    //        }
//    //    }

//    return true;
//}

void NodeWorker::notifyMessagesProcessed()
{
    std::lock_guard<std::recursive_mutex> lock(sync);
    //    node_->aerr << "notifyMessagesProcessed" << std::endl;

    apex_assert_hard(state_ == State::WAITING_FOR_OUTPUTS);
    apex_assert_hard(transition_out_->canSendMessages());
    apex_assert_hard(transition_out_->isSink() || transition_out_->areOutputsIdle());

    setState(State::WAITING_FOR_RESET);

    Q_EMIT messagesProcessed();
}

void NodeWorker::prepareForNextProcess()
{
    apex_assert_hard(thread() == QThread::currentThread());

    {
        std::lock_guard<std::recursive_mutex> lock(sync);

        //        node_->aerr << "prepareForNextProcess" << std::endl;

        apex_assert_hard(thread() == QThread::currentThread());
        apex_assert_hard(state_ == State::WAITING_FOR_RESET);
        apex_assert_hard(transition_out_->isSink() || transition_out_->areOutputsIdle());
        apex_assert_hard(transition_out_->canSendMessages());

        static int iter = 0;
        for(Input* cin : inputs_) {
            for(ConnectionWeakPtr cw : cin->getConnections()) {
                apex_assert_hard(cw.lock()->getState() != Connection::State::UNREAD);
                apex_assert_hard(cw.lock()->getState() != Connection::State::DONE);
                apex_assert_hard(cw.lock()->getState() != Connection::State::READY_TO_RECEIVE);
                apex_assert_hard(cw.lock()->getState() == Connection::State::READ);
            }
        }
        ++iter;

        setState(State::IDLE);

        for(Input* cin : inputs_) {
            cin->free();
        }
        for(Input* cin : inputs_) {
            //            node_->aerr << "notify => " << cin->getUUID() << std::endl;
            cin->notifyMessageProcessed();
        }

        transition_in_->notifyMessageProcessed();
    }

    //    checkInputs();
}

void NodeWorker::checkTransitions()
{
    apex_assert_hard(thread() == QThread::currentThread());

    {
        std::lock_guard<std::recursive_mutex> lock(sync);
        if(state_ != State::IDLE && state_ != State::ENABLED) {
            return;
        }
    }


    if(transition_in_->hasUnestablishedConnection()) {
        transition_in_->establish();
    }
    if(transition_out_->hasUnestablishedConnection()) {
        transition_out_->establish();
    }
    if(transition_in_->hasUnestablishedConnection() || transition_out_->hasUnestablishedConnection()) {
//        setState(State::IDLE);
        return;
    }

    if(!transition_out_->canSendMessages()) {
        return;
    }

    setState(State::ENABLED);

    apex_assert_hard(transition_out_->canSendMessages());

    transition_in_->fireIfPossible();
}



void NodeWorker::publishParameters()
{
    for(std::size_t i = 0; i < parameter_outputs_.size(); ++i) {
        Output* out = parameter_outputs_[i];
        publishParameter(output_2_param_.at(out));
    }
}
void NodeWorker::publishParameter(param::Parameter* p)
{
    Output* out = param_2_output_.at(p->name());
    if(out->isConnected()) {
        while(!out->canSendMessages()) {
            node_->awarn << "waiting for parameter publish: " << p->name() << ", output " << out->getUUID() << " cannot send!" << std::endl;
            std::chrono::milliseconds dura(100);
            std::this_thread::sleep_for(dura);
        }

        if(p->is<int>())
            msg::publish(out, p->as<int>());
        else if(p->is<double>())
            msg::publish(out, p->as<double>());
        else if(p->is<std::string>())
            msg::publish(out, p->as<std::string>());
        out->commitMessages();
    }
}

void NodeWorker::sendMessages()
{
    assertNotInGuiThread();
    std::lock_guard<std::recursive_mutex> lock(sync);

    apex_assert_hard(getState() == State::PROCESSING);
    setState(State::WAITING_FOR_OUTPUTS);

    if(isSink()) {
        notifyMessagesProcessed();
        return;
    }


    //    node_->aerr << "SEND" << std::endl;
    apex_assert_hard(transition_out_->canSendMessages());
    transition_out_->sendMessages();

    Q_EMIT messagesWaitingToBeSent(false);
}

void NodeWorker::setTickFrequency(double f)
{
    if(tick_timer_->isActive()) {
        tick_timer_->stop();
    }
    if(f == 0.0) {
        return;
    }

    tick_immediate_ = (f < 0.0);

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
    return sink_ || outputs_.empty() || transition_out_->isSink();
}

int NodeWorker::getLevel() const
{
    return level_;
}

void NodeWorker::setLevel(int level)
{
    level_ = level;
    for(Input* in : inputs_) {
        in->setLevel(level);
    }
    for(Output* out : outputs_) {
        out->setLevel(level);
    }
}

void NodeWorker::tick()
{
    if(!is_setup_) {
        return;
    }

    assertNotInGuiThread();


    {
        std::unique_lock<std::recursive_mutex> pause_lock(pause_mutex_);
        while(paused_) {
            continue_.wait(pause_lock);
        }
    }

    std::unique_lock<std::recursive_mutex> lock(stop_mutex_);
    if(stop_) {
        return;
    }


    {
        //        Timer::Ptr t = nullptr;
        //        if(profiling_) {
        //            t.reset(new Timer(getUUID()));
        //            Q_EMIT timerStarted(this, OTHER, t->startTimeMs());
        //        }
        //        node_->useTimer(t.get());

        node_->checkConditions(false);
        checkParameters();

        //        if(t) {
        //            finishTimer(t);
        //        }
    }

    if(!thread_initialized_) {
        thread::set_name(thread()->objectName().toStdString().c_str());
        thread_initialized_ = true;
    }

    if(isEnabled()) {
        std::lock_guard<std::recursive_mutex> lock(sync);
        auto state = getState();
        if(state == State::IDLE || state == State::ENABLED) {

            static int ticks = 0;
            if(isTickEnabled() && isSource() && node_->canTick() /*&& ticks++ == 0*/) {
//                std::cerr << "should tick, state is " << (int)getState() << ", can send messages is " << transition_out_->canSendMessages() << std::endl;
                if(transition_out_->canSendMessages()) {
                    //            std::cerr << "ticks" << std::endl;
                    apex_assert_hard(state == State::IDLE || state == State::ENABLED);
                    if(state == State::IDLE) {
                        setState(State::ENABLED);
                    }
                    setState(State::FIRED);
                    setState(State::PROCESSING);

                    transition_out_->clearOutputs();

                    TimerPtr t = nullptr;
                    if(profiling_) {
                        t.reset(new Timer(getUUID()));
                        Q_EMIT timerStarted(this, TICK, t->startTimeMs());
                    }
                    node_->useTimer(t.get());
                    node_->tick();

                    if(trigger_tick_done_->isConnected()) {
                        if(t) {
                            t->step("trigger tick done");
                        }
                        trigger_tick_done_->trigger();
                    }

                    Q_EMIT ticked();

                    ++ticks_;

                    bool has_msg = false;
                    foreach(Output* out, outputs_) {
                        has_msg |= msg::hasMessage(out);
                    }

                    if(has_msg) {
                        apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
                        transition_out_->setConnectionsReadyToReceive();

                        Q_EMIT messagesWaitingToBeSent(true);

                        apex_assert_hard(transition_out_->canSendMessages());
                        sendMessages();
                    } else {
                        setState(state);
                    }

                    if(t) {
                        finishTimer(t);
                    }
                }
            }
        }
    }

    if(tick_immediate_) {
        Q_EMIT tickRequested();
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
                std::cerr << "parameter callback failed: " << e.what() << std::endl;
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
        enableSlots(true);
        enableTriggers(true);

        triggerCheckTransitions();

    } else {
        enableInput(false);
        enableOutput(false);
        enableSlots(false);
        enableTriggers(false);
    }
}


void NodeWorker::enableIO(bool enable)
{
    enableInput(canReceive() && enable);
    enableOutput(enable);
    enableSlots(enable);
    enableTriggers(enable);
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

void NodeWorker::enableSlots (bool enable)
{
    Q_FOREACH(Slot* i, slots_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}

void NodeWorker::enableTriggers (bool enable)
{
    Q_FOREACH(Trigger* i, triggers_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}

void NodeWorker::triggerError(bool e, const std::string &what)
{
    setError(e, what);
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
    node_state_->setMinimized(min);
}

void NodeWorker::assertNotInGuiThread()
{
    assert(this->thread() != QApplication::instance()->thread());
}



void NodeWorker::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    node_->aerr << msg << std::endl;

    if(error && level == ErrorState::ErrorLevel::ERROR) {
        setIOError(true);
    } else {
        setIOError(false);
    }
}

/// MOC
#include "../../include/csapex/model/moc_node_worker.cpp"
