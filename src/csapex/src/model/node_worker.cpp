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
#include <csapex/model/node_state.h>
#include <csapex/utility/q_signal_relay.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <utils_param/trigger_parameter.h>
#include <csapex/model/node_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <QThread>
#include <QApplication>
#include <thread>
#include <iostream>

using namespace csapex;

const double NodeWorker::DEFAULT_FREQUENCY = 30.0;

NodeWorker::NodeWorker(const std::string& type, const UUID& uuid, Node::Ptr node)
    : Unique(uuid),
      node_type_(type), node_(node), node_state_(std::make_shared<NodeState>(this)),
      transition_in_(std::make_shared<InputTransition>(this)),
      transition_out_(std::make_shared<OutputTransition>(this)),
      is_setup_(false), state_(State::IDLE),
      trigger_tick_done_(nullptr), trigger_process_done_(nullptr),
      tick_enabled_(false), ticks_(0),
      source_(false), sink_(false), level_(0),
      profiling_(false)
{
    apex_assert_hard(node_);

    tick_timer_ = new QTimer();
    setTickFrequency(DEFAULT_FREQUENCY);

    node_state_->setLabel(uuid);

    modifier_ = std::make_shared<NodeModifier>(this);
    node_->initialize(uuid, modifier_.get());

    bool params_created_in_constructor = node_->getParameterCount() != 0;

    if(params_created_in_constructor) {
        makeParametersConnectable();
    }

    node_->getParameterState()->parameter_added->connect(std::bind(&NodeWorker::makeParameterConnectable, this, std::placeholders::_1));
    node_->getParameterState()->parameter_removed->connect(std::bind(&NodeWorker::makeParameterNotConnectable, this, std::placeholders::_1));

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

    QObject::connect(tick_timer_, &QTimer::timeout, [this]() {
        tickRequested();
    });
    //    tickRequested();
}

NodeWorker::~NodeWorker()
{
    tick_immediate_ = false;
    is_setup_ = false;

    //    waitUntilFinished();

    tick_timer_->stop();

    QObject::disconnect(this);

    while(!inputs_.empty()) {
        removeInput(inputs_.begin()->get());
    }
    while(!outputs_.empty()) {
        removeOutput(outputs_.begin()->get());
    }
    while(!parameter_inputs_.empty()) {
        removeInput(parameter_inputs_.begin()->get());
    }
    while(!parameter_outputs_.empty()) {
        removeOutput(parameter_outputs_.begin()->get());
    }
    while(!slots_.empty()) {
        removeSlot(slots_.begin()->get());
    }
    while(!triggers_.empty()) {
        removeTrigger(triggers_.begin()->get());
    }

    for(QObject* cb : callbacks) {
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

    node_state_->label_changed->connect([this]() {
        std::string label = node_state_->getLabel();
        if(label.empty()) {
            label = getUUID().getShortName();
        }

        node_->adebug.setPrefix(label);
        node_->ainfo.setPrefix(label);
        node_->awarn.setPrefix(label);
        node_->aerr.setPrefix(label);
    });

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
    nodeStateChanged();
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

NodeWeakPtr NodeWorker::getNode() const
{
    return node_;
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
        apex_assert_hard(state_ == State::PROCESSING || state_ == State::ENABLED);
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
    enabled(e);
}

bool NodeWorker::isWaitingForTrigger() const
{
    for(TriggerPtr t : triggers_) {
        if(t->isBeingProcessed()) {
            return true;
        }
    }
    return false;
}

bool NodeWorker::canProcess()
{
    return canReceive() && canSend() && !isWaitingForTrigger();
}

bool NodeWorker::canReceive()
{
    for(InputPtr i : inputs_) {
        if(!i->isConnected() && !i->isOptional()) {
            return false;
        }
    }
    return true;
}

bool NodeWorker::canSend()
{
    return transition_out_->canStartSendingMessages();
}

template <typename T>
void NodeWorker::makeParameterConnectableImpl(param::ParameterPtr param)
{
    param::Parameter* p = param.get();

    if(param_2_input_.find(p->name()) != param_2_input_.end()) {
        return;
    }

    {
        InputPtr cin = std::make_shared<Input>(transition_in_.get(), UUID::make_sub(getUUID(), std::string("in_") + p->name()));
        cin->setEnabled(true);
        cin->setOptional(true);
        cin->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());

        parameter_inputs_.push_back(cin);
        connectConnector(cin.get());
        //        cin->moveToThread(thread());

        param_2_input_[p->name()] = cin;
        input_2_param_[cin.get()] = p;
    }
    {
        OutputPtr cout = std::make_shared<StaticOutput>(transition_out_.get(), UUID::make_sub(getUUID(), std::string("out_") + p->name()));
        cout->setEnabled(true);
        cout->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());

        if(getState() == State::PROCESSING) {
            cout->clear();
        }

        parameter_outputs_.push_back(cout);
        connectConnector(cout.get());
        //        cout->moveToThread(thread());

        auto cb = std::bind(&NodeWorker::publishParameter, this, p);
        auto c = p->parameter_changed.connect(cb);
        param_connections_.insert(std::make_pair(p, c));

        param_2_output_[p->name()] = cout;
        output_2_param_[cout.get()] = p;
    }
}

void NodeWorker::makeParameterConnectable(param::ParameterPtr p)
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

    param::TriggerParameterPtr t = std::dynamic_pointer_cast<param::TriggerParameter>(p);
    if(t) {
        Trigger* trigger = addTrigger(t->name());
        addSlot(t->name(), std::bind(&param::TriggerParameter::trigger, t), false);
        node_->addParameterCallback(t.get(), std::bind(&Trigger::trigger, trigger));
    }
}

void NodeWorker::makeParameterNotConnectable(param::ParameterPtr p)
{
    InputPtr cin = param_2_input_[p->name()];
    OutputPtr cout = param_2_output_[p->name()];

    disconnectConnector(cin.get());
    disconnectConnector(cout.get());

    parameter_inputs_.erase(std::find(parameter_inputs_.begin(), parameter_inputs_.end(), cin));
    parameter_outputs_.erase(std::find(parameter_outputs_.begin(), parameter_outputs_.end(), cout));

    param_2_input_.erase(p->name());
    input_2_param_.erase(cin.get());

    param_2_output_.erase(p->name());
    output_2_param_.erase(cout.get());

    auto it = param_connections_.find(p.get());
    it->second.disconnect();
    param_connections_.erase(it);
}

void NodeWorker::makeParametersConnectable()
{
    GenericState::Ptr state = node_->getParameterState();
    for(std::map<std::string, param::Parameter::Ptr>::const_iterator it = state->params.begin(), end = state->params.end(); it != end; ++it ) {
        makeParameterConnectable(it->second);
    }
}

void NodeWorker::triggerPanic()
{
    panic();
}

void NodeWorker::triggerProcess()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    apex_assert_hard(state_ == State::ENABLED);

    apex_assert_hard(transition_out_->canStartSendingMessages());
    setState(State::FIRED);
    processRequested();
}

void NodeWorker::triggerCheckTransitions()
{
    checkTransitionsRequested();
}

void NodeWorker::connectConnector(Connectable *c)
{
    c->connectionInProgress.connect(connectionInProgress);
    c->connectionStart.connect(connectionStart);
    c->connectionDone.connect(connectionDone);
    c->connectionDone.connect([this](Connectable*) { checkIO(); });
    c->connectionEnabled.connect([this](bool) { checkIO(); });
    c->connectionRemoved.connect([this](Connectable*) { checkIO(); });

    c->enabled_changed.connect(std::bind(&NodeWorker::checkIO, this));
}


void NodeWorker::disconnectConnector(Connectable*)
{
    //    disconnect(c);
}


void NodeWorker::stop()
{
    QObject::disconnect(tick_timer_);

    assertNotInGuiThread();
    node_->abort();


    for(OutputPtr i : outputs_) {
        i->stop();
    }
    for(InputPtr i : inputs_) {
        i->stop();
    }

    for(InputPtr i : inputs_) {
        disconnectConnector(i.get());
    }
    for(OutputPtr i : outputs_) {
        disconnectConnector(i.get());
    }

    QObject::disconnect(this);
}

void NodeWorker::reset()
{
    assertNotInGuiThread();

    node_->abort();
    setError(false);

    // set state without checking!
    state_ = State::IDLE;

    transition_in_->reset();
    transition_out_->reset();

    updateTransitionConnections();
}

void NodeWorker::setProfiling(bool profiling)
{
    profiling_ = profiling;

    {
        std::unique_lock<std::recursive_mutex> lock(timer_mutex_);
        timer_history_.clear();
    }

    if(profiling_) {
        startProfiling(this);
    } else {
        stopProfiling(this);
    }
}

bool NodeWorker::isProfiling() const
{
    return profiling_;
}

void NodeWorker::killExecution()
{
    // TODO: implement
}

void NodeWorker::updateParameterValue(Connectable *s)
{
    assertNotInGuiThread();

    apex_assert_hard(getState() == State::PROCESSING);
    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);

    param::Parameter* p = input_2_param_.at(source);

    if(msg::isValue<int>(source)) {
        p->set<int>(msg::getValue<int>(source));
    } else if(msg::isValue<double>(source)) {
        p->set<double>(msg::getValue<double>(source));
    } else if(msg::isValue<std::string>(source)) {
        p->set<std::string>(msg::getValue<std::string>(source));
    } else if(msg::isValue<std::pair<int,int>>(source)) {
        p->set<std::pair<int, int>>(msg::getValue<std::pair<int,int>>(source));
    } else if(msg::isValue<std::pair<double,double>>(source)) {
        p->set<std::pair<double, double>>(msg::getValue<std::pair<double,double>>(source));
    } else if(msg::hasMessage(source) && !msg::isMessage<connection_types::NoMessage>(source)) {
        node_->ainfo << "parameter " << p->name() << " got a message of unsupported type" << std::endl;
    }
}

void NodeWorker::startProcessingMessages()
{
    if(!isEnabled()) {
        return;
    }

    std::unique_lock<std::recursive_mutex> lock(sync);

    assertNotInGuiThread();

    apex_assert_hard(state_ == State::FIRED);
    apex_assert_hard(!transition_in_->hasUnestablishedConnection());
    apex_assert_hard(!transition_out_->hasUnestablishedConnection());
    apex_assert_hard(transition_out_->canStartSendingMessages());
    apex_assert_hard(isEnabled());
    apex_assert_hard(canProcess());
    setState(State::PROCESSING);

    // everything has a message here

    {
        bool has_multipart = false;
        bool multipart_are_done = true;

        for(auto input : getAllInputs()) {
            for(auto& c : input->getConnections()) {
                int f = c->getMessage()->flags.data;
                if(f & (int) ConnectionType::Flags::Fields::MULTI_PART) {
                    has_multipart = true;

                    bool last_part = f & (int) ConnectionType::Flags::Fields::LAST_PART;
                    multipart_are_done &= last_part;
                }
            }
        }

        if(has_multipart) {
            for(auto output : getAllOutputs()) {
                bool is_last = multipart_are_done;
                output->setMultipart(true, is_last);
            }
        }
    }


    bool all_inputs_are_present = true;

    {
        std::unique_lock<std::recursive_mutex> lock(sync);

        // check if one is "NoMessage";
        for(InputPtr cin : inputs_) {
            apex_assert_hard(cin->hasReceived() || (cin->isOptional() && !cin->isConnected()));
            if(!cin->isOptional() && !msg::hasMessage(cin.get())) {
                all_inputs_are_present = false;
            }
        }

        // update parameters
        for(InputPtr cin : parameter_inputs_) {
            apex_assert_hard(cin->isOptional());
            if(msg::hasMessage(cin.get())) {
                updateParameterValue(cin.get());
            }
        }
    }

    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
    transition_out_->setConnectionsReadyToReceive();
    transition_out_->clearOutputs();

    transition_in_->notifyMessageProcessed();

    if(!all_inputs_are_present) {
        finishProcessingMessages(false);
        return;
    }


    std::unique_lock<std::recursive_mutex> sync_lock(sync);

    current_process_timer_.reset();

    if(profiling_) {
        current_process_timer_.reset(new Timer(getUUID()));
        timerStarted(this, PROCESS, current_process_timer_->startTimeMs());
    }
    node_->useTimer(current_process_timer_.get());

    bool sync = !node_->isAsynchronous();

    try {
        //            node_->aerr << "processing" << std::endl;
        if(sync) {
            node_->process(*node_);

        } else {
            node_->process(*node_, [this](std::function<void()> f) {
                executionRequested([this, f]() {
                    f();
                    finishProcessingMessages(true);
                });
            });
        }



    } catch(const std::exception& e) {
        setError(true, e.what());
    } catch(...) {
        throw;
    }
    if(sync) {
        finishProcessingMessages(true);
    }
}

void NodeWorker::finishProcessingMessages(bool was_executed)
{
    if(was_executed) {
        if(current_process_timer_) {
            finishTimer(current_process_timer_);
        }

        if(trigger_process_done_->isConnected()) {
            if(current_process_timer_) {
                current_process_timer_->step("trigger process done");
            }
            trigger_process_done_->trigger();
        }
    }

    if(getState() == State::PROCESSING) {
        sendMessages();

        setState(State::IDLE);

        messages_processed();
    }
}

bool NodeWorker::areAllInputsAvailable()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    // check if all inputs have messages
    for(InputPtr cin : inputs_) {
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
        std::unique_lock<std::recursive_mutex> lock(timer_mutex_);
        timer_history_.push_back(t);
    }
    timerStopped(this, t->stopTimeMs());
}

std::vector<TimerPtr> NodeWorker::extractLatestTimers()
{
    std::unique_lock<std::recursive_mutex> lock(timer_mutex_);

    std::vector<TimerPtr> result = timer_history_;
    timer_history_.clear();
    return result;
}

Input* NodeWorker::addInput(ConnectionTypePtr type, const std::string& label, bool dynamic, bool optional)
{
    int id = inputs_.size();
    InputPtr c;
    if(dynamic) {
        c = std::make_shared<DynamicInput>(transition_in_.get(), this, id);
    } else {
        c = std::make_shared<Input>(transition_in_.get(), this, id);
    }
    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);

    registerInput(c);

    return c.get();
}

Output* NodeWorker::addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    int id = outputs_.size();
    OutputPtr c;
    if(dynamic) {
        c = std::make_shared<DynamicOutput>(transition_out_.get(), this, id);
    } else {
        c = std::make_shared<StaticOutput>(transition_out_.get(), this, id);
    }
    c->setLabel(label);
    c->setType(type);

    registerOutput(c);
    return c.get();
}

Slot* NodeWorker::addSlot(const std::string& label, std::function<void()> callback, bool active)
{
    int id = slots_.size();
    SlotPtr slot = std::make_shared<Slot>(callback, this, id, active);
    slot->setLabel(label);
    slot->setEnabled(true);

    registerSlot(slot);

    return slot.get();
}

Trigger* NodeWorker::addTrigger(const std::string& label)
{
    int id = triggers_.size();
    TriggerPtr trigger = std::make_shared<Trigger>(this, id);
    trigger->setLabel(label);
    trigger->setEnabled(true);

    registerTrigger(trigger);

    return trigger.get();
}

InputPtr NodeWorker::getParameterInput(const std::string &name) const
{
    std::map<std::string, InputPtr>::const_iterator it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return nullptr;
    } else {
        return it->second;
    }
}

OutputPtr NodeWorker::getParameterOutput(const std::string &name) const
{
    std::map<std::string, OutputPtr>::const_iterator it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return nullptr;
    } else {
        return it->second;
    }
}


void NodeWorker::removeInput(Input* in)
{
    std::vector<InputPtr>::iterator it;
    for(it = inputs_.begin(); it != inputs_.end(); ++it) {
        if(it->get() == in) {
            break;
        }
    }

    InputPtr input;

    if(it != inputs_.end()) {
        input = *it;
        inputs_.erase(it);
    } else {
        std::vector<InputPtr>::iterator it;
        for(it = parameter_inputs_.begin(); it != parameter_inputs_.end(); ++it) {
            if(it->get() == in) {
                break;
            }
        }
        if(it != parameter_inputs_.end()) {
            input = *it;
            parameter_inputs_.erase(it);
        } else {
            std::cerr << "ERROR: cannot remove input " << in->getUUID().getFullName() << std::endl;
            return;
        }
    }

    if(input) {
        disconnectConnector(input.get());
        connectorRemoved(input);
    }
}

void NodeWorker::removeOutput(Output* out)
{
    std::vector<OutputPtr>::iterator it;
    for(it = outputs_.begin(); it != outputs_.end(); ++it) {
        if(it->get() == out) {
            break;
        }
    }

    OutputPtr output;

    if(it != outputs_.end()) {
        output = *it;
        outputs_.erase(it);
    } else {
        std::vector<OutputPtr>::iterator it;
        for(it = parameter_outputs_.begin(); it != parameter_outputs_.end(); ++it) {
            if(it->get() == out) {
                break;
            }
        }
        if(it != parameter_outputs_.end()) {
            output = *it;
            parameter_outputs_.erase(it);
        } else {
            std::cerr << "ERROR: cannot remove output " << out->getUUID().getFullName() << std::endl;
            return;
        }
    }

    if(output) {
        disconnectConnector(output.get());
        connectorRemoved(output);
    }
}

void NodeWorker::removeSlot(Slot* s)
{
    std::vector<SlotPtr>::iterator it;
    for(it = slots_.begin(); it != slots_.end(); ++it) {
        if(it->get() == s) {
            break;
        }
    }

    auto cb_it = slot_connections_.find(s);
    cb_it->second.disconnect();
    slot_connections_.erase(cb_it);


    if(it != slots_.end()) {
        SlotPtr slot = *it;

        slots_.erase(it);

        disconnectConnector(slot.get());
        connectorRemoved(slot);
    }

}

void NodeWorker::removeTrigger(Trigger* t)
{
    std::vector<TriggerPtr>::iterator it;
    for(it = triggers_.begin(); it != triggers_.end(); ++it) {
        if(it->get() == t) {
            break;
        }
    }

    auto pos_t = trigger_triggered_connections_.find(t);
    pos_t->second.disconnect();
    trigger_triggered_connections_.erase(pos_t);

    auto pos_h = trigger_handled_connections_.find(t);
    pos_h->second.disconnect();
    trigger_handled_connections_.erase(pos_h);

    if(it != triggers_.end()) {
        TriggerPtr trigger = *it;

        triggers_.erase(it);

        disconnectConnector(trigger.get());
        connectorRemoved(trigger);
    }
}

void NodeWorker::registerInput(InputPtr in)
{
    inputs_.push_back(in);

    connectConnector(in.get());
    //    QObject::connect(in, SIGNAL(connectionDone(Connectable*)), this, SLOT(trySendMessages()));

    connectorCreated(in);
}

void NodeWorker::registerOutput(OutputPtr out)
{
    outputs_.push_back(out);

    connectConnector(out.get());

    out->messageProcessed.connect([this](Connectable*) { triggerCheckTransitions(); });
    out->connectionRemoved.connect([this](Connectable*) { triggerCheckTransitions(); });
    out->connectionDone.connect([this](Connectable*) { triggerCheckTransitions(); });
    out->connectionEnabled.connect([this](bool) { triggerCheckTransitions(); });

    connectorCreated(out);
}

void NodeWorker::registerSlot(SlotPtr s)
{
    slots_.push_back(s);

    connectConnector(s.get());

    Slot* slot = s.get();

    auto connection = s->triggered.connect([this, slot](Trigger* source) {
        executionRequested([this, slot, source]() {
            slot->handleTrigger();
            /// PROBLEM: Relaying signals not yet possible here
            ///  if slot triggeres a signal itself...
            /// SOLUTION: boost signal?
            source->signalHandled(slot);
        });
    });
    slot_connections_.insert(std::make_pair(slot, connection));

    connectorCreated(s);
}

void NodeWorker::registerTrigger(TriggerPtr t)
{
    triggers_.push_back(t);

    //PROBLEM: wait for all m slots to be done...
    // in the mean time, allow NO TICK; NO PROCESS;
    // BUT ALSO EXECUTE OTHER TRIGGERS OF THE SAME PROCESS / TICK...
    // solution: "lock" the nodeworker, "unlock" it in signalHandled, once all
    //  TRIGGERs are done (not only this one!!!!!)

    auto connection_triggered = t->triggered.connect([this](){
        triggerCheckTransitions();
    });
    trigger_triggered_connections_.insert(std::make_pair(t.get(), connection_triggered));

    auto connection_handled = t->all_signals_handled.connect([this](){
        triggerCheckTransitions();
    });
    trigger_handled_connections_.insert(std::make_pair(t.get(), connection_handled));

    connectConnector(t.get());

    connectorCreated(t);
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
    for(InputPtr in : inputs_) {
        if(in->getUUID() == uuid) {
            return in.get();
        }
    }
    for(InputPtr in : parameter_inputs_) {
        if(in->getUUID() == uuid) {
            return in.get();
        }
    }

    return nullptr;
}

Output* NodeWorker::getOutput(const UUID& uuid) const
{
    for(OutputPtr out : outputs_) {
        if(out->getUUID() == uuid) {
            return out.get();
        }
    }
    for(OutputPtr out : parameter_outputs_) {
        if(out->getUUID() == uuid) {
            return out.get();
        }
    }

    return nullptr;
}


Slot* NodeWorker::getSlot(const UUID& uuid) const
{
    for(SlotPtr s : slots_) {
        if(s->getUUID() == uuid) {
            return s.get();
        }
    }

    return nullptr;
}


Trigger* NodeWorker::getTrigger(const UUID& uuid) const
{
    for(TriggerPtr t : triggers_) {
        if(t->getUUID() == uuid) {
            return t.get();
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

std::vector<ConnectablePtr> NodeWorker::getAllConnectors() const
{
    std::size_t n = inputs_.size() + parameter_inputs_.size();
    n += outputs_.size() + parameter_outputs_.size();
    n += triggers_.size() + slots_.size();
    std::vector<ConnectablePtr> result(n, nullptr);
    std::size_t pos = 0;
    for(auto i : inputs_) {
        result[pos++] = i;
    }
    for(auto i : parameter_inputs_) {
        result[pos++] = i;
    }
    for(auto i : outputs_) {
        result[pos++] = i;
    }
    for(auto i : parameter_outputs_) {
        result[pos++] = i;
    }
    for(auto i : triggers_) {
        result[pos++] = i;
    }
    for(auto i : slots_) {
        result[pos++] = i;
    }
    return result;
}

std::vector<InputPtr> NodeWorker::getAllInputs() const
{
    std::size_t n = inputs_.size() + parameter_inputs_.size();
    std::vector<InputPtr> result(n, nullptr);
    std::size_t pos = 0;
    for(auto i : inputs_) {
        result[pos++] = i;
    }
    for(auto i : parameter_inputs_) {
        result[pos++] = i;
    }
    return result;
}

std::vector<OutputPtr> NodeWorker::getAllOutputs() const
{
    std::size_t n = outputs_.size() + parameter_outputs_.size();
    std::vector<OutputPtr> result(n, nullptr);
    std::size_t pos = 0;
    for(auto i : outputs_) {
        result[pos++] = i;
    }
    for(auto i : parameter_outputs_) {
        result[pos++] = i;
    }
    return result;
}

std::vector<InputPtr> NodeWorker::getMessageInputs() const
{
    return inputs_;
}

std::vector<OutputPtr> NodeWorker::getMessageOutputs() const
{
    return outputs_;
}

std::vector<SlotPtr> NodeWorker::getSlots() const
{
    return slots_;
}

std::vector<TriggerPtr> NodeWorker::getTriggers() const
{
    return triggers_;
}

std::vector<InputPtr> NodeWorker::getParameterInputs() const
{
    return parameter_inputs_;
}

std::vector<OutputPtr> NodeWorker::getParameterOutputs() const
{
    return parameter_outputs_;
}

void NodeWorker::notifyMessagesProcessed()
{
    {
        std::unique_lock<std::recursive_mutex> lock(state_mutex_);
        //    node_->aerr << "notifyMessagesProcessed" << std::endl;

        //        apex_assert_hard(state_ == State::WAITING_FOR_OUTPUTS);
        apex_assert_hard(transition_out_->isSink() || transition_out_->areOutputsIdle());
    }

    //    setState(State::WAITING_FOR_RESET);

    //    messages_processed();

    triggerCheckTransitions();
}

void NodeWorker::prepareForNextProcess()
{
    //    {
    //        std::unique_lock<std::recursive_mutex> lock(sync);

    //        //        node_->aerr << "prepareForNextProcess" << std::endl;

    //        apex_assert_hard(state_ == State::WAITING_FOR_RESET);
    //        apex_assert_hard(transition_out_->isSink() || transition_out_->areOutputsIdle());
    //        apex_assert_hard(transition_out_->canSendMessages());

    //        setState(State::IDLE);

    //        transition_in_->notifyMessageProcessed();
    //    }

    //    checkTransitions();
    //    checkInputs();
}

void NodeWorker::updateTransitionConnections()
{
    // TODO: can this be moved into transition?
    transition_in_->updateConnections();
    transition_out_->updateConnections();
}

void NodeWorker::checkTransitions()
{
    {
        std::unique_lock<std::recursive_mutex> lock(sync);

        //        if(state_ == State::WAITING_FOR_OUTPUTS) {
        //        transition_out_->updateOutputs();
        //        }

        if(state_ != State::IDLE && state_ != State::ENABLED) {
            return;
        }
    }
    updateTransitionConnections();

    if(transition_in_->hasUnestablishedConnection() || transition_out_->hasUnestablishedConnection()) {
        if(state_ == State::ENABLED) {
            setState(State::IDLE);
        }
        return;
    }

    if(!transition_out_->canStartSendingMessages()) {
        if(state_ == State::ENABLED) {
            setState(State::IDLE);
        }
        return;
    }

    setState(State::ENABLED);

    apex_assert_hard(transition_out_->canStartSendingMessages());

    transition_in_->fireIfPossible();
}



void NodeWorker::publishParameters()
{
    for(OutputPtr out : parameter_outputs_) {
        auto pos = output_2_param_.find(out.get());
        if(pos != output_2_param_.end()) {
            publishParameter(pos->second);
        }
    }
}
void NodeWorker::publishParameter(param::Parameter* p)
{
    if(param_2_output_.find(p->name()) != param_2_output_.end()) {
        OutputPtr out = param_2_output_.at(p->name());
        if(out->isConnected()) {
            if(p->is<int>())
                msg::publish(out.get(), p->as<int>());
            else if(p->is<double>())
                msg::publish(out.get(), p->as<double>());
            else if(p->is<std::string>())
                msg::publish(out.get(), p->as<std::string>());
            else if(p->is<std::pair<int, int>>())
                msg::publish(out.get(), p->as<std::pair<int, int>>());
            else if(p->is<std::pair<double, double>>())
                msg::publish(out.get(), p->as<std::pair<double, double>>());
        }
    }
}

void NodeWorker::sendMessages()
{
    assertNotInGuiThread();

    std::unique_lock<std::recursive_mutex> lock(sync);

    transition_out_->updateConnections();

    apex_assert_hard(getState() == State::PROCESSING);
    //    setState(State::WAITING_FOR_OUTPUTS);

    if(transition_out_->isSink()) {
        notifyMessagesProcessed();

        return;
    }

    int seq = 0;

    bool source = isSource();//inputs_.empty();

    if(source) {
        for(auto o : outputs_) {
            seq = std::max(seq, o->sequenceNumber());
        }
        for(auto o : parameter_outputs_) {
            seq = std::max(seq, o->sequenceNumber());
        }

    } else {
        for(auto i : inputs_) {
            seq = std::max(seq, i->sequenceNumber());
        }
        for(auto i : parameter_inputs_) {
            seq = std::max(seq, i->sequenceNumber());
        }
    }
    for(auto o : getAllOutputs()) {
        o->setSequenceNumber(seq);
    }

    publishParameters();

    transition_out_->sendMessages();

    if(source) {
        for(auto o : getAllOutputs()) {
            o->setSequenceNumber(seq+1);
        }
    }

    //    node_->aerr << "SEND" << std::endl;
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
        for(InputPtr in : inputs_) {
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
    for(InputPtr in : inputs_) {
        in->setLevel(level);
    }
    for(OutputPtr out : outputs_) {
        out->setLevel(level);
    }
}

void NodeWorker::tick()
{
    if(!is_setup_) {
        return;
    }

    auto tickable = std::dynamic_pointer_cast<TickableNode>(node_);
    apex_assert_hard(tickable);

    assertNotInGuiThread();

    {
        if(!isEnabled()) {
            return;
        }
    }

    {
        //        Timer::Ptr t = nullptr;
        //        if(profiling_) {
        //            t.reset(new Timer(getUUID()));
        //            timerStarted(this, OTHER, t->startTimeMs());
        //        }
        //        node_->useTimer(t.get());

        node_->checkConditions(false);
        checkParameters();

        //        if(t) {
        //            finishTimer(t);
        //        }
    }

    if(isEnabled()) {
        std::unique_lock<std::recursive_mutex> lock(sync);
        auto state = getState();
        if(state == State::IDLE || state == State::ENABLED) {
            if(!isWaitingForTrigger() && isTickEnabled() && isSource() && tickable->canTick()) {
                if(state == State::ENABLED) {
                    setState(State::IDLE);
                }
                checkTransitions();

                if(transition_out_->canStartSendingMessages() && !transition_out_->hasFadingConnection()) {
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
                        timerStarted(this, TICK, t->startTimeMs());
                    }
                    node_->useTimer(t.get());
                    tickable->tick();

                    if(trigger_tick_done_->isConnected()) {
                        if(t) {
                            t->step("trigger tick done");
                        }
                        trigger_tick_done_->trigger();
                    }

                    ticked();

                    ++ticks_;

                    bool has_msg = false;
                    for(OutputPtr out : outputs_) {
                        has_msg |= (msg::hasMessage(out.get()) && msg::isConnected(out.get()));
                    }
                    for(OutputPtr out : parameter_outputs_) {
                        has_msg |= (msg::hasMessage(out.get()) && msg::isConnected(out.get()));
                    }

                    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
                    if(has_msg) {
                        transition_out_->setConnectionsReadyToReceive();
                        sendMessages();

                    } else {
                        transition_out_->abortSendingMessages();
                    }

                    if(t) {
                        finishTimer(t);
                    }

                    setState(State::IDLE);
                }
            }
        }
    }

    if(tick_immediate_) {
        tickRequested();
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
    for(InputPtr i : inputs_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
    for(InputPtr i : parameter_inputs_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}


void NodeWorker::enableOutput (bool enable)
{
    for(OutputPtr o : outputs_) {
        if(enable) {
            o->enable();
        } else {
            o->disable();
        }
    }
    for(OutputPtr o : parameter_outputs_) {
        if(enable) {
            o->enable();
        } else {
            o->disable();
        }
    }
}

void NodeWorker::enableSlots (bool enable)
{
    for(SlotPtr i : slots_) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}

void NodeWorker::enableTriggers (bool enable)
{
    for(TriggerPtr i : triggers_) {
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

void NodeWorker::setIOError(bool /*error*/)
{
}

void NodeWorker::setMinimized(bool min)
{
    node_state_->setMinimized(min);
}

void NodeWorker::assertNotInGuiThread()
{
    assert(this->thread() != QApplication::instance()->thread());
}



void NodeWorker::errorEvent(bool error, const std::string& msg, ErrorLevel /*level*/)
{
    node_->aerr << msg << std::endl;

    errorHappened(error);

}
