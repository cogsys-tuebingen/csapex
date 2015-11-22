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
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/tickable_node.h>

/// SYSTEM
#include <thread>
#include <iostream>

using namespace csapex;

const double NodeWorker::DEFAULT_FREQUENCY = 30.0;

NodeWorker::NodeWorker(const std::string& type, const UUID& uuid, Node::Ptr node)
    : Unique(uuid),
      node_type_(type), node_(node), node_state_(std::make_shared<NodeState>(this)),
      transition_in_(std::make_shared<InputTransition>(std::bind(&NodeWorker::triggerCheckTransitions, this))),
      transition_out_(std::make_shared<OutputTransition>(std::bind(&NodeWorker::triggerCheckTransitions, this))),
      is_setup_(false), state_(State::IDLE),
      next_input_id_(0), next_output_id_(0), next_trigger_id_(0), next_slot_id_(0),
      trigger_tick_done_(nullptr), trigger_process_done_(nullptr),
      ticks_(0),
      source_(false), sink_(false), level_(0),
      profiling_(false)
{
    apex_assert_hard(node_);

    transition_out_->messages_processed.connect([this](){
        notifyMessagesProcessed();
    });

    node_state_->setLabel(uuid);

    modifier_ = std::make_shared<NodeModifier>(this);
    node_->initialize(uuid, modifier_.get());

    bool params_created_in_constructor = node_->getParameterCount() != 0;

    if(params_created_in_constructor) {
        makeParametersConnectable();
    }

    node_->parameters_changed.connect(parametersChanged);
    node_->getParameterState()->parameter_set_changed->connect(parametersChanged);
    node_->getParameterState()->parameter_added->connect(std::bind(&NodeWorker::makeParameterConnectable, this, std::placeholders::_1));
    node_->getParameterState()->parameter_removed->connect(std::bind(&NodeWorker::makeParameterNotConnectable, this, std::placeholders::_1));

    addSlot("enable", std::bind(&NodeWorker::setProcessingEnabled, this, true), true);
    addSlot("disable", std::bind(&NodeWorker::setProcessingEnabled, this, false), false);

    auto tickable = std::dynamic_pointer_cast<TickableNode>(node_);
    if(tickable) {
        trigger_tick_done_ = addTrigger("ticked");
    }
    trigger_process_done_ = addTrigger("inputs\nprocessed");

    node_->doSetup();

    is_setup_ = true;

    if(params_created_in_constructor) {
        node_->awarn << "Node creates parameters in its constructor! Please implement 'setupParameters'" << std::endl;
    }
}

NodeWorker::~NodeWorker()
{
    is_setup_ = false;

    //    waitUntilFinished();

    while(!inputs_.empty()) {
        removeInput(inputs_.begin()->get());
    }
    while(!outputs_.empty()) {
        removeOutput(outputs_.begin()->get());
    }
    while(!slots_.empty()) {
        removeSlot(slots_.begin()->get());
    }
    while(!triggers_.empty()) {
        removeTrigger(triggers_.begin()->get());
    }
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

bool NodeWorker::isEnabled() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return state_ == State::ENABLED;
}
bool NodeWorker::isIdle() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return state_ == State::IDLE;
}
bool NodeWorker::isProcessing() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return state_ == State::PROCESSING;
}
bool NodeWorker::isFired() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return state_ == State::FIRED;
}

void NodeWorker::setState(State state)
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    switch(state) {
    case State::IDLE:
        apex_assert_hard(state_ == State::PROCESSING || state_ == State::ENABLED || state_ == State::IDLE);
        break;
    case State::ENABLED:
        apex_assert_hard(state_ == State::IDLE || state_ == State::ENABLED);
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

bool NodeWorker::isProcessingEnabled() const
{
    return node_state_->isEnabled();
}

void NodeWorker::setProcessingEnabled(bool e)
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

bool NodeWorker::canProcess() const
{
    return canReceive() && canSend() && !isWaitingForTrigger();
}

bool NodeWorker::canReceive() const
{
    for(InputPtr i : inputs_) {
        if(!i->isConnected() && !i->isOptional()) {
            return false;
        }
    }
    return true;
}

bool NodeWorker::canSend() const
{
    return transition_out_->canStartSendingMessages();
}

template <typename T>
void NodeWorker::makeParameterConnectableImpl(csapex::param::ParameterPtr param)
{
    csapex::param::Parameter* p = param.get();

    if(param_2_input_.find(p->name()) != param_2_input_.end()) {
        return;
    }

    {
        InputPtr cin = std::make_shared<Input>(UUID::make_sub(getUUID(), std::string("in_") + p->name()));
        cin->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());
        cin->setOptional(true);
        cin->setLabel(p->name());

        addInput(cin);

        param_2_input_[p->name()] = cin;
        input_2_param_[cin.get()] = p;
    }
    {
        OutputPtr cout = std::make_shared<StaticOutput>(UUID::make_sub(getUUID(), std::string("out_") + p->name()));
        cout->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());
        cout->setLabel(p->name());

        if(getState() == State::PROCESSING) {
            cout->startReceiving();
        }

        addOutput(cout);

        param_2_output_[p->name()] = cout;
        output_2_param_[cout.get()] = p;
    }
}

void NodeWorker::makeParameterConnectable(csapex::param::ParameterPtr p)
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

void NodeWorker::makeParameterNotConnectable(csapex::param::ParameterPtr p)
{
    InputPtr cin_ptr = param_2_input_[p->name()].lock();
    OutputPtr cout_ptr = param_2_output_[p->name()].lock();

    Input* cin = cin_ptr.get();
    Output* cout = cout_ptr.get();

    disconnectConnector(cin);
    disconnectConnector(cout);

    removeInput(cin);
    removeOutput(cout);

    apex_assert_hard(param_2_input_.erase(p->name()) != 0);
    apex_assert_hard(input_2_param_.erase(cin) != 0);

    apex_assert_hard(param_2_output_.erase(p->name()) != 0);
    apex_assert_hard(output_2_param_.erase(cout) != 0);
}

void NodeWorker::makeParametersConnectable()
{
    GenericState::Ptr state = node_->getParameterState();
    for(std::map<std::string, csapex::param::Parameter::Ptr>::const_iterator it = state->params.begin(), end = state->params.end(); it != end; ++it ) {
        makeParameterConnectable(it->second);
    }
}

void NodeWorker::triggerPanic()
{
    panic();
}

void NodeWorker::triggerCheckTransitions()
{
    checkTransitionsRequested();
}

void NodeWorker::connectConnector(Connectable *c)
{
    c->connectionInProgress.connect(connectionInProgress);
    c->connectionStart.connect(connectionStart);
    c->connection_added_to.connect(connectionDone);
    c->connection_added_to.connect([this](Connectable*) { checkIO(); });
    c->connectionEnabled.connect([this](bool) { checkIO(); });
    c->connection_removed_to.connect([this](Connectable*) { checkIO(); });

    c->enabled_changed.connect(std::bind(&NodeWorker::checkIO, this));
}


void NodeWorker::disconnectConnector(Connectable*)
{
    //    disconnect(c);
}


void NodeWorker::stop()
{
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

namespace {
template <typename V>
void updateParameterValueFrom(csapex::param::Parameter* p, Input* source)
{
    auto current = p->as<V>();
    auto next = msg::getValue<V>(source);
    if(current != next) {
        p->set<V>(next);
    }
}
}

void NodeWorker::updateParameterValue(Connectable *s)
{
    assertNotInGuiThread();

    apex_assert_hard(getState() == State::PROCESSING);
    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);

    csapex::param::Parameter* p = input_2_param_.at(source);

    if(msg::isValue<int>(source)) {
        updateParameterValueFrom<int>(p, source);
    } else if(msg::isValue<double>(source)) {
        updateParameterValueFrom<double>(p, source);
    } else if(msg::isValue<std::string>(source)) {
        updateParameterValueFrom<std::string>(p, source);
    } else if(msg::isValue<std::pair<int,int>>(source)) {
        updateParameterValueFrom<std::pair<int, int>>(p, source);
    } else if(msg::isValue<std::pair<double,double>>(source)) {
        updateParameterValueFrom<std::pair<double, double>>(p, source);
    } else if(msg::hasMessage(source) && !msg::isMessage<connection_types::NoMessage>(source)) {
        node_->ainfo << "parameter " << p->name() << " got a message of unsupported type" << std::endl;
    }
}

void NodeWorker::startProcessingMessages()
{
    assertNotInGuiThread();

    std::unique_lock<std::recursive_mutex> lock(sync);
    apex_assert_hard(state_ == State::ENABLED);

    apex_assert_hard(transition_out_->canStartSendingMessages());
    setState(State::FIRED);

    if(!isProcessingEnabled()) {
        return;
    }


    apex_assert_hard(state_ == State::FIRED);
    apex_assert_hard(!transition_in_->hasUnestablishedConnection());
    apex_assert_hard(!transition_out_->hasUnestablishedConnection());
    apex_assert_hard(transition_out_->canStartSendingMessages());
    apex_assert_hard(isProcessingEnabled());
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
        for(auto pair : param_2_input_) {
            InputPtr cin = pair.second.lock();
            if(cin) {
                apex_assert_hard(cin->isOptional());
                if(msg::hasMessage(cin.get())) {
                    updateParameterValue(cin.get());
                }
            }
        }
    }

    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
    transition_out_->setConnectionsReadyToReceive();
    transition_out_->startReceiving();

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

bool NodeWorker::areAllInputsAvailable() const
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
    int id = next_input_id_++;
    InputPtr c;
    if(dynamic) {
        c = std::make_shared<DynamicInput>(this, id);
    } else {
        c = std::make_shared<Input>(this, id);
    }
    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);

    addInput(c);

    return c.get();
}

Output* NodeWorker::addOutput(ConnectionTypePtr type, const std::string& label, bool dynamic)
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    int id = next_output_id_++;
    OutputPtr c;
    if(dynamic) {
        c = std::make_shared<DynamicOutput>(this, id);
    } else {
        c = std::make_shared<StaticOutput>(this, id);
    }
    c->setLabel(label);
    c->setType(type);

    addOutput(c);
    return c.get();
}

Slot* NodeWorker::addSlot(const std::string& label, std::function<void()> callback, bool active)
{
    int id = next_slot_id_++;
    SlotPtr slot = std::make_shared<Slot>(callback, this, id, active);
    slot->setLabel(label);

    addSlot(slot);

    return slot.get();
}

Trigger* NodeWorker::addTrigger(const std::string& label)
{
    int id = next_trigger_id_++;
    TriggerPtr trigger = std::make_shared<Trigger>(this, id);
    trigger->setLabel(label);

    addTrigger(trigger);

    return trigger.get();
}

InputWeakPtr NodeWorker::getParameterInput(const std::string &name) const
{
    auto it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return std::weak_ptr<Input>();
    } else {
        return it->second;
    }
}

OutputWeakPtr NodeWorker::getParameterOutput(const std::string &name) const
{
    auto it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return std::weak_ptr<Output>();
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

    if(it != inputs_.end()) {
        InputPtr input = *it;
        transition_in_->removeInput(input);

        inputs_.erase(it);

        disconnectConnector(input.get());
        connectorRemoved(input);

    } else {
        std::cerr << "ERROR: cannot remove input " << in->getUUID().getFullName() << std::endl;
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

    if(it != outputs_.end()) {
        OutputPtr output = *it;
        transition_out_->removeOutput(output);

        outputs_.erase(it);

        disconnectConnector(output.get());
        connectorRemoved(output);

    } else {
        std::cerr << "ERROR: cannot remove output " << out->getUUID().getFullName() << std::endl;
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

void NodeWorker::addInput(InputPtr in)
{
    inputs_.push_back(in);

    connectConnector(in.get());
    //    QObject::connect(in, SIGNAL(connectionDone(Connectable*)), this, SLOT(trySendMessages()));

    connectorCreated(in);
    transition_in_->addInput(in);
}

void NodeWorker::addOutput(OutputPtr out)
{
    outputs_.push_back(out);

    connectConnector(out.get());

    out->messageProcessed.connect([this](Connectable*) { triggerCheckTransitions(); });
    out->connection_removed_to.connect([this](Connectable*) { triggerCheckTransitions(); });
    out->connection_added_to.connect([this](Connectable*) { triggerCheckTransitions(); });
    out->connectionEnabled.connect([this](bool) { triggerCheckTransitions(); });

    connectorCreated(out);
    transition_out_->addOutput(out);
}

bool NodeWorker::isParameterInput(Input *in) const
{
    return input_2_param_.find(in) != input_2_param_.end();
}

bool NodeWorker::isParameterOutput(Output *out) const
{
    return output_2_param_.find(out) != output_2_param_.end();
}

void NodeWorker::addSlot(SlotPtr s)
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

void NodeWorker::addTrigger(TriggerPtr t)
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

    return nullptr;
}

Output* NodeWorker::getOutput(const UUID& uuid) const
{
    for(OutputPtr out : outputs_) {
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
    std::size_t n = inputs_.size();
    n += outputs_.size();
    n += triggers_.size() + slots_.size();
    std::vector<ConnectablePtr> result(n, nullptr);
    std::size_t pos = 0;
    for(auto i : inputs_) {
        result[pos++] = i;
    }
    for(auto i : outputs_) {
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
    return inputs_;
}

std::vector<OutputPtr> NodeWorker::getAllOutputs() const
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

void NodeWorker::notifyMessagesProcessed()
{
    {
        std::unique_lock<std::recursive_mutex> lock(state_mutex_);
        apex_assert_hard(transition_out_->isSink() || transition_out_->areOutputsIdle());
    }

    triggerCheckTransitions();
}


void NodeWorker::updateTransitionConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(state_ == State::IDLE || state_ == State::ENABLED) {
        transition_in_->updateConnections();
        transition_out_->updateConnections();
    }
}

void NodeWorker::checkTransitions(bool try_fire)
{
    {
        std::unique_lock<std::recursive_mutex> lock(sync);

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

    if(!transition_out_->isEnabled()) {
        if(state_ == State::ENABLED) {
            setState(State::IDLE);
        }
        return;
    }

    if(transition_in_->isEnabled()) {
        setState(State::ENABLED);
    } else {
        setState(State::IDLE);
        return;
    }

    if(try_fire && canProcess()) {
        apex_assert_hard(transition_out_->canStartSendingMessages());

        int highest_deviant_seq = transition_in_->findHighestDeviantSequenceNumber();

        if(highest_deviant_seq >= 0) {
            transition_in_->notifyOlderConnections(highest_deviant_seq);
        } else {
            transition_in_->forwardMessages();

            startProcessingMessages();
        }
    }
}



void NodeWorker::publishParameters()
{
    for(auto pair : output_2_param_) {
        auto out = pair.first;
        auto p = pair.second;
        publishParameterOn(*p, out);
    }
}
void NodeWorker::publishParameterOn(const csapex::param::Parameter& p, Output* out)
{
    if(out->isConnected()) {
        if(p.is<int>())
            msg::publish(out, p.as<int>());
        else if(p.is<double>())
            msg::publish(out, p.as<double>());
        else if(p.is<std::string>())
            msg::publish(out, p.as<std::string>());
        else if(p.is<std::pair<int, int>>())
            msg::publish(out, p.as<std::pair<int, int>>());
        else if(p.is<std::pair<double, double>>())
            msg::publish(out, p.as<std::pair<double, double>>());
    }
}

void NodeWorker::publishParameter(csapex::param::Parameter* p)
{
    if(param_2_output_.find(p->name()) != param_2_output_.end()) {
        OutputPtr out = param_2_output_.at(p->name()).lock();
        if(out) {
            publishParameterOn(*p, out.get());
        }
    }
}

void NodeWorker::sendMessages()
{
    assertNotInGuiThread();

    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(getState() == State::PROCESSING);

    publishParameters();

    transition_out_->sendMessages();
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

bool NodeWorker::tick()
{
    bool has_ticked = false;

    if(!is_setup_) {
        return has_ticked;
    }
    auto tickable = std::dynamic_pointer_cast<TickableNode>(node_);
    apex_assert_hard(tickable);

    assertNotInGuiThread();

    {
        if(!isProcessingEnabled()) {
            return has_ticked;
        }
    }

    if(isProcessingEnabled()) {
        std::unique_lock<std::recursive_mutex> lock(sync);
        auto state = getState();
        if(state == State::IDLE || state == State::ENABLED) {
            if(!isWaitingForTrigger() && tickable->canTick()) {
                if(state == State::ENABLED) {
                    setState(State::IDLE);
                }

                checkTransitions(false);

                if(transition_out_->canStartSendingMessages() && !transition_out_->hasFadingConnection()) {
                    //            std::cerr << "ticks" << std::endl;

                    {
                        std::unique_lock<std::recursive_mutex> lock(state_mutex_);
                        auto state = getState();
                        apex_assert_hard(state == State::IDLE || state == State::ENABLED);
                        if(state == State::IDLE) {
                            setState(State::ENABLED);
                        }
                        setState(State::FIRED);
                        setState(State::PROCESSING);
                    }
                    transition_out_->startReceiving();

                    TimerPtr t = nullptr;
                    if(profiling_) {
                        t.reset(new Timer(getUUID()));
                        timerStarted(this, TICK, t->startTimeMs());
                    }
                    node_->useTimer(t.get());

                    tickable->tick();

                    has_ticked = true;

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
                        if(msg::isConnected(out.get())) {
                            if(isParameterOutput(out.get())) {
                                has_msg = true;
                                break;
                            }
                            if(msg::hasMessage(out.get())) {
                                has_msg = true;
                                break;
                            }
                        }
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

    return has_ticked;
}

void NodeWorker::checkParameters()
{
    node_->checkConditions(false);

    // check if a parameter was changed
    Parameterizable::ChangedParameterList changed_params = node_->getChangedParameters();
    if(!changed_params.empty()) {
        for(auto pair : changed_params) {
            try {
                if(pair.first->isEnabled()) {
                    pair.second(pair.first);
                }

            } catch(const std::exception& e) {
                std::cerr << "parameter callback failed: " << e.what() << std::endl;
            }
        }
    }
}

void NodeWorker::checkIO()
{
    if(isProcessingEnabled()) {
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
