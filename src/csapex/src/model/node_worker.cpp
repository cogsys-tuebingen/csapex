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
#include <csapex/model/node_handle.h>
#include <csapex/utility/delegate_bind.h>

/// SYSTEM
#include <thread>
#include <iostream>

using namespace csapex;

NodeWorker::NodeWorker(NodeHandlePtr node_handle)
    : node_handle_(node_handle),
      is_setup_(false), state_(State::IDLE),
      trigger_tick_done_(nullptr), trigger_process_done_(nullptr),
      ticks_(0),
      profiling_(false)
{
    modifier_ = std::make_shared<NodeModifier>(this, node_handle.get());
    NodePtr node = node_handle_->getNode().lock();

    node->initialize(node_handle->getUUID(), modifier_.get());

    apex_assert_hard(!node->isSetup());

    if(!node->isSetup()) {
        node->doSetup();
    }

    handle_connections_.emplace_back(node_handle_->getOutputTransition()->messages_processed.connect([this](){
        notifyMessagesProcessed();
    }));

    node_handle_->addSlot("enable", std::bind(&NodeWorker::setProcessingEnabled, this, true), true);
    node_handle_->addSlot("disable", std::bind(&NodeWorker::setProcessingEnabled, this, false), false);

    auto tickable = std::dynamic_pointer_cast<TickableNode>(node);
    if(tickable) {
        trigger_tick_done_ = node_handle_->addTrigger("ticked");
    }
    trigger_process_done_ = node_handle_->addTrigger("inputs\nprocessed");

    apex_assert_hard(node->isSetup());

    is_setup_ = true;

    handle_connections_.emplace_back(node_handle_->mightBeEnabled.connect([this]() {
        triggerCheckTransitions();
    }));

    for(const auto& c : node_handle_->getAllConnectors()) {
        connectConnector(c.get());
    }
    handle_connections_.emplace_back(node_handle_->connectorCreated.connect([this](ConnectablePtr c) {
        connectConnector(c.get());
    }));
    handle_connections_.emplace_back(node_handle_->connectorRemoved.connect([this](ConnectablePtr c) {
        disconnectConnector(c.get());
    }));

    auto af = delegate::bind(&NodeWorker::triggerCheckTransitions, this);
    node_handle_->getInputTransition()->setActivationFunction(af);
    node_handle_->getOutputTransition()->setActivationFunction(af);
}

NodeWorker::~NodeWorker()
{
    is_setup_ = false;

    for(auto& connection : handle_connections_) {
        connection.disconnect();
    }

    for(auto& pair : connections_) {
        disconnectConnector(pair.first);
    }

    connections_.clear();
}

NodeHandlePtr NodeWorker::getNodeHandle()
{
    return node_handle_;
}

UUID NodeWorker::getUUID() const
{
    return node_handle_->getUUID();
}

NodePtr NodeWorker::getNode() const
{
    return node_handle_->getNode().lock();
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

bool NodeWorker::isProcessingEnabled() const
{
    return node_handle_->getNodeState()->isEnabled();
}

void NodeWorker::setProcessingEnabled(bool e)
{
    node_handle_->getNodeState()->setEnabled(e);

    if(!e) {
        setError(false);
    }

    checkIO();
    enabled(e);
}

bool NodeWorker::isWaitingForTrigger() const
{
    for(TriggerPtr t : node_handle_->getTriggers()) {
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
    for(InputPtr i : node_handle_->getAllInputs()) {
        if(!i->isConnected() && !i->isOptional()) {
            return false;
        }
    }
    return true;
}

bool NodeWorker::canSend() const
{
    return node_handle_->getOutputTransition()->canStartSendingMessages();
}

void NodeWorker::triggerPanic()
{
    panic();
}

void NodeWorker::triggerCheckTransitions()
{
    checkTransitionsRequested();
}

void NodeWorker::reset()
{
    NodePtr node = node_handle_->getNode().lock();
    if(!node) {
        return;
    }

    node->abort();
    setError(false);

    // set state without checking!
    state_ = State::IDLE;

    node_handle_->getOutputTransition()->reset();
    node_handle_->getInputTransition()->reset();

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



void NodeWorker::startProcessingMessages()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    apex_assert_hard(state_ == State::ENABLED);

    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    setState(State::FIRED);

    if(!isProcessingEnabled()) {
        return;
    }


    apex_assert_hard(state_ == State::FIRED);
    apex_assert_hard(!node_handle_->getInputTransition()->hasUnestablishedConnection());
    apex_assert_hard(!node_handle_->getOutputTransition()->hasUnestablishedConnection());
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    apex_assert_hard(isProcessingEnabled());
    apex_assert_hard(canProcess());
    setState(State::PROCESSING);

    // everything has a message here

    {
        bool has_multipart = false;
        bool multipart_are_done = true;

        for(auto input : node_handle_->getAllInputs()) {
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
            for(auto output : node_handle_->getAllOutputs()) {
                bool is_last = multipart_are_done;
                output->setMultipart(true, is_last);
            }
        }
    }


    bool all_inputs_are_present = true;

    {
        std::unique_lock<std::recursive_mutex> lock(sync);

        // check if one is "NoMessage";
        for(InputPtr cin : node_handle_->getAllInputs()) {
            apex_assert_hard(cin->hasReceived() || (cin->isOptional() && !cin->isConnected()));
            if(!cin->isOptional() && !msg::hasMessage(cin.get())) {
                all_inputs_are_present = false;
            }
        }

        // update parameters
        for(auto pair : node_handle_->paramToInputMap()) {
            InputPtr cin = pair.second.lock();
            if(cin) {
                apex_assert_hard(cin->isOptional());
                if(msg::hasMessage(cin.get())) {
                    node_handle_->updateParameterValue(cin.get());
                }
            }
        }
    }

    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
    node_handle_->getOutputTransition()->setConnectionsReadyToReceive();
    node_handle_->getOutputTransition()->startReceiving();

    node_handle_->getInputTransition()->notifyMessageProcessed();

    if(!all_inputs_are_present) {
        finishProcessingMessages(false);
        return;
    }

    std::unique_lock<std::recursive_mutex> sync_lock(sync);

    current_process_timer_.reset();

    if(profiling_) {
        current_process_timer_.reset(new Timer(getUUID().getFullName()));
        timerStarted(this, PROCESS, current_process_timer_->startTimeMs());
    }

    NodePtr node = node_handle_->getNode().lock();
    if(!node) {
        return;
    }

    node->useTimer(current_process_timer_.get());

    bool sync = !node->isAsynchronous();

    try {
        if(sync) {
            node->process(*node);

        } else {
            node->process(*node, [this](std::function<void()> f) {
                node_handle_->executionRequested([this, f]() {
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
    for(InputPtr cin : node_handle_->getAllInputs()) {
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


void NodeWorker::notifyMessagesProcessed()
{
    {
        std::unique_lock<std::recursive_mutex> lock(state_mutex_);
        apex_assert_hard(node_handle_->getOutputTransition()->isSink() ||
                         node_handle_->getOutputTransition()->areOutputsIdle());
    }

    triggerCheckTransitions();
}


void NodeWorker::updateTransitionConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(state_ == State::IDLE || state_ == State::ENABLED) {
        node_handle_->getInputTransition()->updateConnections();
        node_handle_->getOutputTransition()->updateConnections();
    }
}

void NodeWorker::checkTransitions()
{
    checkTransitionsImpl(true);
}

void NodeWorker::checkTransitionsImpl(bool try_fire)
{
    {
        std::unique_lock<std::recursive_mutex> lock(sync);

        if(state_ != State::IDLE && state_ != State::ENABLED) {
            return;
        }
    }
    updateTransitionConnections();

    InputTransition* transition_in_ = node_handle_->getInputTransition();
    OutputTransition* transition_out_ = node_handle_->getOutputTransition();

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
    for(auto pair : node_handle_->outputToParamMap()) {
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
    auto map = node_handle_->paramToOutputMap();
    if(map.find(p->name()) != map.end()) {
        OutputPtr out = map.at(p->name()).lock();
        if(out) {
            publishParameterOn(*p, out.get());
        }
    }
}

void NodeWorker::sendMessages()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(getState() == State::PROCESSING);

    publishParameters();

    node_handle_->getOutputTransition()->sendMessages();
}


bool NodeWorker::tick()
{
    if(!is_setup_) {
        return false;
    }

    NodePtr node = node_handle_->getNode().lock();
    if(!node) {
        return false;
    }

    auto tickable = std::dynamic_pointer_cast<TickableNode>(node);
    apex_assert_hard(tickable);

    {
        if(!isProcessingEnabled()) {
            return false;
        }
    }

    bool has_ticked = false;

    if(isProcessingEnabled()) {
        std::unique_lock<std::recursive_mutex> lock(sync);
        auto state = getState();
        if(state == State::IDLE || state == State::ENABLED) {
            if(!isWaitingForTrigger() && tickable->canTick()) {
                if(state == State::ENABLED) {
                    setState(State::IDLE);
                }

                checkTransitionsImpl(false);

                if(node_handle_->getOutputTransition()->canStartSendingMessages() &&
                        !node_handle_->getOutputTransition()->hasFadingConnection()) {
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
                    node_handle_->getOutputTransition()->startReceiving();

                    TimerPtr t = nullptr;
                    if(profiling_) {
                        t.reset(new Timer(getUUID().getFullName()));
                        timerStarted(this, TICK, t->startTimeMs());
                    }
                    node->useTimer(t.get());

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
                    for(OutputPtr out : node_handle_->getAllOutputs()) {
                        if(msg::isConnected(out.get())) {
                            if(node_handle_->isParameterOutput(out.get())) {
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
                        node_handle_->getOutputTransition()->setConnectionsReadyToReceive();
                        sendMessages();

                    } else {
                        node_handle_->getOutputTransition()->abortSendingMessages();
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
    NodePtr node = node_handle_->getNode().lock();
    if(!node) {
        return;
    }

    node->checkConditions(false);

    // check if a parameter was changed
    Parameterizable::ChangedParameterList changed_params = node->getChangedParameters();
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


void NodeWorker::connectConnector(Connectable *c)
{
    connections_[c].emplace_back(c->connection_added_to.connect([this](Connectable*) { checkIO(); }));
    connections_[c].emplace_back(c->connectionEnabled.connect([this](bool) { checkIO(); }));
    connections_[c].emplace_back(c->connection_removed_to.connect([this](Connectable*) { checkIO(); }));
    connections_[c].emplace_back(c->enabled_changed.connect([this](bool) { checkIO(); }));
}

void NodeWorker::disconnectConnector(Connectable *c)
{
    for(auto& connection : connections_[c]) {
        connection.disconnect();
    }
    connections_[c].clear();
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
    for(InputPtr i : node_handle_->getAllInputs()) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}


void NodeWorker::enableOutput (bool enable)
{
    for(OutputPtr o : node_handle_->getAllOutputs()) {
        if(enable) {
            o->enable();
        } else {
            o->disable();
        }
    }
}

void NodeWorker::enableSlots (bool enable)
{
    for(SlotPtr i : node_handle_->getSlots()) {
        if(enable) {
            i->enable();
        } else {
            i->disable();
        }
    }
}

void NodeWorker::enableTriggers (bool enable)
{
    for(TriggerPtr i : node_handle_->getTriggers()) {
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
    node_handle_->getNodeState()->setMinimized(min);
}

void NodeWorker::errorEvent(bool error, const std::string& msg, ErrorLevel /*level*/)
{
    if(NodePtr node = node_handle_->getNode().lock()) {
        node->aerr << msg << std::endl;
    }

    errorHappened(error);

}
