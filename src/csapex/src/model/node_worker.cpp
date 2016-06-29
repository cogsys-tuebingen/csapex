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
#include <csapex/signal/event.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/factory/node_factory.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/tickable_node.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/delegate_bind.h>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/utility/exceptions.h>

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
    node_handle->setNodeWorker(this);

    NodePtr node = node_handle_->getNode().lock();


    try {
        handle_connections_.emplace_back(node_handle_->connectorCreated.connect([this](ConnectablePtr c) {
                                             connectConnector(c.get());
                                         }));
        handle_connections_.emplace_back(node_handle_->connectorRemoved.connect([this](ConnectablePtr c) {
                                             disconnectConnector(c.get());
                                         }));

        node->setupParameters(*node);

        if(!node->isIsolated()) {

            node->setup(*node_handle);

            handle_connections_.emplace_back(node_handle_->getOutputTransition()->messages_processed.connect([this](){
                notifyMessagesProcessed();
            }));

            node_handle_->addSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), "enable", std::bind(&NodeWorker::setProcessingEnabled, this, true), true);
            node_handle_->addSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), "disable", std::bind(&NodeWorker::setProcessingEnabled, this, false), false);


            auto tickable = std::dynamic_pointer_cast<TickableNode>(node);
            if(tickable) {
                trigger_tick_done_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"ticked");
            }

            trigger_activated_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"activated");
            trigger_deactivated_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"deactivated");

            auto generator = std::dynamic_pointer_cast<GeneratorNode>(node);
            if(generator) {
                generator->updated.connect(delegate::Delegate0<>(this, &NodeWorker::finishGenerator));
            }

            trigger_process_done_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"inputs processed");

            is_setup_ = true;

            handle_connections_.emplace_back(node_handle_->mightBeEnabled.connect([this]() {
                triggerTryProcess();
            }));
            handle_connections_.emplace_back(node_handle_->getNodeState()->enabled_changed->connect([this](){
                setProcessingEnabled(isProcessingEnabled());
            }));
            handle_connections_.emplace_back(node_handle_->activationChanged.connect([this](){
                if(node_handle_->isActive()) {
                    trigger_activated_->trigger();
                } else {
                    trigger_deactivated_->trigger();
                }
            }));

            auto af = delegate::bind(&NodeWorker::triggerTryProcess, this);
            node_handle_->getInputTransition()->setActivationFunction(af);
            node_handle_->getOutputTransition()->setActivationFunction(af);
        }

        sendEvents(node_handle_->isActive());

    } catch(const std::exception& e) {
        node->aerr << "setup failed: " << e.what() << std::endl;
    }
}

NodeWorker::~NodeWorker()
{
    destroyed();

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

long NodeWorker::getSequenceNumber() const
{
    return node_handle_->getOutputTransition()->getSequenceNumber();
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
    } else {
        checkIO();
    }
    enabled(e);
}

bool NodeWorker::canProcess() const
{
    return canReceive() && canSend();
}

bool NodeWorker::canReceive() const
{
    for(InputPtr i : node_handle_->getExternalInputs()) {
        if(!i->isConnected() && !i->isOptional()) {
            return false;
        }
    }
    return true;
}

bool NodeWorker::canSend() const
{
    if(!node_handle_->getOutputTransition()->canStartSendingMessages()) {
        return false;
    }

    for(Event* e : node_handle_->getEvents()){
        if(!e->canSendMessages()) {
            return false;
        }
    }

    return true;
}

void NodeWorker::triggerPanic()
{
    panic();
}

void NodeWorker::triggerTryProcess()
{
    tryProcessRequested();
}

void NodeWorker::reset()
{
    NodePtr node = node_handle_->getNode().lock();
    if(!node) {
        return;
    }

    node->reset();
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

    node_handle_->getInputTransition()->forwardMessages();

    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    setState(State::FIRED);


    apex_assert_hard(state_ == State::FIRED);
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    apex_assert_hard(canProcess());
    setState(State::PROCESSING);

    // everything has a message here

    bool has_active_token = false;

    {
        bool has_multipart = false;
        bool multipart_are_done = true;

        for(auto input : node_handle_->getExternalInputs()) {
            for(auto& c : input->getConnections()) {
                TokenConstPtr token = c->getToken();
                int f = token->flags.data;
                if(f & (int) Token::Flags::Fields::MULTI_PART) {
                    has_multipart = true;

                    bool last_part = f & (int) Token::Flags::Fields::LAST_PART;
                    multipart_are_done &= last_part;
                }

                if(c->holdsActiveToken()) {
                    has_active_token = true;
                }
            }
        }

        if(has_multipart) {
            for(auto output : node_handle_->getExternalOutputs()) {
                bool is_last = multipart_are_done;
                output->setMultipart(true, is_last);
            }
        }
    }


    bool all_inputs_are_present = true;
    connection_types::MarkerMessageConstPtr marker;

    {
        std::unique_lock<std::recursive_mutex> lock(sync);

        // check if one is a marker message
        for(InputPtr cin : node_handle_->getExternalInputs()) {
            apex_assert_hard(cin->hasReceived() || (cin->isOptional() && !cin->isConnected()));
            if(!cin->isOptional() && !msg::hasMessage(cin.get())) {
                all_inputs_are_present = false;
            }

            if(auto m = std::dynamic_pointer_cast<connection_types::MarkerMessage const>(cin->getToken()->getTokenData())) {
                if(!std::dynamic_pointer_cast<connection_types::NoMessage const>(m)) {
                    marker = m;
                    all_inputs_are_present = false;
                    break;
                }
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
    node_handle_->getOutputTransition()->clearBuffer();

    node_handle_->getInputTransition()->notifyMessageProcessed();


    NodePtr node = node_handle_->getNode().lock();
    if(!node) {
        return;
    }

    if(has_active_token) {
        if(!node_handle_->isActive()) {
            node_handle_->setActive(true);
        }
    }

    if(marker) {
        node->processMarker(marker);

        for(OutputPtr out : node_handle_->getExternalOutputs()) {
            msg::publish(out.get(), marker);
        }
    }

    if(marker || !all_inputs_are_present) {
        forwardMessages(false);
        signalMessagesProcessed();
        return;
    }

    std::unique_lock<std::recursive_mutex> sync_lock(sync);

    current_process_timer_.reset();

    if(profiling_) {
        current_process_timer_.reset(new Timer(getUUID().getFullName()));
        timerStarted(this, PROCESS, current_process_timer_->startTimeMs());
    }

    node->useTimer(current_process_timer_.get());

    bool sync = !node->isAsynchronous();

    if(isProcessingEnabled()) {
        try {
            if(sync) {
                node->process(*node_handle_, *node);

            } else {
                node->process(*node_handle_, *node, [this, node](std::function<void(csapex::NodeModifier&, Parameterizable &)> f) {
                    node_handle_->executionRequested([this, f, node]() {
                        f(*node_handle_, *node);
                        finishProcessing();
                    });
                });
            }



        } catch(const std::exception& e) {
            setError(true, e.what());
        } catch(const Failure& f) {
            throw;
        } catch(...) {
            throw Failure("Unknown exception caught in NodeWorker.");
        }
        if(sync) {
            finishProcessing();
        }

    } else {
        forwardMessages(true);
        signalMessagesProcessed();
        triggerTryProcess();
    }

}

void NodeWorker::finishGenerator()
{
    apex_assert_hard(canSend());

    bool has_msg = false;
    for(OutputPtr out : node_handle_->getExternalOutputs()) {
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

    if(has_msg) {
        activateOutput();

    } else {
        node_handle_->getOutputTransition()->setOutputsIdle();
    }

    //sendEventsAndMaybeDeactivate(node_handle_->isActive());
}

void NodeWorker::finishProcessing()
{
    signalExecutionFinished();
    forwardMessages(true);
    signalMessagesProcessed();
    triggerTryProcess();
}

void NodeWorker::signalExecutionFinished()
{
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

void NodeWorker::signalMessagesProcessed()
{
    setState(State::IDLE);

    messages_processed();
}

void NodeWorker::forwardMessages(bool send_parameters)
{
    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);

    if(!node_handle_->isSink()) {
        if(send_parameters) {
            publishParameters();
        }
    }
    sendMessages(true);
}

void NodeWorker::activateOutput()
{
    bool has_marker = false;
    for(OutputPtr out : node_handle_->getExternalOutputs()) {
        if(msg::isConnected(out.get()) && !node_handle_->isParameterOutput(out.get())) {
            if(out->hasMessage() && out->hasMarkerMessage()) {
                has_marker = true;
                break;
            }
        }
    }

    bool send_parameters = !has_marker;

    apex_assert_hard(getState() == NodeWorker::State::PROCESSING ||
                     getState() == NodeWorker::State::IDLE);
    if(send_parameters) {
        publishParameters();
    }
    sendMessages(false);
}

bool NodeWorker::areAllInputsAvailable() const
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    // check if all inputs have messages
    for(InputPtr cin : node_handle_->getExternalInputs()) {
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

    auto generator = std::dynamic_pointer_cast<GeneratorNode>(node_handle_->getNode().lock());
    if(generator) {
        apex_assert_hard(canSend());
        generator->notifyMessagesProcessed();
    }


    triggerTryProcess();
}


void NodeWorker::updateTransitionConnections()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(state_ == State::IDLE || state_ == State::ENABLED) {
        node_handle_->getInputTransition()->updateConnections();
        node_handle_->getOutputTransition()->updateConnections();
    }
}

void NodeWorker::updateState()
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
}

void NodeWorker::tryProcess()
{
    updateState();
    if(isEnabled() && canProcess()) {
        apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

        auto it = node_handle_->getInputTransition();

        int highest_deviant_seq = it->findHighestDeviantSequenceNumber();

        if(highest_deviant_seq >= 0) {
            it->notifyOlderConnections(highest_deviant_seq);
        } else {
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
        if(p.is<bool>())
            msg::publish(out, p.as<bool>());
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

bool NodeWorker::hasActiveOutputConnection()
{
    if(node_handle_->getOutputTransition()->hasActiveConnection()) {
        return true;
    }
    for(Event* e : node_handle_->getEvents()){
        for(const ConnectionPtr& c : e->getConnections()) {
            if(c->isEnabled() && c->isActive()) {
                return true;
            }
        }
    }
    return false;
}

void NodeWorker::sendEvents(bool active)
{
    bool sent_active_external = false;
    for(EventPtr e : node_handle_->getExternalEvents()){
        if(e->hasMessage()) {
            e->commitMessages(active);
            e->publish();
            if(e->hasActiveConnection()) {
                sent_active_external = true;
            }
        }
    }
    for(EventPtr e : node_handle_->getInternalEvents()){
        if(e->hasMessage()) {
            e->commitMessages(active);
            e->publish();
        }
    }

    if(node_handle_->isActive() && sent_active_external) {
        node_handle_->setActive(false);
    }
}

void NodeWorker::sendMessages(bool ignore_sink)
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(getState() == State::PROCESSING ||
                     getState() == State::IDLE);
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    for(Event* e : node_handle_->getEvents()){
        apex_assert_hard(e->canSendMessages());
    }

    //tokens are activated if the node is active.
    bool active = node_handle_->isActive();

    bool has_sent_active_message = false;
    if(!(ignore_sink && node_handle_->isSink())) {
        has_sent_active_message = node_handle_->getOutputTransition()->sendMessages(active);
    }

    sendEvents(active);

    // if there is an active connection -> deactivate
    if(active && has_sent_active_message) {
        node_handle_->setActive(false);
    }

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

    if(!isProcessingEnabled()) {
        return false;
    }

    bool has_ticked = false;

    if(isProcessingEnabled() && tickable->isTickEnabled()) {
        std::unique_lock<std::recursive_mutex> lock(sync);
        auto state = getState();
        if(state == State::IDLE || state == State::ENABLED) {
            if(tickable->canTick()) {
                if(state == State::ENABLED) {
                    setState(State::IDLE);
                }

                updateState();

                if(node_handle_->getOutputTransition()->canStartSendingMessages()) {

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
                    node_handle_->getOutputTransition()->clearBuffer();

                    TimerPtr t = nullptr;
                    if(profiling_) {
                        t.reset(new Timer(getUUID().getFullName()));
                        timerStarted(this, TICK, t->startTimeMs());
                    }
                    node->useTimer(t.get());

                    has_ticked = tickable->doTick(*node_handle_, *node);

                    if(has_ticked) {
                        if(trigger_tick_done_->isConnected()) {
                            trigger_tick_done_->trigger();
                        }

                        ticked();

                        ++ticks_;
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

    node->checkConditions(false);
}


void NodeWorker::connectConnector(Connectable *c)
{
    connections_[c].emplace_back(c->connection_added_to.connect([this](Connectable*) { checkIO(); }));
    connections_[c].emplace_back(c->connectionEnabled.connect([this](bool) { checkIO(); }));
    connections_[c].emplace_back(c->connection_removed_to.connect([this](Connectable*) { checkIO(); }));
    connections_[c].emplace_back(c->enabled_changed.connect([this](bool) { checkIO(); }));

    if(Event* event = dynamic_cast<Event*>(c)) {
        auto connection = event->triggered.connect([this, event]() {
            node_handle_->executionRequested([this, event]() {
                sendEvents(node_handle_->isActive());
            });
        });
        connections_[c].emplace_back(connection);

    } else if(Slot* slot = dynamic_cast<Slot*>(c)) {
        auto connection = slot->triggered.connect([this, slot]() {
            TokenPtr token = slot->getToken();
            apex_assert_hard(token);
            node_handle_->executionRequested([this, slot]() {
                TokenPtr token = slot->getToken();
                if(token) {
                    //apex_assert_hard(token);
                    if(token->isActive()) {
                        node_handle_->setActive(true);
                    }
                    slot->handleEvent();
                }
            });
        });
        connections_[c].emplace_back(connection);
    }
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
        triggerTryProcess();
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
