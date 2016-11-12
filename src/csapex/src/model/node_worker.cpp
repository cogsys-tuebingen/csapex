/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/input.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/profiling/timer.h>
#include <csapex/utility/thread.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph/vertex.h>
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
#include <csapex/profiling/profiler.h>
#include <csapex/utility/debug.h>

/// SYSTEM
#include <thread>
#include <iostream>

using namespace csapex;

NodeWorker::NodeWorker(NodeHandlePtr node_handle)
    : node_handle_(node_handle),
      is_setup_(false), state_(State::IDLE),
      trigger_tick_done_(nullptr), trigger_process_done_(nullptr),
      ticks_(0),
      guard_(-1)
{
    node_handle->setNodeWorker(this);

    profiler_ = std::make_shared<Profiler>(false, 16);

    NodePtr node = node_handle_->getNode().lock();
    node->useTimer(profiler_->getTimer(node_handle->getUUID().getFullName()));


    try {
        connections_.emplace_back(node_handle_->connectorCreated.connect([this](ConnectablePtr c) {
                                      connectConnector(c);
                                  }));
        connections_.emplace_back(node_handle_->connectorRemoved.connect([this](ConnectablePtr c) {
                                      disconnectConnector(c.get());
                                  }));

        node->setupParameters(*node);

        if(!node->isIsolated()) {

            node->setup(*node_handle);

            connections_.emplace_back(node_handle_->getOutputTransition()->messages_processed.connect([this](){
                outgoingMessagesProcessed();
            }));

            node_handle_->addSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), "enable", [this](){
                setProcessingEnabled(true);
            }, true, false);
            node_handle_->addSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), "disable", [this](){
                setProcessingEnabled(false);
            }, false, false);


            auto tickable = std::dynamic_pointer_cast<TickableNode>(node);
            if(tickable) {
                trigger_tick_done_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"ticked");
            }

            trigger_activated_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"activated");
            trigger_deactivated_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"deactivated");

            auto generator = std::dynamic_pointer_cast<GeneratorNode>(node);
            if(generator) {
                connections_.emplace_back(generator->finished.connect(delegate::Delegate0<>(this, &NodeWorker::triggerTryProcess)));
                connections_.emplace_back(generator->updated.connect(delegate::Delegate0<>(this, &NodeWorker::finishGenerator)));
            }

            trigger_process_done_ = node_handle_->addEvent(connection_types::makeEmpty<connection_types::AnyMessage>(),"inputs processed");

            is_setup_ = true;

            connections_.emplace_back(node_handle_->mightBeEnabled.connect([this]() {
                triggerTryProcess();
            }));
            connections_.emplace_back(node_handle_->getNodeState()->enabled_changed->connect([this](){
                setProcessingEnabled(isProcessingEnabled());
            }));
            connections_.emplace_back(node_handle_->activationChanged.connect([this](){
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
    std::unique_lock<std::recursive_mutex> lock(sync);

    destroyed();

    is_setup_ = false;

    for(auto& pair : port_connections_) {
        disconnectConnector(pair.first);
    }

    port_connections_.clear();

    guard_ = 0xDEADBEEF;
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

std::shared_ptr<Profiler> NodeWorker::getProfiler()
{
    return profiler_;
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

    for(SlotPtr slot : node_handle_->getSlots()) {
        slot->setEnabled(e);
    }
    for(EventPtr event : node_handle_->getEvents()) {
        event->setEnabled(e);
    }

    if(!e) {
        setError(false);
    } else {
        checkIO();
    }
    enabled(e);
}

bool NodeWorker::canProcess() const
{
    if(auto generator = std::dynamic_pointer_cast<GeneratorNode>(node_handle_->getNode().lock())) {
        if(!generator->canProcess()) {
            return false;
        }
    }
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
    apex_assert_hard(guard_ == -1);
    if(!node_handle_->getOutputTransition()->canStartSendingMessages()) {
        return false;
    }

    for(EventPtr e : node_handle_->getExternalEvents()){
        if(!e->canReceiveToken()) {
            return false;
        }
    }

    //    for(EventPtr e : node_handle_->getInternalEvents()){
    //        if(!e->canReceiveToken()) {
    //            return false;
    //        }
    //    }

    return true;
}

void NodeWorker::triggerPanic()
{
    panic();
}

void NodeWorker::triggerTryProcess()
{
    try_process_changed();
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
    profiler_->setEnabled(profiling);

    if(profiling) {
        start_profiling(this);
    } else {
        stop_profiling(this);
    }
}

bool NodeWorker::isProfiling() const
{
    return profiler_->isEnabled();
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

    apex_assert_hard(node_handle_->getInputTransition()->areMessagesComplete());

    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    for(EventPtr e : node_handle_->getEvents()) {
        for(ConnectionPtr c : e->getConnections()) {
            apex_assert_hard(c->getState() != Connection::State::UNREAD);
        }
    }
    setState(State::FIRED);


    apex_assert_hard(state_ == State::FIRED);
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
    apex_assert_hard(canProcess());
    setState(State::PROCESSING);

    // everything has a message here

    std::vector<ActivityModifier> activity_modifiers;

    {

        for(auto input : node_handle_->getExternalInputs()) {
            for(const ConnectionPtr& c : input->getConnections()) {
                if(c->holdsActiveToken()) {
                    activity_modifiers.push_back(c->getToken()->getActivityModifier());
                }
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
        bool change = false;
        for(auto pair : node_handle_->paramToInputMap()) {
            InputPtr cin = pair.second.lock();
            if(cin) {
                apex_assert_hard(cin->isOptional());
                if(msg::hasMessage(cin.get())) {
                    node_handle_->updateParameterValue(cin.get());
                    change = true;
                }
            }
        }
        if(change) {
            checkParameters();
        }
    }

    apex_assert_hard(getState() == NodeWorker::State::PROCESSING);
    node_handle_->getOutputTransition()->clearBuffer();

    NodePtr node = node_handle_->getNode().lock();

    lock.unlock();

    node_handle_->getInputTransition()->notifyMessageRead();

    if(!node) {
        return;
    }

    lock.lock();

    if(!activity_modifiers.empty()) {
        bool activate = false;
        bool deactivate = false;

        for(ActivityModifier& modifier : activity_modifiers) {
            switch(modifier) {
            case ActivityModifier::ACTIVATE:
                activate = true;
                break;
            case ActivityModifier::DEACTIVATE:
                deactivate = true;
                break;
            default:
                // nothing
                break;
            }
        }

        if(!node_handle_->isActive() && activate) {
            node_handle_->setActive(true);
        } else if(node_handle_->isActive() && deactivate) {
            node_handle_->setActive(false);
        }
    }

    if(marker) {
        node->processMarker(marker);

        for(OutputPtr out : node_handle_->getExternalOutputs()) {
            msg::publish(out.get(), marker);
        }
    }

    current_exec_mode_ = getNodeHandle()->getNodeState()->getExecutionMode();

    if(marker || !all_inputs_are_present) {
        if(!marker) {
            // no processing because a non-optional port has a NoMessage marker
            bool can_drop_marker = node_handle_->getVertex()->getNodeCharacteristics().is_vertex_separator
                    && !node_handle_->getVertex()->getNodeCharacteristics().is_leading_to_essential_vertex;
            if(can_drop_marker) {
                signalMessagesProcessed(true);
            } else {
                forwardMessages(false);
                signalMessagesProcessed();
            }

        } else {
            // no processing because a non-NoMessage marker is received
            forwardMessages(false);
            signalMessagesProcessed();
        }
        return;
    }


    if(profiler_->isEnabled()) {
        Timer::Ptr timer = profiler_->getTimer(node_handle_->getUUID().getFullName());
        timer->restart();
        timer->root->setActive(node_handle_->isActive());
        interval_start(this, PROCESS, timer->root);
    }


    bool sync = !node->isAsynchronous();

    if(isProcessingEnabled()) {
        try {
            if(sync) {
                node->process(*node_handle_, *node);

            } else {
                try {
                    node->process(*node_handle_, *node, [this, node](std::function<void(csapex::NodeModifier&, Parameterizable &)> f) {
                        node_handle_->executionRequested([this, f, node]() {
                            f(*node_handle_, *node);
                            finishProcessing();
                        });
                    });
                } catch(...) {
                    // if flow is aborted -> call continuation anyway
                    finishProcessing();
                    throw;
                }
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
    apex_assert_hard(guard_ == -1);
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

    triggerTryProcess();
}

void NodeWorker::finishProcessing()
{
    if(getState() == State::PROCESSING) {
        signalExecutionFinished();
        forwardMessages(true);
        signalMessagesProcessed();

        triggerTryProcess();
    }
}

void NodeWorker::signalExecutionFinished()
{
    finishTimer(profiler_->getTimer(node_handle_->getUUID().getFullName()));

    if(trigger_process_done_->isConnected()) {
        trigger_process_done_->trigger();
    }
}

void NodeWorker::signalMessagesProcessed(bool processing_aborted)
{
    setState(State::IDLE);

    if(processing_aborted || node_handle_->isSink() || (current_exec_mode_ && current_exec_mode_.get() == ExecutionMode::PIPELINING)) {
        APEX_DEBUG_TRACE getNode()->ainfo << "notify " << getUUID() <<", sink: " << node_handle_->isSink() << ", mode: " << (int) current_exec_mode_.get() << std::endl;
        node_handle_->getInputTransition()->notifyMessageProcessed();

        current_exec_mode_.reset();
    }

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
    if(!t) {
        return;
    }

    t->finish();
    if(t->isEnabled()) {
        interval_end(this, t->root);
    }
}

void NodeWorker::outgoingMessagesProcessed()
{
    if(auto generator = std::dynamic_pointer_cast<GeneratorNode>(node_handle_->getNode().lock())) {
        generator->notifyMessagesProcessed();
    }

    if(current_exec_mode_) {
        if(current_exec_mode_.get() == ExecutionMode::SEQUENTIAL) {
            APEX_DEBUG_TRACE getNode()->ainfo << "done" << std::endl;
            node_handle_->getInputTransition()->notifyMessageProcessed();

            current_exec_mode_.reset();
        }

        APEX_DEBUG_TRACE getNode()->ainfo << "notify, try process" << std::endl;
        triggerTryProcess();

    } else {
        APEX_DEBUG_TRACE getNode()->ainfo << "cannot notify, no current exec mode" << std::endl;
    }

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
    std::unique_lock<std::recursive_mutex> lock(sync);

    if(state_ != State::IDLE && state_ != State::ENABLED) {
        return;
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

bool NodeWorker::tryProcess()
{
    APEX_DEBUG_TRACE getNode()->ainfo << "try process" << std::endl;

    APEX_DEBUG_TRACE {
        getNode()->ainfo << "try to  process" << std::endl;
        getNode()->ainfo << "enabled: " << isEnabled() << ", process:" << canProcess() << ", state: " << (int) getState() << std::endl;
        getNode()->ainfo << "receive:" << canReceive() << " / send: " << canSend() << std::endl;
        getNode()->ainfo << "incoming message connections:" << std::endl;
        for(ConnectionPtr c : node_handle_->getInputTransition()->getConnections()) {
            getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
        }
        getNode()->ainfo << "outgoing message connections:" << std::endl;
        for(ConnectionPtr c : node_handle_->getOutputTransition()->getConnections()) {
            getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
        }
        getNode()->ainfo << "outputs:" << std::endl;
        for(UUID uid : node_handle_->getOutputTransition()->getOutputs()) {
            OutputPtr o =node_handle_->getOutputTransition()->getOutput(uid);
            getNode()->ainfo << "- " << o->getUUID() << ": " << (int) o->getState() << std::endl;
        }
        getNode()->ainfo << "event connections:" << std::endl;
        for(EventPtr e : node_handle_->getExternalEvents()){
            for(ConnectionPtr c : e->getConnections()) {
                getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
            }
        }
        getNode()->ainfo << "slot connections:" << std::endl;
        for(SlotPtr s : node_handle_->getExternalSlots()){
            for(ConnectionPtr c : s->getConnections()) {
                getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
            }
        }
    }
    updateState();
    if(isEnabled() && canProcess()) {
        apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

        startProcessingMessages();
        return true;

    } else {
        APEX_DEBUG_TRACE {
            getNode()->ainfo << "cannot process" << std::endl;
            getNode()->ainfo << "enabled: " << isEnabled() << ", process:" << canProcess() << ", state: " << (int) getState() << std::endl;
            getNode()->ainfo << "receive:" << canReceive() << " / send: " << canSend() << std::endl;
            getNode()->ainfo << "incoming message connections:" << std::endl;
            for(ConnectionPtr c : node_handle_->getInputTransition()->getConnections()) {
                getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
            }
            getNode()->ainfo << "outgoing message connections:" << std::endl;
            for(ConnectionPtr c : node_handle_->getOutputTransition()->getConnections()) {
                getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
            }
            getNode()->ainfo << "outputs:" << std::endl;
            for(UUID uid : node_handle_->getOutputTransition()->getOutputs()) {
                OutputPtr o =node_handle_->getOutputTransition()->getOutput(uid);
                getNode()->ainfo << "- " << o->getUUID() << ": " << (int) o->getState() << std::endl;
            }
            getNode()->ainfo << "event connections:" << std::endl;
            for(EventPtr e : node_handle_->getExternalEvents()){
                for(ConnectionPtr c : e->getConnections()) {
                    getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
                }
            }
            getNode()->ainfo << "slot connections:" << std::endl;
            for(SlotPtr s : node_handle_->getExternalSlots()){
                for(ConnectionPtr c : s->getConnections()) {
                    getNode()->ainfo << "- " << *c << ": " << (int) c->getState() << std::endl;
                }
            }
        }
    }

    return false;
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
    for(EventPtr e : node_handle_->getEvents()){
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
        if(e->hasMessage() && e->isConnected()) {
            if(!e->canReceiveToken()) {
                continue;
            }

            //            apex_assert_hard(e->canReceiveToken());
            e->commitMessages(active);
            e->publish();
            if(e->hasActiveConnection()) {
                sent_active_external = true;
            }
        }
    }
    for(EventPtr e : node_handle_->getInternalEvents()){
        if(e->hasMessage() && e->isConnected()) {
            if(!e->canReceiveToken()) {
                continue;
            }

            //            apex_assert_hard(e->canReceiveToken());
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

    //tokens are activated if the node is active.
    bool active = node_handle_->isActive();

    bool has_sent_activator_message = false;
    if(!(ignore_sink && node_handle_->isSink())) {
        has_sent_activator_message = node_handle_->getOutputTransition()->sendMessages(active);
    }

    sendEvents(active);

    // if there is an active connection -> deactivate
    if(active && has_sent_activator_message) {
        node_handle_->setActive(false);
    }

}


bool NodeWorker::tick()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    apex_assert_hard(guard_ == -1);
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
        auto state = getState();
        if(state == State::IDLE || state == State::ENABLED) {
            if(tickable->canTick()) {
                if(state == State::ENABLED) {
                    setState(State::IDLE);
                }

                updateState();

                if(node_handle_->getOutputTransition()->canStartSendingMessages()) {

                    current_exec_mode_ = getNodeHandle()->getNodeState()->getExecutionMode();

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

                    Timer::Ptr timer;

                    if(profiler_->isEnabled()) {
                        timer = profiler_->getTimer(node_handle_->getUUID().getFullName());
                        timer->restart();
                        timer->root->setActive(node_handle_->isActive());
                        interval_start(this, TICK, timer->root);
                    }


                    has_ticked = tickable->doTick(*node_handle_, *node);

                    if(has_ticked) {
                        if(trigger_tick_done_->isConnected()) {
                            trigger_tick_done_->trigger();
                        }

                        if(node_handle_->isSource()) {
                            // reset input transition on tick
                            node_handle_->getInputTransition()->forwardMessages();
                        }

                        apex_assert_hard(guard_ == -1);
                        ticked();

                        ++ticks_;
                    }

                    finishTimer(timer);

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
            if(param::ParameterPtr p = pair.first.lock()) {
                try {
                    if(p->isEnabled()) {
                        pair.second(p.get());
                    }

                } catch(const std::exception& e) {
                    std::cerr << "parameter callback failed: " << e.what() << std::endl;
                }
            }
        }
    }

    node->checkConditions(false);
}

void NodeWorker::trySendEvents()
{
    sendEvents(node_handle_->isActive());
}

void NodeWorker::connectConnector(ConnectablePtr c)
{
    port_connections_[c.get()].emplace_back(c->connection_added_to.connect([this](Connectable*) { checkIO(); }));
    port_connections_[c.get()].emplace_back(c->connectionEnabled.connect([this](bool) { checkIO(); }));
    port_connections_[c.get()].emplace_back(c->connection_removed_to.connect([this](Connectable*) { checkIO(); }));
    port_connections_[c.get()].emplace_back(c->enabled_changed.connect([this](bool) { checkIO(); }));

    if(EventPtr event = std::dynamic_pointer_cast<Event>(c)) {
        port_connections_[c.get()].emplace_back(event->triggered.connect([this]() {
            node_handle_->executionRequested([this]() {
                sendEvents(node_handle_->isActive());
            });
        }));
        port_connections_[c.get()].emplace_back(event->message_processed.connect([this](Connectable*) {
                                                    triggerTryProcess();
                                                }));

    } else if(SlotPtr slot = std::dynamic_pointer_cast<Slot>(c)) {
        SlotWeakPtr slot_w = slot;
        auto connection = slot->triggered.connect([this, slot_w]() {
            node_handle_->executionRequested([this, slot_w]() {
                if(SlotPtr slot = slot_w.lock()) {
                    TokenPtr token = slot->getToken();
                    if(token) {
                        //apex_assert_hard(token);
                        if(token->hasActivityModifier()) {
                            if(token->getActivityModifier() == ActivityModifier::ACTIVATE) {
                                node_handle_->setActive(true);

                            } else if(token->getActivityModifier() == ActivityModifier::DEACTIVATE) {
                                node_handle_->setActive(false);
                            }
                        }

                        Timer::Ptr timer;
                        Interlude::Ptr interlude;

                        if(profiler_->isEnabled()) {
                            timer = profiler_->getTimer(node_handle_->getUUID().getFullName());
                            timer->restart();
                            timer->root->setActive(node_handle_->isActive());
                            interval_start(this, SLOT, timer->root);

                            interlude = timer->step(std::string("slot ") + slot->getLabel());
                        }

                        slot->handleEvent();

                        interlude.reset();

                        finishTimer(timer);
                    }
                }
            });
        });
        port_connections_[c.get()].emplace_back(connection);
    }
}

void NodeWorker::disconnectConnector(Connectable *c)
{
    for(auto& connection : port_connections_[c]) {
        connection.disconnect();
    }
    port_connections_[c].clear();
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

void NodeWorker::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    if(!msg.empty()) {
        if(NodePtr node = node_handle_->getNode().lock()) {
            if(level == ErrorLevel::ERROR) {
                node->aerr << msg << std::endl;
            } else if(level == ErrorLevel::WARNING) {
                node->awarn << msg << std::endl;
            } else {
                node->ainfo << msg << std::endl;
            }
        }
    }

    Notification message;
    message.message = msg;
    message.auuid = getUUID().getAbsoluteUUID();

    message.error = error ? level : ErrorLevel::NONE;

    notification(message);
}
