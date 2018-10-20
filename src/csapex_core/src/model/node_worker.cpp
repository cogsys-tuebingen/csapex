/// HEADER
#include <csapex/model/node_worker.h>

/// COMPONENT
#include <csapex/factory/node_factory_impl.h>
#include <csapex/model/generic_state.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_modifier.h>
#include <csapex/model/node_state.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/end_of_sequence_message.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/input.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/io.h>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/no_message.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/profiling/profiler_impl.h>
#include <csapex/profiling/timer.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/utility/debug.h>
#include <csapex/utility/delegate_bind.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/thread.h>
#include <csapex/serialization/packet_serializer.h>

/// SYSTEM
#include <thread>
#include <iostream>
#include <cstdlib>

using namespace csapex;

NodeWorker::NodeWorker(NodeHandlePtr node_handle)
  : node_handle_(node_handle)
  , is_setup_(false)
  , is_processing_(false)
  , trigger_process_done_(nullptr)
  , trigger_activated_(nullptr)
  , trigger_deactivated_(nullptr)
  , slot_enable_(nullptr)
  , slot_disable_(nullptr)
  , guard_(-1)
{
    //    node_handle->setNodeWorker(this);

    //    observe(node_handle->stopped, [this](){
    //        stopObserving();
    //    });

    profiler_ = std::make_shared<ProfilerImplementation>(false, 16);

    for (auto& port : node_handle_->getInternalInputs())
        connectConnector(port);
    for (auto& port : node_handle_->getInternalOutputs())
        connectConnector(port);
    for (auto& port : node_handle_->getInternalSlots())
        connectConnector(port);
    for (auto& port : node_handle_->getInternalEvents())
        connectConnector(port);

    observe(node_handle_->connector_created, [this](ConnectablePtr c, bool internal) { connectConnector(c); });
    observe(node_handle_->connector_removed, [this](ConnectablePtr c, bool internal) { disconnectConnector(c.get()); });

    observe(node_handle_->getInputTransition()->enabled_changed, [this]() { updateState(); });
    observe(node_handle_->getOutputTransition()->enabled_changed, [this]() { updateState(); });

    observe(node_handle_->getOutputTransition()->messages_processed, outgoing_messages_processed);


    for (const EventPtr& e : node_handle->getEvents()) {
        const std::string& label = e->getLabel();
        if (!trigger_activated_ && label == "activated")
            trigger_activated_ = e.get();
        else if (!trigger_deactivated_ && label == "deactivated")
            trigger_deactivated_ = e.get();
        else if (!trigger_process_done_ && label == "inputs processed")
            trigger_process_done_ = e.get();
    }

    if (!trigger_activated_)
        trigger_activated_ = node_handle_->addEvent(makeEmpty<connection_types::AnyMessage>(), "activated");
    if (!trigger_deactivated_)
        trigger_deactivated_ = node_handle_->addEvent(makeEmpty<connection_types::AnyMessage>(), "deactivated");
    if (!trigger_process_done_)
        trigger_process_done_ = node_handle_->addEvent(makeEmpty<connection_types::AnyMessage>(), "inputs processed");

    for (const SlotPtr& s : node_handle->getSlots()) {
        if (!slot_enable_ && s->getLabel() == "enable")
            slot_enable_ = s.get();
        if (!slot_disable_ && s->getLabel() == "disable")
            slot_disable_ = s.get();
    }
    if (!slot_enable_)
        slot_enable_ = node_handle_->addSlot(makeEmpty<connection_types::AnyMessage>(), "enable", [this]() { setProcessingEnabled(true); }, true, true);
    if (!slot_disable_)
        slot_disable_ = node_handle_->addSlot(makeEmpty<connection_types::AnyMessage>(), "disable", [this]() { setProcessingEnabled(false); }, false, true);

    observe(node_handle_->activation_changed, [this]() {
        if (node_handle_->isActive()) {
            msg::trigger(trigger_activated_);
        } else {
            msg::trigger(trigger_deactivated_);
        }
    });

    observe(node_handle_->might_be_enabled, [this]() { triggerTryProcess(); });
    observe(node_handle_->getNodeState()->enabled_changed, [this]() {
        bool e = isProcessingEnabled();

        for (const UUID& in : node_handle_->getInputTransition()->getInputs()) {
            node_handle_->getInputTransition()->getInput(in)->setEnabled(e);
        }
        for (const UUID& out : node_handle_->getOutputTransition()->getOutputs()) {
            node_handle_->getOutputTransition()->getOutput(out)->setEnabled(e);
        }

        for (const SlotPtr& slot : node_handle_->getSlots()) {
            slot->setEnabled(e);
        }
        for (const EventPtr& event : node_handle_->getEvents()) {
            event->setEnabled(e);
        }

        if (!e) {
            setError(false);
        } else {
            triggerTryProcess();
        }
        enabled(e);
    });

    NodePtr node = node_handle_->getNode().lock();

    observe(node->getParameterState()->parameter_changed, [this](param::Parameter* p) { triggerTryProcess(); });
    node->useTimer(profiler_->getTimer(node_handle->getUUID().getFullName()));
}

NodeWorker::~NodeWorker()
{
    stopObserving();

    std::unique_lock<std::recursive_mutex> lock(sync);

    destroyed();

    is_setup_ = false;

    for (auto& pair : port_connections_) {
        disconnectConnector(pair.first);
    }

    port_connections_.clear();

    guard_ = 0xDEADBEEF;
}

NodeHandlePtr NodeWorker::getNodeHandle() const
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
    apex_assert_hard(!node_handle_->getNode().expired());
    return node_handle_->getNode().lock();
}

ExecutionState NodeWorker::getExecutionState() const
{
    if (isEnabled()) {
        return ExecutionState::ENABLED;
    } else if (is_processing_) {
        return ExecutionState::PROCESSING;
    } else {
        return ExecutionState::IDLE;
    }
}

std::shared_ptr<ProfilerImplementation> NodeWorker::getProfiler()
{
    return profiler_;
}

bool NodeWorker::isEnabled() const
{
    return getNodeHandle()->getInputTransition()->isEnabled() && getNodeHandle()->getOutputTransition()->isEnabled();
}
bool NodeWorker::isIdle() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return !is_processing_;
}
bool NodeWorker::isProcessing() const
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    return is_processing_;
}

bool NodeWorker::isProcessingEnabled() const
{
    return node_handle_->getNodeState()->isEnabled();
}

void NodeWorker::setProcessingEnabled(bool e)
{
    node_handle_->getNodeState()->setEnabled(e);
}

bool NodeWorker::canProcess() const
{
    if (isProcessing()) {
        return false;
    }
    NodePtr node = getNode();
    if (!node) {
        return false;
    }
    if (!node->canProcess()) {
        return false;
    }

    return canReceive() && canSend();
}

bool NodeWorker::canReceive() const
{
    for (const InputPtr& i : node_handle_->getExternalInputs()) {
        if (i->isOptional()) {
            // optional -> do nothing
        } else {
            if (!i->isConnected()) {
                // !optional, !connected
                return false;
            } else if (!i->hasEnabledConnection()) {
                // !optional, connected, !enabled
                return false;
            }
        }
    }
    return true;
}

bool NodeWorker::canSend() const
{
    apex_assert_hard(guard_ == -1);
    if (!node_handle_->getOutputTransition()->canStartSendingMessages()) {
        return false;
    }

    for (const EventPtr& e : node_handle_->getExternalEvents()) {
        if (!e->canReceiveToken()) {
            return false;
        }
    }

    return true;
}

void NodeWorker::ioChanged()
{
    triggerTryProcess();
}

void NodeWorker::triggerTryProcess()
{
    try_process_changed();
}

void NodeWorker::initialize()
{
    is_setup_ = true;

    handleChangedParameters();

    sendEvents(node_handle_->isActive());
}

void NodeWorker::setProcessing(bool processing)
{
    std::unique_lock<std::recursive_mutex> lock(state_mutex_);
    is_processing_ = processing;
}

void NodeWorker::reset()
{
    if (NodePtr node = node_handle_->getNode().lock()) {
        node->reset();
    }

    setError(false);

    setProcessing(false);

    node_handle_->getOutputTransition()->reset();
    node_handle_->getInputTransition()->reset();

    triggerTryProcess();
}

void NodeWorker::setProfiling(bool profiling)
{
    if (isProfiling() != profiling) {
        profiler_->setEnabled(profiling);

        if (profiling) {
            start_profiling(this);
        } else {
            stop_profiling(this);
        }
    }
}

bool NodeWorker::isProfiling() const
{
    return profiler_->isEnabled();
}

bool NodeWorker::allInputsArePresent()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    for (const InputPtr& cin : node_handle_->getExternalInputs()) {
        apex_assert_hard(cin->hasReceived() || (cin->isOptional() && !cin->isConnected()));
        if (!cin->isOptional() && !msg::hasMessage(cin.get())) {
            return false;
        }

        if (cin->hasReceived()) {
            if (auto m = std::dynamic_pointer_cast<connection_types::MarkerMessage const>(cin->getToken()->getTokenData())) {
                if (!std::dynamic_pointer_cast<connection_types::NoMessage const>(m)) {
                    return false;
                }
            }
        }
    }

    return true;
}

connection_types::MarkerMessageConstPtr NodeWorker::getFirstMarkerMessage()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    for (const InputPtr& cin : node_handle_->getExternalInputs()) {
        apex_assert_hard(cin->hasReceived() || (cin->isOptional() && !cin->isConnected()));
        if (cin->hasReceived()) {
            if (auto m = std::dynamic_pointer_cast<connection_types::MarkerMessage const>(cin->getToken()->getTokenData())) {
                if (cin->isConnected()) {
                    return m;
                }
            }
        }
    }

    return {};
}

std::vector<ActivityModifier> NodeWorker::getIncomingActivityModifiers()
{
    std::vector<ActivityModifier> activity_modifiers;

    for (const auto& input : node_handle_->getExternalInputs()) {
        for (const ConnectionPtr& c : input->getConnections()) {
            if (c->holdsActiveToken()) {
                activity_modifiers.push_back(c->getToken()->getActivityModifier());
            }
        }
    }

    return activity_modifiers;
}

void NodeWorker::applyActivityModifiers(std::vector<ActivityModifier> activity_modifiers)
{
    bool activate = false;
    bool deactivate = false;

    for (ActivityModifier& modifier : activity_modifiers) {
        switch (modifier) {
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

    if (!node_handle_->isActive() && activate) {
        node_handle_->setActive(true);
    } else if (node_handle_->isActive() && deactivate) {
        node_handle_->setActive(false);
    }
}

void NodeWorker::processNode()
{
    std::unique_lock<std::recursive_mutex> lock(current_exec_mode_mutex_);

    NodePtr node = node_handle_->getNode().lock();
    apex_assert_hard(node);

    bool sync = !node->isAsynchronous();

    try {
        apex_assert_hard(node->getNodeHandle());
        if (sync) {
            node->process(*node_handle_, *node);

        } else {
            try {
                // TRACE node->ainfo << "process async" << std::endl;
                node->process(*node_handle_, *node, [this, node](ProcessingFunction f) {
                    node_handle_->execution_requested([this, f, node]() {
                        if (f) {
                            f(*node_handle_, *node);
                        }
                        // TRACE getNode()->ainfo << "async process done -> finish processing" << std::endl;
                        finishProcessing();
                    });
                });
            } catch (...) {
                // if flow is aborted -> call continuation anyway
                // TRACE getNode()->ainfo << "async process has thrown -> finish processing" << std::endl;
                finishProcessing();
                throw;
            }
        }

    } catch (const std::exception& e) {
        setError(true, e.what());
    } catch (const Failure& f) {
        throw f;
    } catch (...) {
        throw Failure("Unknown exception caught in NodeWorker.");
    }
    if (sync) {
        lock.unlock();
        // TRACE getNode()->ainfo << "sync process done -> finish processing" << std::endl;
        finishProcessing();
    }
}

void NodeWorker::processSlot(const SlotWeakPtr& slot_w)
{
    if (SlotPtr slot = slot_w.lock()) {
        const TokenPtr& token = slot->getToken();
        if (token) {
            if (!slot->isGraphPort() && token->hasActivityModifier()) {
                if (token->getActivityModifier() == ActivityModifier::ACTIVATE) {
                    node_handle_->setActive(true);

                } else if (token->getActivityModifier() == ActivityModifier::DEACTIVATE) {
                    node_handle_->setActive(false);
                }
            }

            Timer::Ptr timer;
            Trace::Ptr interlude;

            startProfilerInterval(TracingType::SLOT_CALLBACK);
            if (profiler_->isEnabled()) {
                interlude = timer->step(std::string("slot ") + slot->getLabel());
            }

            slot->handleEvent();

            interlude.reset();

            finishTimer(timer);
        }
    }
}

void NodeWorker::rememberExecutionMode()
{
    std::unique_lock<std::recursive_mutex> lock(current_exec_mode_mutex_);
    current_exec_mode_ = getNodeHandle()->getNodeState()->getExecutionMode();
    apex_assert_hard(current_exec_mode_);
}

void NodeWorker::startProfilerInterval(TracingType type)
{
    if (profiler_->isEnabled()) {
        Timer::Ptr timer = profiler_->getTimer(node_handle_->getUUID().getFullName());
        timer->restart();
        timer->root->setActive(node_handle_->isActive());
        interval_start(this, type, timer->root);
    }
}

void NodeWorker::stopActiveProfilerInterval()
{
    if (profiler_->isEnabled()) {
        finishTimer(profiler_->getTimer(node_handle_->getUUID().getFullName()));
    }
}

void NodeWorker::updateParameterValues()
{
    std::unique_lock<std::recursive_mutex> lock(sync);
    // update parameters
    bool change = node_handle_->updateParameterValues();
    if (change) {
        handleChangedParameters();
    }
}

bool NodeWorker::processMarker(const connection_types::MarkerMessageConstPtr& marker)
{
    if (!marker) {
        return false;
    }

    NodePtr node = node_handle_->getNode().lock();
    apex_assert_hard(node);

    auto no_message = std::dynamic_pointer_cast<connection_types::NoMessage const>(marker);

    if (no_message) {
        if (node->processNothingMarkers()) {
            // process the marker
            node->processMarker(marker);

            // (don't forward it to all outputs)
        }
        return true;

    } else {
        // process the marker
        node->processMarker(marker);

        // forward it to all outputs
        for (const OutputPtr& out : node_handle_->getExternalOutputs()) {
            msg::publish(out.get(), marker);
        }
        return false;
    }
}

bool NodeWorker::startProcessingMessages()
{
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

    NodePtr node = node_handle_->getNode().lock();
    apex_assert_hard(node);

    //rememberThreadId();

    if (node->hasChangedParameters()) {
        handleChangedParameters();
    }

    {
        std::unique_lock<std::recursive_mutex> lock(sync);
        apex_assert_hard(isEnabled());
        apex_assert_hard(canProcess());
        apex_assert_hard(!is_processing_);

        node_handle_->getInputTransition()->forwardMessages();

        apex_assert_hard(node_handle_->getInputTransition()->areMessagesComplete());

        apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
        for (const EventPtr& e : node_handle_->getEvents()) {
            for (const ConnectionPtr& c : e->getConnections()) {
                apex_assert_hard(c->getState() != Connection::State::UNREAD);
            }
        }

        setProcessing(true);

        updateParameterValues();
    }

    node_handle_->getInputTransition()->notifyMessageRead();

    if (!isProcessingEnabled()) {
        skipExecution();

        // signal that processing was successful
        // this is done so that no retries are scheduled.
        return true;
    }

    rememberExecutionMode();

    std::vector<ActivityModifier> activity_modifiers = getIncomingActivityModifiers();
    if (!activity_modifiers.empty()) {
        applyActivityModifiers(activity_modifiers);
    }

    auto marker = getFirstMarkerMessage();
    bool nothing_marker = processMarker(marker);
    if (marker && !nothing_marker) {
        // no processing because a non-NoMessage marker is received
        skipExecution();
        return false;
    }

    if (nothing_marker || !allInputsArePresent()) {
        // no processing because a non-optional port has a NoMessage marker
        bool is_required = node_handle_->getVertex()->getNodeCharacteristics().is_leading_to_joining_vertex || node_handle_->getVertex()->getNodeCharacteristics().is_leading_to_essential_vertex;
        if (is_required) {
            skipExecution();
        } else {
            pruneExecution();
        }
        return false;

    } else {
        std::unique_lock<std::recursive_mutex> lock(sync);

        startProfilerInterval(TracingType::PROCESS);

        // actually call the process function
        processNode();
        return true;
    }
}

bool NodeWorker::startProcessingSlot(const SlotWeakPtr& slot)
{
    processSlot(slot);
    return true;
}

void NodeWorker::pruneExecution()
{
    signalMessagesProcessed(true);
}

void NodeWorker::skipExecution()
{
    forwardMessages();
    signalMessagesProcessed(false);
}

void NodeWorker::finishProcessing()
{
    // TRACE getNode()->ainfo << "finish processing" << std::endl;
    apex_assert_hard(isProcessing());
    if (isProcessing()) {
        signalExecutionFinished();

        // TRACE getNode()->ainfo << "finish processing -> forward messages" << std::endl;

        publishParameters();
        forwardMessages();

        signalMessagesProcessed(false);

        triggerTryProcess();
    }
}

void NodeWorker::signalExecutionFinished()
{
    stopActiveProfilerInterval();

    if (trigger_process_done_->isConnected()) {
        msg::trigger(trigger_process_done_);
    }
}

void NodeWorker::signalMessagesProcessed(bool execution_pruned)
{
    setProcessing(false);

    bool is_pipelining = false;
    {
        std::unique_lock<std::recursive_mutex> lock(current_exec_mode_mutex_);
        is_pipelining = (current_exec_mode_ && current_exec_mode_.get() == ExecutionMode::PIPELINING);
    }

    if (execution_pruned || node_handle_->isSink() || is_pipelining) {
        // when we pruned the execution, the node effectively becomes a sink.
        // for a sink, we need to generate the processed notification to send back upstream.
        node_handle_->getInputTransition()->notifyMessageProcessed();
    }

    messages_processed();
}

void NodeWorker::forwardMessages()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(isProcessing());
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

    // tokens are activated if the node is active.
    bool active = node_handle_->isActive();

    lock.unlock();
    // TRACE getNode()->ainfo << "send messages" << std::endl;
    bool has_sent_activator_message = node_handle_->getOutputTransition()->sendMessages(active);
    lock.lock();

    sendEvents(active);

    // if there is an active connection -> deactivate
    if (active && has_sent_activator_message) {
        node_handle_->setActive(false);
    }
}

void NodeWorker::finishTimer(Timer::Ptr t)
{
    if (!t) {
        return;
    }

    t->finish();
    if (t->isEnabled()) {
        interval_end(this, t->root);
    }
}

void NodeWorker::notifyMessagesProcessedDownstream()
{
    //assertSameThreadId();

    if (auto subgraph = std::dynamic_pointer_cast<SubgraphNode>(node_handle_->getNode().lock())) {
        subgraph->notifyMessagesProcessed();
    }

    std::unique_lock<std::recursive_mutex> lock(current_exec_mode_mutex_);
    if (current_exec_mode_) {
        if (current_exec_mode_.get() == ExecutionMode::SEQUENTIAL) {
            // TRACE APEX_DEBUG_TRACE getNode()->ainfo << "done" << std::endl;
            node_handle_->getInputTransition()->notifyMessageProcessed();

            //            current_exec_mode_.reset();
        }

        // TRACE APEX_DEBUG_TRACE getNode()->ainfo << "notify, try process" << std::endl;
        //triggerTryProcess();

    } else {
        // TRACE APEX_DEBUG_TRACE getNode()->aerr << "cannot notify, no current exec mode" << std::endl;
    }

    triggerTryProcess();
}

void NodeWorker::updateState()
{
    if (isEnabled()) {
        triggerTryProcess();
    }
}

bool NodeWorker::canExecute()
{
    if (isEnabled() && canProcess()) {
        return true;
    } else {
        return false;
    }
}

void NodeWorker::publishParameters()
{
    std::unique_lock<std::recursive_mutex> lock(sync);

    apex_assert_hard(isProcessing());

    if (!node_handle_->isSink()) {
        for (auto pair : node_handle_->outputToParamMap()) {
            auto out = pair.first;
            auto p = pair.second;
            publishParameterOn(*p, node_handle_->getOutput(out).get());
        }
    }
}

void NodeWorker::publishParameter(csapex::param::Parameter* p)
{
    auto map = node_handle_->paramToOutputMap();
    if (map.find(p->name()) != map.end()) {
        OutputPtr out = map.at(p->name()).lock();
        if (out) {
            publishParameterOn(*p, out.get());
        }
    }
}

void NodeWorker::publishParameterOn(const csapex::param::Parameter& p, Output* out)
{
    if (out->isConnected()) {
        if (p.is<int>())
            msg::publish(out, p.as<int>());
        else if (p.is<double>())
            msg::publish(out, p.as<double>());
        if (p.is<bool>())
            msg::publish(out, p.as<bool>());
        else if (p.is<std::string>())
            msg::publish(out, p.as<std::string>());
        else if (p.is<std::pair<int, int>>())
            msg::publish(out, p.as<std::pair<int, int>>());
        else if (p.is<std::pair<double, double>>())
            msg::publish(out, p.as<std::pair<double, double>>());
    }
}

void NodeWorker::sendEvents(bool active)
{
    bool sent_active_external = false;
    for (const EventPtr& e : node_handle_->getExternalEvents()) {
        if (e->hasMessage() && e->isConnected()) {
            if (!e->canReceiveToken()) {
                continue;
            }

            //            apex_assert_hard(e->canReceiveToken());
            e->commitMessages(active);
            e->publish();
            if (e->hasActiveConnection()) {
                sent_active_external = true;
            }
        }
    }
    for (const EventPtr& e : node_handle_->getInternalEvents()) {
        if (e->hasMessage() && e->isConnected()) {
            if (!e->canReceiveToken()) {
                continue;
            }

            //            apex_assert_hard(e->canReceiveToken());
            e->commitMessages(active);
            e->publish();
        }
    }

    if (node_handle_->isActive() && sent_active_external) {
        node_handle_->setActive(false);
    }
}

void NodeWorker::handleChangedParameters()
{
    NodePtr node = node_handle_->getNode().lock();
    if (!node) {
        return;
    }

    // check if a parameter was changed
    Parameterizable::ChangedParameterList changed_params = node->getChangedParameters();
    if (!changed_params.empty()) {
        handleChangedParametersImpl(changed_params);
    }

    node->checkConditions(false);
}

void NodeWorker::handleChangedParametersImpl(const Parameterizable::ChangedParameterList& changed_params)
{
    for (auto pair : changed_params) {
        if (param::ParameterPtr p = pair.first.lock()) {
            try {
                // if(p->isEnabled()) {
                for (auto& cb : pair.second) {
                    cb(p.get());
                }
                //}

            } catch (const std::exception& e) {
                NOTIFICATION("parameter callback failed: " << e.what());
            }
        }
    }
}

void NodeWorker::connectConnector(ConnectablePtr c)
{
    port_connections_[c.get()].emplace_back(c->connection_added_to.connect([this](const ConnectorPtr&) { ioChanged(); }));
    port_connections_[c.get()].emplace_back(c->connectionEnabled.connect([this](bool) { ioChanged(); }));
    port_connections_[c.get()].emplace_back(c->connection_removed_to.connect([this](const ConnectorPtr&) { ioChanged(); }));
    port_connections_[c.get()].emplace_back(c->enabled_changed.connect([this](bool) { ioChanged(); }));

    if (EventPtr event = std::dynamic_pointer_cast<Event>(c)) {
        port_connections_[c.get()].emplace_back(event->triggered.connect([this]() { node_handle_->execution_requested([this]() { sendEvents(node_handle_->isActive()); }); }));
        port_connections_[c.get()].emplace_back(event->message_processed.connect([this](const ConnectorPtr&) { triggerTryProcess(); }));

    } else if (SlotPtr slot = std::dynamic_pointer_cast<Slot>(c)) {
        SlotWeakPtr slot_w = slot;
        auto connection = slot->triggered.connect([this, slot_w]() { getNodeHandle()->execution_requested([this, slot_w]() { processSlot(slot_w); }); });
        port_connections_[c.get()].emplace_back(connection);
    }
}

void NodeWorker::disconnectConnector(Connector* c)
{
    for (auto& connection : port_connections_[c]) {
        connection.disconnect();
    }
    port_connections_[c].clear();
}

void NodeWorker::errorEvent(bool error, const std::string& msg, ErrorLevel level)
{
    if (!msg.empty()) {
        if (NodePtr node = node_handle_->getNode().lock()) {
            if (level == ErrorLevel::ERROR) {
                node->aerr << msg << std::endl;
            } else if (level == ErrorLevel::WARNING) {
                node->awarn << msg << std::endl;
            } else {
                node->ainfo << msg << std::endl;
            }
        }
    }

    Notification message;
    message << msg;
    message.auuid = getUUID().getAbsoluteUUID();

    message.error = error ? level : ErrorLevel::NONE;

    notification(message);
}
