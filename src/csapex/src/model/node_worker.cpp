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
    : NodeHandle(type, uuid, node),
      is_setup_(false), state_(State::IDLE),
      trigger_tick_done_(nullptr), trigger_process_done_(nullptr),
      ticks_(0),
      source_(false), sink_(false), level_(0),
      profiling_(false)
{
    apex_assert_hard(node_);

    modifier_ = std::make_shared<NodeModifier>(this);
    node_->initialize(uuid, modifier_.get());

    bool params_created_in_constructor = node_->getParameterCount() != 0;
    apex_assert_hard(!params_created_in_constructor);

    transition_out_->messages_processed.connect([this](){
        notifyMessagesProcessed();
    });



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

void NodeWorker::triggerPanic()
{
    panic();
}

void NodeWorker::triggerCheckTransitions()
{
    checkTransitionsRequested();
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


void NodeWorker::connectConnector(Connectable *c)
{
    c->connection_added_to.connect([this](Connectable*) { checkIO(); });
    c->connectionEnabled.connect([this](bool) { checkIO(); });
    c->connection_removed_to.connect([this](Connectable*) { checkIO(); });

    c->enabled_changed.connect([this](bool) { checkIO(); });
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
