/// HEADER
#include <csapex/model/subgraph_node.h>

/// PROJECT
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/bitset_parameter.h>
#include <csapex/utility/debug.h>
#include <csapex/model/node_worker.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

SubgraphNode::SubgraphNode(GraphImplementationPtr graph)
    : graph_(graph),
      transition_relay_in_(new InputTransition),
      transition_relay_out_(new OutputTransition),
      is_subgraph_finished_(false),
      is_iterating_(false), has_sent_current_iteration_(false),
      is_initialized_(false),

      activation_event_(nullptr),
      deactivation_event_(nullptr),

      guard_(-1)
{
    transition_relay_in_->setActivationFunction(delegate::Delegate0<>(this, &SubgraphNode::subgraphHasProducedAllMessages));
    observe(transition_relay_out_->messages_processed, delegate::Delegate0<>(this, &SubgraphNode::currentIterationIsProcessed));
}

SubgraphNode::~SubgraphNode()
{
    guard_ = 0xDEADBEEF;
}

bool SubgraphNode::canRunInSeparateProcess() const
{
    return false;
}

GraphPtr SubgraphNode::getGraph() const
{
    return graph_;
}

GraphImplementationPtr SubgraphNode::getLocalGraph() const
{
    return graph_;
}

void SubgraphNode::initialize(NodeHandlePtr node_handle)
{
    Node::initialize(node_handle);

    if(node_handle->getUUIDProvider()) {
        graph_->setParent(node_handle->getUUIDProvider()->shared_from_this(), node_handle->getUUID().getAbsoluteUUID());
    }
}

void SubgraphNode::setNodeFacade(csapex::NodeFacadeImplementation* graph_node_facade)
{
    graph_->setNodeFacade(graph_node_facade);
}

void SubgraphNode::detach()
{
    graph_->clear();
    Node::detach();
}

void SubgraphNode::tearDown()
{
    is_initialized_ = false;
}

void SubgraphNode::reset()
{
    Node::reset();

    graph_->resetActivity();

    {
        std::unique_lock<std::recursive_mutex> lock(continuation_mutex_);
        continuation_ = Continuation();
    }

    transition_relay_out_->reset();
    transition_relay_in_->reset();
}

void SubgraphNode::stateChanged()
{
    Node::stateChanged();

    if(!is_initialized_) {
        is_initialized_ = true;
    }
}
void SubgraphNode::setup(NodeModifier &modifier)
{
    setupVariadic(modifier);

    activation_event_ = createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), graph_->makeUUID("event_activation"), "activation");
    deactivation_event_ = createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), graph_->makeUUID("event_deactivation"), "deactivation");
}

void SubgraphNode::activation()
{
    if(activation_event_) {
        TokenDataConstPtr data(new connection_types::AnyMessage);
        TokenPtr token = std::make_shared<Token>(data);
        token->setActivityModifier(ActivityModifier::ACTIVATE);
        activation_event_->triggerWith(token);
    }
}

void SubgraphNode::deactivation()
{
    if(deactivation_event_) {
        TokenDataConstPtr data(new connection_types::AnyMessage);
        TokenPtr token = std::make_shared<Token>(data);
        token->setActivityModifier(ActivityModifier::DEACTIVATE);
        deactivation_event_->triggerWith(token);
    }
}

bool SubgraphNode::canProcess() const
{
    if(!is_initialized_) {
        //TRACE ainfo << "cannot process: not initialized" << std::endl;
        return false;
    }
    if(!transition_relay_out_->canStartSendingMessages()) {
        //TRACE ainfo << "cannot process, out relay cannot send" << std::endl;
        return false;
    }
    if(transition_relay_in_->hasConnection() ||
            transition_relay_out_->hasConnection()) {
        return true;
    } else {
        return false;
    }
}


void SubgraphNode::setupParameters(Parameterizable &params)
{
    setupVariadicParameters(params);

    params.addParameter(param::ParameterFactory::declareBool("iterate_containers",
                                                             param::ParameterDescription("When true, input vectors will be iterated internally"),
                                                             false));


    std::map<std::string, std::pair<int, bool> > flags;

    iterated_inputs_param_ = std::dynamic_pointer_cast<param::BitSetParameter>(param::ParameterFactory::declareParameterBitSet("iterated_containers", flags).build());

    params.addConditionalParameter(iterated_inputs_param_,
                                   [this](){
        return readParameter<bool>("iterate_containers");
    },
    [this](param::Parameter* p) {
        for(auto& pair : relay_to_external_input_) {
            UUID id = pair.second;
            InputPtr i = node_handle_->getInput(id);
            apex_assert_hard(i);

            bool iterate = iterated_inputs_param_->isSet(id.getFullName());
            setIterationEnabled(id, iterate);
        }
    });
}

void SubgraphNode::process(NodeModifier &node_modifier, Parameterizable &params, Continuation continuation)
{
    {
        std::unique_lock<std::recursive_mutex> lock(continuation_mutex_);
        continuation_ = continuation;
    }
    //TRACE ainfo << "start process with continuation" << std::endl;

    apex_assert_hard(is_initialized_);

    // can fail...
    apex_assert_hard(transition_relay_out_->areAllConnections(Connection::State::NOT_INITIALIZED));
    apex_assert_hard(transition_relay_out_->canStartSendingMessages());

    is_iterating_ = false;
    has_sent_current_iteration_ = false;
    is_subgraph_finished_ = false;

    for(const InputPtr& i : node_modifier.getMessageInputs()) {
        if(msg::hasMessage(i.get())) {
            TokenDataConstPtr m = msg::getMessage(i.get());
            OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

            if(m->isContainer() && iterated_inputs_.find(i->getUUID()) != iterated_inputs_.end()) {
                is_iterating_ = true;
                iteration_count_ = m->nestedValueCount();
                iteration_index_ = 1;

                if(iteration_count_ > 0)  {
                    msg::publish(o.get(), m->nestedValue(0));
                }

            } else {
                msg::publish(o.get(), m);
            }
        }
    }

    if(transition_relay_out_->hasConnection()) {
        //TRACE ainfo << "send internal output messages" << std::endl;
        transition_relay_out_->sendMessages(node_handle_->isActive());
    }

    tryFinishSubgraph();
    if(transition_relay_in_->areMessagesForwarded()) {
        //TRACE ainfo << "process: all messages forwarded -> finish subgraph" << std::endl;
        finishSubgraph();
    }
}

bool SubgraphNode::isAsynchronous() const
{
    return true;
}

bool SubgraphNode::isIterating() const
{
    return is_iterating_;
}

namespace {
void crossConnectLabelChange(Connectable* a, Connectable* b)
{
    a->labelChanged.connect([b](const std::string& label) {
        b->setLabel(label);
    });
    b->labelChanged.connect([a](const std::string& label) {
        a->setLabel(label);
    });
}
}

Input* SubgraphNode::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional)
{
    auto pair = addForwardingInput(type, label, optional);
    return node_handle_->getInput(pair.external).get();
}

InputPtr SubgraphNode::createInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool optional)
{
    InputPtr input = node_handle_->addInternalInput(type, internal_uuid, label, optional);
    input->setGraphPort(true);
    input->setEssential(true);

    transition_relay_in_->addInput(input);

    return input;
}


void SubgraphNode::removeVariadicInput(InputPtr input)
{
    OutputPtr relay = external_to_internal_outputs_[input->getUUID()];
    forwarding_connector_removed(relay);

    VariadicInputs::removeVariadicInput(input);

    relay_to_external_input_.erase(relay->getUUID());

    external_to_internal_outputs_.erase(input->getUUID());
    transition_relay_out_->removeOutput(relay);
}

RelayMapping SubgraphNode::addForwardingInput(const TokenDataConstPtr& type,
                                              const std::string& label, bool optional)
{
    UUID internal_uuid = graph_->generateDerivedUUID(UUID(),"relayout");
    UUID external_uuid = addForwardingInput(internal_uuid, type, label, optional);

    return {external_uuid, internal_uuid};
}

UUID  SubgraphNode::addForwardingInput(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label, bool optional)
{
    graph_->registerUUID(internal_uuid);

    Input* external_input = VariadicInputs::createVariadicInput(type, label, optional);

    OutputPtr relay = createInternalOutput(type, internal_uuid, label);

    crossConnectLabelChange(external_input, relay.get());

    external_to_internal_outputs_[external_input->getUUID()] = relay;

    relay_to_external_input_[internal_uuid] = external_input->getUUID();

    forwarding_connector_added(relay);

    std::map<std::string, int> possibly_iterated_inputs;
    for(auto& pair : iterated_inputs_param_->getBitSet()) {
        possibly_iterated_inputs[pair.first] = pair.second;

    }
    possibly_iterated_inputs[external_input->getUUID().getFullName()] = possibly_iterated_inputs.size();

    iterated_inputs_param_->setBitSet(possibly_iterated_inputs);

    return external_input->getUUID();
}


Output* SubgraphNode::createVariadicOutput(TokenDataConstPtr type, const std::string& label)
{
    auto pair = addForwardingOutput(type, label);
    return node_handle_->getOutput(pair.external).get();
}


OutputPtr SubgraphNode::createInternalOutput(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    OutputPtr output = node_handle_->addInternalOutput(type, internal_uuid, label);
    output->setGraphPort(true);
    output->setEssential(true);

    transition_relay_out_->addOutput(output);

    return output;
}

void SubgraphNode::removeVariadicOutput(OutputPtr output)
{
    InputPtr relay = external_to_internal_inputs_[output->getUUID()];
    forwarding_connector_removed(relay);

    relay->message_set.disconnectAll();

    VariadicOutputs::removeVariadicOutput(output);

    relay_to_external_output_.erase(relay->getUUID());

    external_to_internal_inputs_.erase(output->getUUID());
    transition_relay_in_->removeInput(relay);
}

RelayMapping SubgraphNode::addForwardingOutput(const TokenDataConstPtr& type,
                                               const std::string& label)
{
    UUID internal_uuid = graph_->generateDerivedUUID(UUID(),"relayin");
    UUID external_uuid = addForwardingOutput(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID SubgraphNode::addForwardingOutput(const UUID& internal_uuid, const TokenDataConstPtr& type,  const std::string& label)
{
    graph_->registerUUID(internal_uuid);

    Output* external_output = VariadicOutputs::createVariadicOutput(type, label);

    InputPtr relay = createInternalInput(type, internal_uuid, label, true);

    crossConnectLabelChange(external_output, relay.get());

    std::weak_ptr<Output> external_output_weak = std::dynamic_pointer_cast<Output>(external_output->shared_from_this());
    relay->message_set.connect([this, external_output_weak, relay](Connectable*) {
        if(auto external_output = external_output_weak.lock()) {
            TokenPtr token = relay->getToken();
            if(is_iterating_) {
                connection_types::GenericVectorMessage::Ptr vector;
                if(!external_output->hasMessage()) {
                    vector = connection_types::GenericVectorMessage::make(token->getTokenData());
                } else {
                    auto collected = external_output->getAddedToken()->getTokenData()->clone();
                    vector = std::dynamic_pointer_cast<connection_types::GenericVectorMessage>(collected);
                }
                apex_assert(vector);
                vector->addNestedValue(token->getTokenData());
                msg::publish(external_output.get(), vector);

                external_output->setType(vector);

            } else {
                msg::publish(external_output.get(), token->getTokenData());
            }
        }
    });

    external_to_internal_inputs_[external_output->getUUID()] = relay;

    relay_to_external_output_[internal_uuid] = external_output->getUUID();

    forwarding_connector_added(relay);

    return external_output->getUUID();
}



SlotPtr SubgraphNode::createInternalSlot(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, std::function<void (const TokenPtr& )> callback)
{
    SlotPtr slot = node_handle_->addInternalSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), internal_uuid, label, callback);
    slot->setGraphPort(true);
    slot->setEssential(true);

    internal_slots_[internal_uuid] = slot;
    internal_slot_ids_.push_back(slot->getUUID());

    return slot;
}

Slot* SubgraphNode::createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr&)> /*callback*/, bool /*active*/, bool /*blocking*/)
{
    auto pair = addForwardingSlot(type, label);
    return node_handle_->getSlot(pair.external).get();
}

void SubgraphNode::removeVariadicSlot(SlotPtr slot)
{
    EventPtr relay = external_to_internal_events_.at(slot->getUUID());
    external_to_internal_events_.erase(slot->getUUID());

    internal_events_.erase(relay->getUUID());

    auto it = std::find(internal_event_ids_.begin(), internal_event_ids_.end(), relay->getUUID());
    if(it != internal_event_ids_.end()) {
        internal_event_ids_.erase(it);
    }

    forwarding_connector_removed(relay);

    VariadicSlots::removeVariadicSlot(slot);

    relay_to_external_slot_.erase(relay->getUUID());
}

RelayMapping SubgraphNode::addForwardingSlot(const TokenDataConstPtr& type, const std::string& label)
{
    UUID internal_uuid = graph_->generateDerivedUUID(UUID(),"relayevent");
    UUID external_uuid = addForwardingSlot(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID SubgraphNode::addForwardingSlot(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label)
{
    graph_->registerUUID(internal_uuid);

    EventPtr relay = createInternalEvent(type, internal_uuid, label);

    auto cb = [relay](const TokenConstPtr& data) {
        relay->triggerWith(std::make_shared<Token>(*data));
        relay->message_processed(relay);
    };

    Slot* external_slot = VariadicSlots::createVariadicSlot(type, label, cb, false, false);

    relay->message_processed.connect(std::bind(&Slot::notifyEventHandled, external_slot));

    crossConnectLabelChange(external_slot, relay.get());

    external_to_internal_events_[external_slot->getUUID()] = relay;

    relay_to_external_slot_[internal_uuid] = external_slot->getUUID();

    forwarding_connector_added(relay);

    return external_slot->getUUID();
}

EventPtr SubgraphNode::createInternalEvent(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    EventPtr event = node_handle_->addInternalEvent(type, internal_uuid, label);
    event->setGraphPort(true);
    event->setEssential(true);

    internal_events_[internal_uuid] = event;

    internal_event_ids_.push_back(event->getUUID());

    //    event->triggered.connect([this]() {
    //        std::cerr << "tigger internal event" << std::endl;
    //    });

    return event;
}

Event* SubgraphNode::createVariadicEvent(TokenDataConstPtr type, const std::string& label)
{
    auto pair = addForwardingEvent(type, label);
    return node_handle_->getEvent(pair.external).get();
}

void SubgraphNode::removeVariadicEvent(EventPtr event)
{
    SlotPtr relay = external_to_internal_slots_.at(event->getUUID());
    external_to_internal_slots_.erase(event->getUUID());

    internal_slots_.erase(relay->getUUID());

    auto it = std::find(internal_slot_ids_.begin(), internal_slot_ids_.end(), relay->getUUID());
    if(it != internal_slot_ids_.end()) {
        internal_slot_ids_.erase(it);
    }

    forwarding_connector_removed(relay);

    VariadicEvents::removeVariadicEvent(event);

    relay_to_external_event_.erase(relay->getUUID());
}

RelayMapping SubgraphNode::addForwardingEvent(const TokenDataConstPtr& type, const std::string& label)
{
    UUID internal_uuid = graph_->generateDerivedUUID(UUID(),"relayslot");
    UUID external_uuid = addForwardingEvent(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID SubgraphNode::addForwardingEvent(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label)
{
    graph_->registerUUID(internal_uuid);

    Event* external_event = VariadicEvents::createVariadicEvent(type, label);

    auto cb = [this, external_event](const TokenPtr& token){
        if(external_event->isConnected()) {
            external_event->triggerWith(token);
//            node_handle_->getNodeWorker()->trySendEvents();
        } else {
            external_event->notifyMessageProcessed();
        }
    };

    SlotPtr relay = createInternalSlot(type, internal_uuid, label, cb);
    external_event->message_processed.connect(std::bind(&Slot::notifyEventHandled, relay.get()));

    crossConnectLabelChange(external_event, relay.get());

    external_to_internal_slots_[external_event->getUUID()] = relay;

    relay_to_external_event_[internal_uuid] = external_event->getUUID();

    forwarding_connector_added(relay);

    return external_event->getUUID();
}

OutputPtr SubgraphNode::getRelayForInput(const UUID& external_uuid) const
{
    return external_to_internal_outputs_.at(external_uuid);
}
InputPtr SubgraphNode::getRelayForOutput(const UUID& external_uuid) const
{
    return external_to_internal_inputs_.at(external_uuid);
}
EventPtr SubgraphNode::getRelayForSlot(const UUID& external_uuid) const
{
    return external_to_internal_events_.at(external_uuid);
}
SlotPtr SubgraphNode::getRelayForEvent(const UUID& external_uuid) const
{
    return external_to_internal_slots_.at(external_uuid);
}

InputPtr SubgraphNode::getForwardedInputInternal(const UUID &internal_uuid) const
{
    return transition_relay_in_->getInput(internal_uuid);
}

OutputPtr SubgraphNode::getForwardedOutputInternal(const UUID &internal_uuid) const
{
    return transition_relay_out_->getOutput(internal_uuid);
}

SlotPtr SubgraphNode::getForwardedSlotInternal(const UUID &internal_uuid) const
{
    return internal_slots_.at(internal_uuid);
}

EventPtr SubgraphNode::getForwardedEventInternal(const UUID &internal_uuid) const
{
    return internal_events_.at(internal_uuid);
}

InputPtr SubgraphNode::getForwardedInputInternalNoThrow(const UUID &internal_uuid) const noexcept
{
    return transition_relay_in_->getInputNoThrow(internal_uuid);
}

OutputPtr SubgraphNode::getForwardedOutputInternalNoThrow(const UUID &internal_uuid) const noexcept
{
    return transition_relay_out_->getOutputNoThrow(internal_uuid);
}

SlotPtr SubgraphNode::getForwardedSlotInternalNoThrow(const UUID &internal_uuid) const noexcept
{
    auto pos = internal_slots_.find(internal_uuid);
    if(pos == internal_slots_.end()) {
        return nullptr;
    }

    return pos->second;
}

EventPtr SubgraphNode::getForwardedEventInternalNoThrow(const UUID &internal_uuid) const noexcept
{
    auto pos = internal_events_.find(internal_uuid);
    if(pos == internal_events_.end()) {
        return nullptr;
    }

    return pos->second;
}

UUID SubgraphNode::getForwardedInputExternal(const UUID &internal_uuid) const
{
    return relay_to_external_output_.at(internal_uuid);
}

UUID SubgraphNode::getForwardedOutputExternal(const UUID &internal_uuid) const
{
    return relay_to_external_input_.at(internal_uuid);
}

UUID SubgraphNode::getForwardedSlotExternal(const UUID &internal_uuid) const
{
    return relay_to_external_slot_.at(internal_uuid);
}

UUID SubgraphNode::getForwardedEventExternal(const UUID &internal_uuid) const
{
    return relay_to_external_event_.at(internal_uuid);
}

std::vector<UUID> SubgraphNode::getInternalOutputs() const
{
    return transition_relay_out_->getOutputs();
}
std::vector<UUID> SubgraphNode::getInternalInputs() const
{
    return transition_relay_in_->getInputs();
}
std::vector<UUID> SubgraphNode::getInternalSlots() const
{
    return internal_slot_ids_;
}
std::vector<UUID> SubgraphNode::getInternalEvents() const
{
    return internal_event_ids_;
}

void SubgraphNode::setIterationEnabled(const UUID& external_input_uuid, bool enabled)
{
    if(enabled) {
        // only one iteration is allowed for now
        apex_assert_eq(0, iterated_inputs_.size());

        iterated_inputs_.insert(external_input_uuid);

        InputPtr i = node_handle_->getInput(external_input_uuid);
        OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

        // change the output type of the subgraph
        TokenDataConstPtr vector_type = i->getType();
        if(vector_type->isContainer()) {
            o->setType(vector_type->nestedType());
        }

        // change the input type of the subgraph
        for(const UUID& id: transition_relay_in_->getInputs()) {
            InputPtr i = transition_relay_in_->getInput(id);

            TokenDataConstPtr type = i->getType();

            original_types_[id] = type;

            if(auto vector = std::dynamic_pointer_cast<const connection_types::GenericVectorMessage>(type)) {
                i->setType(vector->nestedType());
            }
        }

    } else {
        iterated_inputs_.erase(external_input_uuid);

        // change back the input type of the subgraph
        for(const UUID& id: transition_relay_in_->getInputs()) {
            InputPtr i = transition_relay_in_->getInput(id);
            i->setType(original_types_[id]);
        }
    }
}

void SubgraphNode::notifyMessagesProcessed()
{
    //TRACE ainfo << "messages processed" << std::endl;
    //    tryFinishProcessing();
    transition_relay_in_->notifyMessageProcessed();
}

void SubgraphNode::currentIterationIsProcessed()
{
    //TRACE ainfo << "current iteration processed" << std::endl;
    apex_assert(node_handle_);

    tryFinishSubgraph();

    yield();
}

void SubgraphNode::subgraphHasProducedAllMessages()
{
    if(transition_relay_in_->isEnabled()) { // TODO: check this in checkIfEnabled

        apex_assert_hard(!has_sent_current_iteration_);
        sendCurrentIteration();

        tryFinishSubgraph();
    }
}

void SubgraphNode::tryFinishSubgraph()
{
    //TRACE ainfo << "try finish" << std::endl;
    bool can_start_next_iteration = node_handle_->isSink() || has_sent_current_iteration_;
    if(can_start_next_iteration) {
        bool last_iteration = !is_iterating_ || iteration_index_ >= iteration_count_;
        if(last_iteration) {
            //TRACE ainfo << "last iteration -> finish subgraph: " << is_iterating_ << ", " << iteration_index_ << " >= " << iteration_count_ << std::endl;
            finishSubgraph();
        } else {
            if(transition_relay_out_->canStartSendingMessages()) {
                startNextIteration();
            }
        }
    }

}

void SubgraphNode::finishSubgraph()
{
    //TRACE ainfo << "called finish" << std::endl;
    if(is_subgraph_finished_) {
        return;
    }

    // if transition_relay_in is connected, we need to make sure all connections have received...
    if(transition_relay_in_->hasConnection()) {
        if(!transition_relay_in_->areMessagesForwarded()) {
            return;
        }
    }

    //TRACE ainfo << "finish" << std::endl;
    if(!is_subgraph_finished_) {
        is_subgraph_finished_ = true;
        //is_iterating_ = false;
        has_sent_current_iteration_ = false;

        notifySubgraphProcessed();
    }
}


void SubgraphNode::notifySubgraphProcessed()
{
//    if(node_handle_->isSource() && node_handle_->isSink()) {
//        notifyMessagesProcessed();

//    } else {
        std::unique_lock<std::recursive_mutex> lock(continuation_mutex_);
        if(continuation_) {
            auto cnt = continuation_;
            continuation_ = Continuation();
            lock.unlock();

            //TRACEainfo << "continuation" << std::endl;
            cnt([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
        }
//    }
}

void SubgraphNode::sendCurrentIteration()
{
//    ainfo << "send current iteration " << iteration_index_ << " < " << iteration_count_<< std::endl;
    transition_relay_in_->forwardMessages();

    has_sent_current_iteration_ = true;
    if(is_iterating_ && iteration_index_ < iteration_count_) {
        //TRACE ainfo << "mark read" << std::endl;
        transition_relay_in_->notifyMessageRead();
        transition_relay_in_->notifyMessageProcessed();
        // allow another step for all nested nodes...
        // only do this once ALL NODES are done stepping
        for(NodeHandle* nh : graph_->getAllNodeHandles()){
            nh->getNodeRunner()->step();
        }
    }


}

void SubgraphNode::startNextIteration()
{
//    ainfo << "start iteration " << iteration_index_ << std::endl;
    for(const InputPtr& i : node_modifier_->getMessageInputs()) {
        TokenDataConstPtr m = msg::getMessage(i.get());
        OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

        if(m->isContainer() && iterated_inputs_.find(i->getUUID()) != iterated_inputs_.end()) {
            has_sent_current_iteration_ = false;
            msg::publish(o.get(), m->nestedValue(iteration_index_));

        } else {
            msg::publish(o.get(), m);
        }
    }

    ++iteration_index_;
    apex_assert_hard(transition_relay_out_->canStartSendingMessages());
    //TRACE ainfo << "send internal output messages in next iteration" << std::endl;
    transition_relay_out_->sendMessages(node_handle_->isActive());
}

std::string SubgraphNode::makeStatusString() const
{
    std::stringstream ss;
    ss << "UUID: " << getUUID() << '\n';
    ss << "AUUID: " << getUUID().getAbsoluteUUID() << '\n';
    {
        std::unique_lock<std::recursive_mutex> lock(continuation_mutex_);
        ss << "continuation_: " << ((bool) continuation_) << '\n';
    }
    if(node_handle_) {
        ss << "output transiton:\n";
        ss << " - " << (node_handle_->getOutputTransition()->canStartSendingMessages() ? "can send" : "can't send") << '\n';
    }



    auto printStatus = [&ss](const ConnectionPtr& c) {
        switch(c->getState()) {
        case Connection::State::DONE:
            //        case Connection::State::NOT_INITIALIZED:
            ss << "DONE  ";
            break;
        case Connection::State::READ:
            ss << "READ  ";
            break;
        case Connection::State::UNREAD:
            ss << "UNREAD";
            break;
        default:
            ss << "???";
        }
    };

    ss << "(left) transition_relay_out:\n";
    ss << " - " << (transition_relay_out_->isEnabled() ? "enabled" : "disabled") << '\n';
    ss << " - " << (transition_relay_out_->canStartSendingMessages() ? "can" : "can't") << " send\n";
    ss << " - " << (transition_relay_out_->hasConnection() ? "has" : "doesn't have") <<  " established connection\n";
    ss << " - outputs are " << (transition_relay_out_->areOutputsIdle() ? "idle" : "busy") << '\n';
    ss << "(right) transition_relay_in:\n";
    ss << " - " << (transition_relay_in_->isEnabled() ? "enabled" : "disabled") << '\n';
    ss << " - established connections: ";
    for(const ConnectionPtr& c : transition_relay_in_->getConnections()) {
        printStatus(c);
        ss << '\t';
    }
    ss << '\n';

    return ss.str();
}
