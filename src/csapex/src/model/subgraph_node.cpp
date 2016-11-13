/// HEADER
#include <csapex/model/subgraph_node.h>

/// PROJECT
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
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

using namespace csapex;

SubgraphNode::SubgraphNode()
    : transition_relay_in_(new InputTransition),
      transition_relay_out_(new OutputTransition),
      is_subgraph_finished_(false),
      is_iterating_(false), has_sent_current_iteration_(false),
      is_initialized_(false)
{
    transition_relay_in_->setActivationFunction(delegate::Delegate0<>(this, &SubgraphNode::subgraphHasProducedAllMessages));
    transition_relay_out_->messages_processed.connect(delegate::Delegate0<>(this, &SubgraphNode::currentIterationIsProcessed));
}

SubgraphNode::~SubgraphNode()
{
    clear();
}

NodeHandle* SubgraphNode::findNodeHandle(const UUID& uuid) const
{
    if(uuid.empty()) {
        return node_handle_;
    }
    return Graph::findNodeHandle(uuid);
}

NodeHandle *SubgraphNode::findNodeHandleNoThrow(const UUID& uuid) const noexcept
{
    if(uuid.empty()) {
        return node_handle_;
    }
    return Graph::findNodeHandleNoThrow(uuid);
}

ConnectablePtr SubgraphNode::findConnectorNoThrow(const UUID &uuid) noexcept
{
    if(internal_slots_.find(uuid) != internal_slots_.end()) {
        return internal_slots_.at(uuid);
    }
    if(internal_events_.find(uuid) != internal_events_.end()) {
        return internal_events_.at(uuid);
    }

    return Graph::findConnectorNoThrow(uuid);
}

void SubgraphNode::initialize(csapex::NodeHandle* node_handle, const UUID &uuid)
{
    Node::initialize(node_handle, uuid);

    if(node_handle->getUUIDProvider()) {
        setParent(node_handle->getUUIDProvider()->shared_from_this(), node_handle->getUUID().getAbsoluteUUID());
    }
}

void SubgraphNode::reset()
{
    Node::reset();

    resetActivity();

    continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();

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

    activation_event_ = createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), makeUUID("event_activation"), "activation");
    deactivation_event_ = createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), makeUUID("event_deactivation"), "deactivation");
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
        deactivation_event_->trigger();
    }
}

bool SubgraphNode::canProcess() const
{
    if(transition_relay_out_->canStartSendingMessages()) {
        return true;
    } else {
        APEX_DEBUG_TRACE ainfo << "cannot process, out relay cannot send" << std::endl;
        return false;
    }
}

bool SubgraphNode::isDoneProcessing() const
{
    return transition_relay_out_->canStartSendingMessages();
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

void SubgraphNode::process(NodeModifier &node_modifier, Parameterizable &params,
                    std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation)
{
    continuation_ = continuation;

    // can fail...
    apex_assert_hard(transition_relay_out_->areAllConnections(Connection::State::NOT_INITIALIZED));
    apex_assert_hard(transition_relay_in_->areAllConnections(Connection::State::NOT_INITIALIZED));
    apex_assert_hard(transition_relay_out_->canStartSendingMessages());

    is_iterating_ = false;
    has_sent_current_iteration_ = false;
    is_subgraph_finished_ = false;

    for(InputPtr i : node_modifier.getMessageInputs()) {
        TokenDataConstPtr m = msg::getMessage(i.get());
        OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

        if(m->isContainer() && iterated_inputs_.find(i->getUUID()) != iterated_inputs_.end()) {
            is_iterating_ = true;
            iteration_count_ = m->nestedValueCount();
            iteration_index_ = 1;

            msg::publish(o.get(), m->nestedValue(0));

        } else {
            msg::publish(o.get(), m);
        }
    }

    if(transition_relay_out_->hasConnection()) {
        transition_relay_out_->sendMessages(node_handle_->isActive());
    } else {

        finishSubgraph();
    }
}

bool SubgraphNode::isAsynchronous() const
{
    return true;
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
    input->setVirtual(true);

    transition_relay_in_->addInput(input);

    input->connectionInProgress.connect(internalConnectionInProgress);

    return input;
}


void SubgraphNode::removeVariadicInput(InputPtr input)
{
    OutputPtr relay = external_to_internal_outputs_[input->getUUID()];
    forwardingRemoved(relay);

    VariadicInputs::removeVariadicInput(input);

    relay_to_external_input_.erase(relay->getUUID());

    external_to_internal_outputs_.erase(input->getUUID());
    transition_relay_out_->removeOutput(relay);
}

RelayMapping SubgraphNode::addForwardingInput(const TokenDataConstPtr& type,
                                       const std::string& label, bool optional)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayout");
    UUID external_uuid = addForwardingInput(internal_uuid, type, label, optional);

    return {external_uuid, internal_uuid};
}

UUID  SubgraphNode::addForwardingInput(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label, bool optional)
{
    registerUUID(internal_uuid);

    Input* external_input = VariadicInputs::createVariadicInput(type, label, optional);

    OutputPtr relay = createInternalOutput(type, internal_uuid, label);

    crossConnectLabelChange(external_input, relay.get());

    external_to_internal_outputs_[external_input->getUUID()] = relay;

    relay_to_external_input_[internal_uuid] = external_input->getUUID();

    forwardingAdded(relay);

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
    output->setVirtual(true);

    transition_relay_out_->addOutput(output);

    output->connectionInProgress.connect(internalConnectionInProgress);

    return output;
}

void SubgraphNode::removeVariadicOutput(OutputPtr output)
{
    InputPtr relay = external_to_internal_inputs_[output->getUUID()];
    forwardingRemoved(relay);

    relay->message_set.disconnectAll();

    VariadicOutputs::removeVariadicOutput(output);

    relay_to_external_output_.erase(relay->getUUID());

    external_to_internal_inputs_.erase(output->getUUID());
    transition_relay_in_->removeInput(relay);
}

RelayMapping SubgraphNode::addForwardingOutput(const TokenDataConstPtr& type,
                                        const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayin");
    UUID external_uuid = addForwardingOutput(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID SubgraphNode::addForwardingOutput(const UUID& internal_uuid, const TokenDataConstPtr& type,  const std::string& label)
{
    registerUUID(internal_uuid);

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

    forwardingAdded(relay);

    return external_output->getUUID();
}



SlotPtr SubgraphNode::createInternalSlot(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, std::function<void (const TokenPtr& )> callback)
{
    SlotPtr slot = node_handle_->addInternalSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), internal_uuid, label, callback);
    slot->setVirtual(true);

    slot->connectionInProgress.connect(internalConnectionInProgress);

    internal_slots_[internal_uuid] = slot;

    return slot;
}

Slot* SubgraphNode::createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr&)> /*callback*/, bool /*active*/, bool /*asynchronous*/)
{
    auto pair = addForwardingSlot(type, label);
    return node_handle_->getSlot(pair.external).get();
}

void SubgraphNode::removeVariadicSlot(SlotPtr slot)
{
    EventPtr relay = external_to_internal_events_.at(slot->getUUID());
    external_to_internal_events_.erase(slot->getUUID());

    internal_events_.erase(relay->getUUID());

    forwardingRemoved(relay);

    VariadicSlots::removeVariadicSlot(slot);

    relay_to_external_slot_.erase(relay->getUUID());
}

RelayMapping SubgraphNode::addForwardingSlot(const TokenDataConstPtr& type, const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayevent");
    UUID external_uuid = addForwardingSlot(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID SubgraphNode::addForwardingSlot(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label)
{
    registerUUID(internal_uuid);

    EventPtr relay = createInternalEvent(type, internal_uuid, label);

    auto cb = [relay](const TokenConstPtr& data) {
        relay->triggerWith(std::make_shared<Token>(*data));
        relay->message_processed(relay.get());
    };

    Slot* external_slot = VariadicSlots::createVariadicSlot(type, label, cb, false, true);

    relay->message_processed.connect(std::bind(&Slot::notifyEventHandled, external_slot));

    crossConnectLabelChange(external_slot, relay.get());

    external_to_internal_events_[external_slot->getUUID()] = relay;

    relay_to_external_slot_[internal_uuid] = external_slot->getUUID();

    forwardingAdded(relay);

    return external_slot->getUUID();
}

EventPtr SubgraphNode::createInternalEvent(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    EventPtr event = node_handle_->addInternalEvent(type, internal_uuid, label);
    event->setVirtual(true);

    event->connectionInProgress.connect(internalConnectionInProgress);

    internal_events_[internal_uuid] = event;

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

    forwardingRemoved(relay);

    VariadicEvents::removeVariadicEvent(event);

    relay_to_external_event_.erase(relay->getUUID());
}

RelayMapping SubgraphNode::addForwardingEvent(const TokenDataConstPtr& type, const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayslot");
    UUID external_uuid = addForwardingEvent(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID SubgraphNode::addForwardingEvent(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label)
{
    registerUUID(internal_uuid);

    Event* external_event = VariadicEvents::createVariadicEvent(type, label);

    auto cb = [this, external_event](const TokenPtr& token){
        external_event->triggerWith(token);
        node_handle_->getNodeWorker()->trySendEvents();
    };

    SlotPtr relay = createInternalSlot(type, internal_uuid, label, cb);
    external_event->message_processed.connect(std::bind(&Slot::notifyEventHandled, relay.get()));

    crossConnectLabelChange(external_event, relay.get());

    external_to_internal_slots_[external_event->getUUID()] = relay;

    relay_to_external_event_[internal_uuid] = external_event->getUUID();

    forwardingAdded(relay);

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
    std::vector<UUID> res;
    for(const auto& pair : internal_slots_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}
std::vector<UUID> SubgraphNode::getInternalEvents() const
{
    std::vector<UUID> res;
    for(const auto& pair : internal_events_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}

void SubgraphNode::setIterationEnabled(const UUID& external_input_uuid, bool enabled)
{
    if(enabled) {
        iterated_inputs_.insert(external_input_uuid);
        InputPtr i = node_handle_->getInput(external_input_uuid);
        OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

        TokenDataConstPtr vector_type = i->getType();
        if(vector_type->isContainer()) {
            o->setType(vector_type->nestedType());
        }

    } else {
        iterated_inputs_.erase(external_input_uuid);
    }
}

void SubgraphNode::removeInternalPorts()
{
    node_handle_->removeInternalPorts();
    internal_events_.clear();
    internal_slots_.clear();
}

void SubgraphNode::notifyMessagesProcessed()
{
    GeneratorNode::notifyMessagesProcessed();

    //    tryFinishProcessing();
    APEX_DEBUG_TRACE ainfo << "is notified" << std::endl;
    transition_relay_in_->notifyMessageProcessed();
}

void SubgraphNode::currentIterationIsProcessed()
{
    APEX_DEBUG_TRACE ainfo << "input activated" << std::endl;

    if(!is_subgraph_finished_) {
        tryFinishSubgraph();
    }
    finished();
}

void SubgraphNode::subgraphHasProducedAllMessages()
{
    if(transition_relay_in_->isEnabled()) { // TODO: check this in checkIfEnabled
        APEX_DEBUG_TRACE ainfo << "output activated" << std::endl;

        apex_assert_hard(!has_sent_current_iteration_);
        sendCurrentIteration();

        tryFinishSubgraph();
    }
}

void SubgraphNode::tryFinishSubgraph()
{
    bool can_start_next_iteration = node_handle_->isSink() || has_sent_current_iteration_;
    if(can_start_next_iteration) {
        bool last_iteration = !is_iterating_ || iteration_index_ >= iteration_count_;
        if(last_iteration) {
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
    is_subgraph_finished_ = true;
    is_iterating_ = false;
    has_sent_current_iteration_ = false;

    if(node_handle_->isSource()) {
        notifySubgraphHasProducedTokens();
    } else {
        notifySubgraphProcessed();
    }
}


void SubgraphNode::notifySubgraphProcessed()
{
    if(continuation_) {
        continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
        continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();
    }
}

void SubgraphNode::notifySubgraphHasProducedTokens()
{
    if(node_handle_->isSource()) {
        updated();
    }
}

void SubgraphNode::sendCurrentIteration()
{
    apex_assert_hard(transition_relay_in_->isEnabled());
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

    APEX_DEBUG_TRACE ainfo << "forward_messages" << std::endl;
    transition_relay_in_->forwardMessages();

    has_sent_current_iteration_ = true;
    if(is_iterating_) {
        transition_relay_in_->notifyMessageRead();
        transition_relay_in_->notifyMessageProcessed();
    }
}

void SubgraphNode::startNextIteration()
{
    for(InputPtr i : node_modifier_->getMessageInputs()) {
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
    transition_relay_out_->sendMessages(node_handle_->isActive());
}

std::string SubgraphNode::makeStatusString() const
{
    std::stringstream ss;
    ss << "UUID: " << getUUID() << '\n';
    ss << "AUUID: " << getUUID().getAbsoluteUUID() << '\n';
    ss << "continuation_: " << ((bool) continuation_) << '\n';
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
