/// HEADER
#include <csapex/model/node_handle.h>

/// COMPONENT
#include <csapex/model/connectable.h>
#include <csapex/model/generic_state.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_state.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/io.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/msg/marker_message.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/static_output.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/exceptions.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeHandle::NodeHandle(const std::string &type, const UUID& uuid, NodePtr node,
                       UUIDProviderPtr uuid_provider,
                       InputTransitionPtr transition_in, OutputTransitionPtr transition_out)
    : ConnectableOwner(uuid),
      node_(node),
      node_state_(std::make_shared<NodeState>(this)),
      node_type_(type),
      
      transition_in_(transition_in),
      transition_out_(transition_out),

      uuid_provider_(uuid_provider),

      guard_(-1)
{
    node_state_->setLabel(uuid.getFullName());
    node_state_->setParent(this);
    
    node_state_->enabled_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->active_changed->connect([this](){
        activation_changed();
        if(isActive()) {
            node_->activation();
        } else {
            node_->deactivation();
        }
    });
    node_state_->flipped_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->label_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->minimized_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->parent_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->pos_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->thread_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->color_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    
    node_state_->label_changed->connect([this]() {
        std::string label = node_state_->getLabel();
        if(label.empty()) {
            label = getUUID().getShortName();
        }
        
        label = getUUID().getAbsoluteUUID().getFullName();
        node_->adebug.setPrefix(label);
        node_->ainfo.setPrefix(label);
        node_->awarn.setPrefix(label);
        node_->aerr.setPrefix(label);

        triggerNodeStateChanged();
    });
    node_state_->logger_level_changed->connect([this](){
        updateLoggerLevel();
    });
    node_state_->muted_changed->connect([this](){
        updateLoggerLevel();
    });
    
    //    triggerNodeStateChanged();
    
    node_->parameters_changed.connect(parameters_changed);
    node_->getParameterState()->parameter_set_changed->connect(parameters_changed);
    node_->getParameterState()->parameter_added->connect(std::bind(&NodeHandle::makeParameterConnectable, this, std::placeholders::_1));
    node_->getParameterState()->parameter_removed->connect(std::bind(&NodeHandle::makeParameterNotConnectable, this, std::placeholders::_1));
}

NodeHandle::~NodeHandle()
{
    while(!external_inputs_.empty()) {
        removeInput(external_inputs_.begin()->get());
    }
    while(!external_outputs_.empty()) {
        removeOutput(external_outputs_.begin()->get());
    }
    while(!external_slots_.empty()) {
        removeSlot(external_slots_.begin()->get());
    }
    while(!external_events_.empty()) {
        removeEvent(external_events_.begin()->get());
    }

    node_removed();

    node_.reset();

    guard_ = 0xDEADBEEF;
}

void NodeHandle::updateLoggerLevel()
{
    int level = node_state_->getLoggerLevel();
    bool enabled = !node_state_->isMuted();
    if(!enabled) {
        level = 10;
    }

    node_->adebug.setEnabled(level <= 0);
    node_->ainfo.setEnabled(level <= 1);
    node_->awarn.setEnabled(level <= 2);
    node_->aerr.setEnabled(level <= 3);

    triggerNodeStateChanged();
}

bool NodeHandle::isIsolated() const
{
    return node_->isIsolated();
}

bool NodeHandle::isSource() const
{
    for(InputPtr in : external_inputs_) {
        if(!in->isOptional() || in->isConnected()) {
            return false;
        }
    }
    
    return true;
}


bool NodeHandle::isSink() const
{
    return external_outputs_.empty() || !hasConnectionsOutgoing();
}

bool NodeHandle::hasConnectionsIncoming() const
{
    return transition_in_->hasConnection();
}
bool NodeHandle::hasConnectionsOutgoing() const
{
    return transition_out_->hasConnection();
}

bool NodeHandle::isVariadic() const
{
    return dynamic_cast<VariadicBase*>(node_.get());
}
bool NodeHandle::hasVariadicInputs() const
{
    return dynamic_cast<VariadicInputs*>(node_.get());
}
bool NodeHandle::hasVariadicOutputs() const
{
    return dynamic_cast<VariadicOutputs*>(node_.get());
}
bool NodeHandle::hasVariadicEvents() const
{
    return dynamic_cast<VariadicEvents*>(node_.get());
}
bool NodeHandle::hasVariadicSlots() const
{
    return dynamic_cast<VariadicSlots*>(node_.get());
}

void NodeHandle::setActive(bool active)
{
    node_state_->setActive(active);
}

bool NodeHandle::isActive() const
{
    return node_state_->isActive();
}


void NodeHandle::stop()
{
    if(node_) {
        node_->tearDown();
    }

    stopped();

    if(node_) {
        node_->reset();
    }

    for(OutputPtr i : getExternalOutputs()) {
        i->stop();
    }
    for(InputPtr i : getExternalInputs()) {
        i->stop();
    }

    if(node_) {
        node_->detach();
        node_.reset();
    }
}

std::string NodeHandle::getType() const
{
    return node_type_;
}

NodeWeakPtr NodeHandle::getNode() const
{
    return node_;
}

void NodeHandle::setVertex(graph::VertexWeakPtr vertex)
{
    apex_assert_hard(guard_ == -1);
    vertex_ = vertex;
}

graph::VertexPtr NodeHandle::getVertex() const
{
    apex_assert_hard(guard_ == -1);
    return vertex_.lock();
}

InputTransition* NodeHandle::getInputTransition() const
{
    return transition_in_.get();
}

OutputTransition* NodeHandle::getOutputTransition() const
{
    return transition_out_.get();
}


void NodeHandle::setNodeState(NodeStatePtr memento)
{
    std::string old_label = node_state_->getLabel();
    
    *node_state_ = *memento;
    
    
    if(memento->getParameterState()) {
        node_->setParameterState(memento->getParameterState());
    }
    
    if(node_state_->getLabel().empty()) {
        if(old_label.empty()) {
            node_state_->setLabel(getUUID().getShortName());
        } else {
            node_state_->setLabel(old_label);
        }
    }
    
    triggerNodeStateChanged();
}

void NodeHandle::triggerNodeStateChanged()
{
    node_state_changed();
    node_->stateChanged();
}

NodeState::Ptr NodeHandle::getNodeStateCopy() const
{
    apex_assert_hard(node_state_);
    
    NodeState::Ptr memento(new NodeState(this));
    *memento = *node_state_;
    
    memento->setParameterState(node_->getParameterStateClone());
    
    return memento;
}

NodeState::Ptr NodeHandle::getNodeState()
{
    apex_assert_hard(node_state_);
    
    return node_state_;
}


template <typename T>
void NodeHandle::makeParameterConnectableImpl(csapex::param::ParameterPtr param)
{
    makeParameterConnectableTyped<connection_types::GenericValueMessage<T>>(param);
}

template <typename T>
void NodeHandle::makeParameterConnectableTyped(csapex::param::ParameterPtr param)
{
    apex_assert_hard(uuid_provider_);
    csapex::param::Parameter* p = param.get();
    
    auto pos = param_2_input_.find(p->name());
    if(pos != param_2_input_.end() && pos->second.lock()) {
        return;
    }
    
    
    {
        InputPtr cin = std::make_shared<Input>(uuid_provider_->makeDerivedUUID(getUUID(), std::string("in_") + p->name()), shared_from_this());
        cin->setType(connection_types::makeEmpty<T>());
        cin->setOptional(true);
        cin->setLabel(p->name());
        cin->setParameter(true);
        
        param_2_input_[p->name()] = cin;
        input_2_param_[cin->getUUID()] = p;
        
        manageInput(cin);
    }
    {
        OutputPtr cout = std::make_shared<StaticOutput>(uuid_provider_->makeDerivedUUID(getUUID(), std::string("out_") + p->name()), shared_from_this());
        cout->setType(connection_types::makeEmpty<T>());
        cout->setLabel(p->name());
        cout->setParameter(true);
        
        param_2_output_[p->name()] = cout;
        output_2_param_[cout->getUUID()] = p;
        
        manageOutput(cout);
    }
}

void NodeHandle::makeParameterConnectable(csapex::param::ParameterPtr p)
{
    if(!uuid_provider_) {
        return;
    }
    
    if(p->isTemporary()) {
        return;
    }
    
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
    } else {
        makeParameterConnectableTyped<connection_types::AnyMessage>(p);
    }
    
    param::TriggerParameterPtr t = std::dynamic_pointer_cast<param::TriggerParameter>(p);
    if(t) {
        Event* trigger = NodeModifier::addEvent(t->name());
        Slot* slot = NodeModifier::addSlot(t->name(), [t]() {
            t->trigger();
        }, false);
        node_->addParameterCallback(t, [slot, trigger](param::Parameter*) {
            auto token = slot->getToken();
            if(token) {
                trigger->triggerWith(token);
            } else {
                msg::trigger(trigger);
            }
        });

        param::TriggerParameterWeakPtr t_weak = t;
        slot->connection_added.connect([this, slot, t_weak](ConnectionPtr) {
            if(slot->getConnections().size() == 1) {
                if(param::TriggerParameterPtr t = t_weak.lock()) {
                    t->first_connect(t.get());
                }
            }
        });
        slot->connection_faded.connect([this, slot, t_weak](ConnectionPtr) {
            if(slot->getConnections().empty()) {
                if(param::TriggerParameterPtr t = t_weak.lock()) {
                    t->last_disconnect(t.get());
                }
            }
        });
    }
}

void NodeHandle::makeParameterNotConnectable(csapex::param::ParameterPtr p)
{
    auto pin = param_2_input_.find(p->name());
    auto pout = param_2_output_.find(p->name());
    
    if(pin == param_2_input_.end() || pout == param_2_output_.end()) {
        return;
    }
    
    InputPtr cin_ptr = pin->second.lock();
    OutputPtr cout_ptr = pout->second.lock();
    
    Input* cin = cin_ptr.get();
    Output* cout = cout_ptr.get();
    
    if(!cin || !cout) {
        return;
    }
    
    disconnectConnector(cin);
    disconnectConnector(cout);
    
    removeInput(cin);
    removeOutput(cout);
    
    apex_assert_hard(param_2_input_.erase(p->name()) != 0);
    apex_assert_hard(input_2_param_.erase(cin->getUUID()) != 0);
    
    apex_assert_hard(param_2_output_.erase(p->name()) != 0);
    apex_assert_hard(output_2_param_.erase(cout->getUUID()) != 0);
}

bool NodeHandle::updateParameterValues()
{
    bool change = false;
    for(auto pair : param_2_input_) {
        InputPtr cin = pair.second.lock();
        if(cin) {
            apex_assert_hard(cin->isOptional());
            if(msg::hasMessage(cin.get())) {
                updateParameterValue(cin.get());
                change = true;
            }
        }
    }

    return change;
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

void NodeHandle::updateParameterValue(Connectable *s)
{
    Input* source = dynamic_cast<Input*> (s);
    apex_assert_hard(source);
    
    csapex::param::Parameter* p = input_2_param_.at(source->getUUID());

    if(msg::isExactValue<int>(source)) {
        updateParameterValueFrom<int>(p, source);
    } else if(msg::isExactValue<bool>(source)) {
        updateParameterValueFrom<bool>(p, source);
    } else if(msg::isExactValue<double>(source)) {
        updateParameterValueFrom<double>(p, source);
    } else if(msg::isExactValue<std::string>(source)) {
        updateParameterValueFrom<std::string>(p, source);
    } else if(msg::isExactValue<std::pair<int,int>>(source)) {
        updateParameterValueFrom<std::pair<int, int>>(p, source);
    } else if(msg::isExactValue<std::pair<double,double>>(source)) {
        updateParameterValueFrom<std::pair<double, double>>(p, source);
    } else if(msg::hasMessage(source) && !msg::isMessage<connection_types::MarkerMessage>(source)) {
        node_->ainfo << "parameter " << p->name() << " got a message of unsupported type" << std::endl;
    }
}

Input* NodeHandle::addInput(TokenDataConstPtr type, const std::string& label, bool optional)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "in");
    InputPtr c = std::make_shared<Input>(uuid, shared_from_this());

    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);
    c->setVariadic(variadic_);
    
    manageInput(c);
    
    return c.get();
}

Output* NodeHandle::addOutput(TokenDataConstPtr type, const std::string& label)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "out");
    OutputPtr c = std::make_shared<StaticOutput>(uuid, shared_from_this());

    c->setLabel(label);
    c->setType(type);
    c->setVariadic(variadic_);
    
    manageOutput(c);
    return c.get();
}

Slot* NodeHandle::addSlot(TokenDataConstPtr type, const std::string& label, std::function<void()> callback, bool active, bool blocking)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "slot");
    SlotPtr slot = std::make_shared<Slot>(callback, uuid, active, blocking, shared_from_this());
    slot->setLabel(label);
    slot->setType(type);
    slot->setVariadic(variadic_);

    manageSlot(slot);

    return slot.get();
}

Slot* NodeHandle::addSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr& )> callback, bool active, bool blocking)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "slot");
    SlotPtr slot = std::make_shared<Slot>(callback, uuid, active, blocking, shared_from_this());
    slot->setLabel(label);
    slot->setType(type);
    slot->setVariadic(variadic_);

    manageSlot(slot);

    return slot.get();
}

Slot* NodeHandle::addSlot(TokenDataConstPtr type, const std::string& label, std::function<void(Slot*, const TokenPtr& )> callback, bool active, bool blocking)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "slot");
    SlotPtr slot = std::make_shared<Slot>(callback, uuid, active, blocking, shared_from_this());
    slot->setLabel(label);
    slot->setType(type);

    manageSlot(slot);

    return slot.get();
}

Event* NodeHandle::addEvent(TokenDataConstPtr type, const std::string& label)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "event");
    EventPtr event = std::make_shared<Event>(uuid, shared_from_this());
    event->setLabel(label);
    event->setType(type);
    
    manageEvent(event);
    
    return event.get();
}

InputPtr NodeHandle::addInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool optional)
{
    InputPtr in = std::make_shared<Input>(internal_uuid, shared_from_this());
    in->setType(type);
    in->setLabel(label);
    in->setOptional(optional);
    
    internal_inputs_.push_back(in);

    connector_created(in, true);
    
    return in;
}

OutputPtr NodeHandle::addInternalOutput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label)
{
    OutputPtr out = std::make_shared<StaticOutput>(internal_uuid, shared_from_this());
    out->setType(type);
    out->setLabel(label);
    
    internal_outputs_.push_back(out);

    connector_created(out, true);

    return out;
}

SlotPtr NodeHandle::addInternalSlot(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string &label, std::function<void (const TokenPtr &)> callback)
{
    apex_assert_hard(uuid_provider_);
    SlotPtr slot = std::make_shared<Slot>(callback, internal_uuid, false, false, shared_from_this());
    slot->setLabel(label);
    slot->setType(type);
    
    internal_slots_.push_back(slot);
    
    connectConnector(slot.get());
    
    connector_created(slot, true);
    
    return slot;
}

EventPtr NodeHandle::addInternalEvent(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    apex_assert_hard(uuid_provider_);
    EventPtr event = std::make_shared<Event>(internal_uuid, shared_from_this());
    event->setLabel(label);
    event->setType(type);
    
    internal_events_.push_back(event);
    
    connectConnector(event.get());
    
    connector_created(event, true);
    
    return event;
}

InputWeakPtr NodeHandle::getParameterInput(const std::string &name) const
{
    auto it = param_2_input_.find(name);
    if(it == param_2_input_.end()) {
        return std::weak_ptr<Input>();
    } else {
        return it->second;
    }
}

OutputWeakPtr NodeHandle::getParameterOutput(const std::string &name) const
{
    auto it = param_2_output_.find(name);
    if(it == param_2_output_.end()) {
        return std::weak_ptr<Output>();
    } else {
        return it->second;
    }
}


void NodeHandle::removeInput(Input* in)
{
    std::vector<InputPtr>::iterator it;
    for(it = external_inputs_.begin(); it != external_inputs_.end(); ++it) {
        if(it->get() == in) {
            break;
        }
    }
    
    if(it != external_inputs_.end()) {
        InputPtr input = *it;
        transition_in_->removeInput(input);
        
        external_inputs_.erase(it);
        
        disconnectConnector(input.get());
        connector_removed(input, false);
        
    } else {
        std::cerr << "ERROR: cannot remove input " << in->getUUID().getFullName() << std::endl;
    }
}

void NodeHandle::removeOutput(Output* out)
{
    std::vector<OutputPtr>::iterator it;
    for(it = external_outputs_.begin(); it != external_outputs_.end(); ++it) {
        if(it->get() == out) {
            break;
        }
    }
    
    if(it != external_outputs_.end()) {
        OutputPtr output = *it;
        transition_out_->removeOutput(output);
        
        external_outputs_.erase(it);
        
        disconnectConnector(output.get());
        connector_removed(output, false);

    } else {
        std::cerr << "ERROR: cannot remove output " << out->getUUID().getFullName() << std::endl;
    }
    
}

void NodeHandle::removeSlot(Slot* s)
{
    std::vector<SlotPtr>::iterator it;
    for(it = external_slots_.begin(); it != external_slots_.end(); ++it) {
        if(it->get() == s) {
            break;
        }
    }
    
    
    if(it != external_slots_.end()) {
        SlotPtr slot = *it;
        
        external_slots_.erase(it);
        
        disconnectConnector(slot.get());
        connector_removed(slot, false);
    }
    
}

void NodeHandle::removeEvent(Event* t)
{
    std::vector<EventPtr>::iterator it;
    for(it = external_events_.begin(); it != external_events_.end(); ++it) {
        if(it->get() == t) {
            break;
        }
    }
    
    if(it != external_events_.end()) {
        EventPtr trigger = *it;
        
        external_events_.erase(it);
        
        disconnectConnector(trigger.get());
        connector_removed(trigger, false);
    }
}

void NodeHandle::manageInput(InputPtr in)
{
    if(!getUUID().empty()) {
        apex_assert_hard(in->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_inputs_.push_back(in);
    
    connectConnector(in.get());

    connections_[in.get()].emplace_back(in->message_available.connect([this](Connection*) { might_be_enabled(); }));
    
    connector_created(in, false);
    transition_in_->addInput(in);
}

void NodeHandle::manageOutput(OutputPtr out)
{
    if(!getUUID().empty()) {
        apex_assert_hard(out->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_outputs_.push_back(out);
    
    connectConnector(out.get());
    
    connections_[out.get()].emplace_back(out->message_processed.connect([this](const ConnectorPtr&) { might_be_enabled(); }));
    connections_[out.get()].emplace_back(out->connection_removed_to.connect([this](const ConnectorPtr&) { might_be_enabled(); }));
    connections_[out.get()].emplace_back(out->connection_added_to.connect([this](const ConnectorPtr&) { might_be_enabled(); }));
    connections_[out.get()].emplace_back(out->connectionEnabled.connect([this](bool) { might_be_enabled(); }));
    
    connector_created(out, false);
    transition_out_->addOutput(out);
}

bool NodeHandle::isParameterInput(const UUID &id) const
{
    return input_2_param_.find(id) != input_2_param_.end();
}

bool NodeHandle::isParameterOutput(const UUID &id) const
{
    return output_2_param_.find(id) != output_2_param_.end();
}

void NodeHandle::manageSlot(SlotPtr s)
{
    if(!getUUID().empty()) {
        apex_assert_hard(s->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_slots_.push_back(s);
    
    connectConnector(s.get());
    
    connector_created(s, false);
}

void NodeHandle::manageEvent(EventPtr t)
{
    if(!getUUID().empty()) {
        apex_assert_hard(t->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_events_.push_back(t);
    
    connectConnector(t.get());
    
    connector_created(t, false);
}

ConnectablePtr NodeHandle::getConnector(const UUID &uuid) const
{
    ConnectablePtr res = getConnectorNoThrow(uuid);
    if(!res) {
        throw std::logic_error(std::string("the connector '") + uuid.getFullName() + "' is unknown.");
    }
    return res;
}

ConnectablePtr NodeHandle::getConnectorNoThrow(const UUID &uuid) const noexcept
{
    std::string type = uuid.type();

    if(type == "in" || type == "relayin") {
        return getInput(uuid);
    } else if(type == "out" || type == "relayout") {
        return getOutput(uuid);
    } else if(type == "slot" || type == "relayslot") {
        return getSlot(uuid);
    } else if(type == "event" || type == "relayevent") {
        return getEvent(uuid);
    } else {
        return nullptr;
    }
}

InputPtr NodeHandle::getInput(const UUID& uuid) const noexcept
{
    for(InputPtr in : external_inputs_) {
        if(in->getUUID() == uuid) {
            return in;
        }
    }
    
    if(SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(node_)) {
        return graph->getForwardedInputInternalNoThrow(uuid.id());
    }
    
    return nullptr;
}

OutputPtr NodeHandle::getOutput(const UUID& uuid) const noexcept
{
    for(OutputPtr out : external_outputs_) {
        if(out->getUUID() == uuid) {
            return out;
        }
    }

    if(SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(node_)) {
        return graph->getForwardedOutputInternalNoThrow(uuid.id());
    }
    
    return nullptr;
}


SlotPtr NodeHandle::getSlot(const UUID& uuid) const noexcept
{
    for(SlotPtr s : external_slots_) {
        if(s->getUUID() == uuid) {
            return s;
        }
    }

    if(SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(node_)) {
        return graph->getForwardedSlotInternalNoThrow(uuid.id());
    }
    
    return nullptr;
}


EventPtr NodeHandle::getEvent(const UUID& uuid) const noexcept
{
    for(EventPtr t : external_events_) {
        if(t->getUUID() == uuid) {
            return t;
        }
    }

    if(SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(node_)) {
        return graph->getForwardedEventInternalNoThrow(uuid.id());
    }
    
    return nullptr;
}

void NodeHandle::removeInput(const UUID &uuid)
{
    removeInput(getInput(uuid).get());
}

void NodeHandle::removeOutput(const UUID &uuid)
{
    removeOutput(getOutput(uuid).get());
}

void NodeHandle::removeSlot(const UUID &uuid)
{
    removeSlot(getSlot(uuid).get());
}

void NodeHandle::removeEvent(const UUID &uuid)
{
    removeEvent(getEvent(uuid).get());
}

std::vector<ConnectablePtr> NodeHandle::getExternalConnectors() const
{
    std::size_t n = external_inputs_.size();
    n += external_outputs_.size();
    n += external_events_.size() + external_slots_.size();
    std::vector<ConnectablePtr> result(n, nullptr);
    std::size_t pos = 0;
    for(auto i : external_inputs_) {
        result[pos++] = i;
    }
    for(auto i : external_outputs_) {
        result[pos++] = i;
    }
    for(auto i : external_events_) {
        result[pos++] = i;
    }
    for(auto i : external_slots_) {
        result[pos++] = i;
    }
    return result;
}

std::vector<ConnectorDescription> NodeHandle::getExternalInputDescriptions() const
{
    return external_inputs_.getDescription();
}

std::vector<ConnectorDescription> NodeHandle::getInternalInputDescriptions() const
{
    return internal_inputs_.getDescription();
}

std::vector<InputPtr> NodeHandle::getExternalInputs() const
{
    return external_inputs_;
}
std::vector<InputPtr> NodeHandle::getInternalInputs() const
{
    return internal_inputs_;
}



std::vector<ConnectorDescription> NodeHandle::getExternalOutputDescriptions() const
{
    return external_outputs_.getDescription();
}

std::vector<ConnectorDescription> NodeHandle::getInternalOutputDescriptions() const
{
    return internal_outputs_.getDescription();
}

std::vector<OutputPtr> NodeHandle::getExternalOutputs() const
{
    return external_outputs_;
}
std::vector<OutputPtr> NodeHandle::getInternalOutputs() const
{
    return internal_outputs_;
}



std::vector<ConnectorDescription> NodeHandle::getExternalSlotDescriptions() const
{
    return external_slots_.getDescription();
}

std::vector<ConnectorDescription> NodeHandle::getInternalSlotDescriptions() const
{
    return internal_slots_.getDescription();
}

std::vector<SlotPtr> NodeHandle::getExternalSlots() const
{
    return external_slots_;
}
std::vector<SlotPtr> NodeHandle::getInternalSlots() const
{
    return internal_slots_;
}



std::vector<ConnectorDescription> NodeHandle::getExternalEventDescriptions() const
{
    return external_events_.getDescription();
}

std::vector<ConnectorDescription> NodeHandle::getInternalEventDescriptions() const
{
    return internal_events_.getDescription();
}

std::vector<EventPtr> NodeHandle::getExternalEvents() const
{
    return external_events_;
}
std::vector<EventPtr> NodeHandle::getInternalEvents() const
{
    return internal_events_;
}


std::map<std::string, InputWeakPtr>& NodeHandle::paramToInputMap()
{
    return param_2_input_;
}

std::map<std::string, OutputWeakPtr>& NodeHandle::paramToOutputMap()
{
    return param_2_output_;
}

std::unordered_map<UUID,csapex::param::Parameter*,UUID::Hasher>& NodeHandle::inputToParamMap()
{
    return input_2_param_;
}

std::unordered_map<UUID,csapex::param::Parameter*,UUID::Hasher>& NodeHandle::outputToParamMap()
{
    return output_2_param_;
}


void NodeHandle::connectConnector(Connectable *c)
{
    connections_[c].emplace_back(c->connectionStart.connect(connection_start));
    connections_[c].emplace_back(c->connection_added_to.connect(connection_added));
    connections_[c].emplace_back(c->connection_removed_to.connect(connection_removed));
}


void NodeHandle::disconnectConnector(Connectable* c)
{
    for(auto& connection : connections_[c]) {
        connection.disconnect();
    }
    connections_[c].clear();
}

AUUID NodeHandle::getSubgraphAUUID() const
{
    SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(node_);
    apex_assert_hard(graph);
    return graph->getGraph()->getAbsoluteUUID();
}

UUIDProvider* NodeHandle::getUUIDProvider()
{
    return uuid_provider_.get();
}

bool NodeHandle::isGraph() const
{
    return node_type_ == "csapex::Graph";
}

Rate& NodeHandle::getRate()
{
    return rate_;
}
const Rate& NodeHandle::getRate() const
{
    return rate_;
}

void NodeHandle::setNodeRunner(NodeRunnerWeakPtr runner)
{
    node_runner_ = runner;
}

NodeRunnerPtr NodeHandle::getNodeRunner() const
{
    return node_runner_.lock();
}
