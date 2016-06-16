/// HEADER
#include <csapex/model/node_handle.h>

/// COMPONENT
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node.h>
#include <csapex/model/graph.h>
#include <csapex/msg/input.h>
#include <csapex/msg/io.h>
#include <csapex/msg/static_output.h>
#include <csapex/param/trigger_parameter.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/msg/marker_message.h>
#include <csapex/utility/uuid_provider.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

NodeHandle::NodeHandle(const std::string &type, const UUID& uuid, NodePtr node,
                       UUIDProvider *uuid_provider,
                       InputTransitionPtr transition_in, OutputTransitionPtr transition_out)
    : Unique(uuid),
      node_(node),
      node_state_(std::make_shared<NodeState>(this)),
      node_type_(type),
      
      transition_in_(transition_in),
      transition_out_(transition_out),
      
      uuid_provider_(uuid_provider),
      level_(0),
      source_(false), sink_(false)
{
    node_->initialize(this, uuid);
    
    node_state_->setLabel(uuid.getFullName());
    node_state_->setParent(this);
    
    node_state_->enabled_changed->connect(std::bind(&NodeHandle::triggerNodeStateChanged, this));
    node_state_->active_changed->connect([this](){
        activationChanged();
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
        
        node_->adebug.setPrefix(label);
        node_->ainfo.setPrefix(label);
        node_->awarn.setPrefix(label);
        node_->aerr.setPrefix(label);
    });
    
    //    triggerNodeStateChanged();
    
    node_->parameters_changed.connect(parametersChanged);
    node_->getParameterState()->parameter_set_changed->connect(parametersChanged);
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
    
    nodeRemoved();
}

int NodeHandle::getLevel() const
{
    return level_;
}

void NodeHandle::setLevel(int level)
{
    level_ = level;
    for(InputPtr in : external_inputs_) {
        in->setLevel(level);
    }
    for(OutputPtr out : external_outputs_) {
        out->setLevel(level);
    }
}

void NodeHandle::setIsSource(bool source)
{
    source_ = source;
}

bool NodeHandle::isSource() const
{
    if(source_) {
        return true;
    }
    
    // check if there are no (mandatory) inputs -> then it's a virtual source
    // TODO: remove and refactor old plugins
    for(InputPtr in : external_inputs_) {
        if(!in->isOptional() || in->isConnected()) {
            return false;
        }
    }
    
    return true;
}


void NodeHandle::setIsSink(bool sink)
{
    sink_ = sink;
}

bool NodeHandle::isSink() const
{
    return sink_ || external_outputs_.empty() || transition_out_->isSink();
}

void NodeHandle::setActive(bool active)
{
    if(active != node_state_->isActive()) {
        node_->ainfo << "set active to " << (active ? "true" : "false") << std::endl;
    }
    node_state_->setActive(active);
}

bool NodeHandle::isActive() const
{
    return node_state_->isActive();
}


void NodeHandle::stop()
{
    node_->reset();
    
    for(OutputPtr i : getExternalOutputs()) {
        i->stop();
    }
    for(InputPtr i : getExternalInputs()) {
        i->stop();
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
    nodeStateChanged();
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
    apex_assert_hard(uuid_provider_);
    csapex::param::Parameter* p = param.get();
    
    auto pos = param_2_input_.find(p->name());
    if(pos != param_2_input_.end() && pos->second.lock()) {
        return;
    }
    
    
    {
        InputPtr cin = std::make_shared<Input>(uuid_provider_->makeDerivedUUID(getUUID(), std::string("in_") + p->name()));
        cin->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());
        cin->setOptional(true);
        cin->setLabel(p->name());
        
        param_2_input_[p->name()] = cin;
        input_2_param_[cin.get()] = p;
        
        manageInput(cin);
    }
    {
        OutputPtr cout = std::make_shared<StaticOutput>(uuid_provider_->makeDerivedUUID(getUUID(), std::string("out_") + p->name()));
        cout->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());
        cout->setLabel(p->name());
        
        param_2_output_[p->name()] = cout;
        output_2_param_[cout.get()] = p;
        
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
    }
    // else: do nothing and ignore the parameter
    
    param::TriggerParameterPtr t = std::dynamic_pointer_cast<param::TriggerParameter>(p);
    if(t) {
        Event* trigger = NodeModifier::addEvent(t->name());
        NodeModifier::addSlot(t->name(), std::bind(&param::TriggerParameter::trigger, t), false);
        node_->addParameterCallback(t.get(), std::bind(&Event::trigger, trigger));
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
    apex_assert_hard(input_2_param_.erase(cin) != 0);
    
    apex_assert_hard(param_2_output_.erase(p->name()) != 0);
    apex_assert_hard(output_2_param_.erase(cout) != 0);
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
    } else if(msg::hasMessage(source) && !msg::isMessage<connection_types::MarkerMessage>(source)) {
        node_->ainfo << "parameter " << p->name() << " got a message of unsupported type" << std::endl;
    }
}

Input* NodeHandle::addInput(TokenDataConstPtr type, const std::string& label, bool dynamic, bool optional)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "in");
    InputPtr c;
    if(dynamic) {
        c = std::make_shared<DynamicInput>(uuid);
    } else {
        c = std::make_shared<Input>(uuid);
    }
    c->setLabel(label);
    c->setOptional(optional);
    c->setType(type);
    
    manageInput(c);
    
    return c.get();
}

Output* NodeHandle::addOutput(TokenDataConstPtr type, const std::string& label, bool dynamic)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "out");
    OutputPtr c;
    if(dynamic) {
        c = std::make_shared<DynamicOutput>(uuid);
    } else {
        c = std::make_shared<StaticOutput>(uuid);
    }
    c->setLabel(label);
    c->setType(type);
    
    manageOutput(c);
    return c.get();
}

Slot* NodeHandle::addSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr& )> callback, bool active)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "slot");
    SlotPtr slot = std::make_shared<Slot>(callback, uuid, active);
    slot->setLabel(label);
    slot->setType(type);
    
    manageSlot(slot);
    
    return slot.get();
}

Event* NodeHandle::addEvent(TokenDataConstPtr type, const std::string& label)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "event");
    EventPtr event = std::make_shared<Event>(uuid);
    event->setLabel(label);
    event->setType(type);
    
    manageEvent(event);
    
    return event.get();
}

InputPtr NodeHandle::addInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool dynamic, bool optional)
{
    InputPtr in = std::make_shared<Input>(internal_uuid);
    in->setType(type);
    in->setLabel(label);
    in->setOptional(optional);
    
    internal_inputs_.push_back(in);
    
    return in;
}

OutputPtr NodeHandle::addInternalOutput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool dynamic)
{
    OutputPtr out;
    if(dynamic) {
        out = std::make_shared<DynamicOutput>(internal_uuid);
    } else {
        out = std::make_shared<StaticOutput>(internal_uuid);
    }
    out->setType(type);
    out->setLabel(label);
    
    internal_outputs_.push_back(out);
    
    return out;
}

SlotPtr NodeHandle::addInternalSlot(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string &label, std::function<void (const TokenPtr &)> callback)
{
    apex_assert_hard(uuid_provider_);
    SlotPtr slot = std::make_shared<Slot>(callback, internal_uuid, false);
    slot->setLabel(label);
    slot->setType(type);
    
    internal_slots_.push_back(slot);
    
    connectConnector(slot.get());
    
    connectorCreated(slot);
    
    return slot;
}

EventPtr NodeHandle::addInternalEvent(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    apex_assert_hard(uuid_provider_);
    EventPtr event = std::make_shared<Event>(internal_uuid);
    event->setLabel(label);
    event->setType(type);
    
    internal_events_.push_back(event);
    
    connectConnector(event.get());
    
    connectorCreated(event);
    
    return event;
}

void NodeHandle::removeInternalPorts()
{
    internal_outputs_.clear();
    internal_inputs_.clear();
    internal_slots_.clear();
    internal_events_.clear();
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
        connectorRemoved(input);
        
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
        connectorRemoved(output);
        
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
        connectorRemoved(slot);
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
        connectorRemoved(trigger);
    }
}

void NodeHandle::manageInput(InputPtr in)
{
    if(!getUUID().empty()) {
        apex_assert_hard(in->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_inputs_.push_back(in);
    
    connectConnector(in.get());
    
    connectorCreated(in);
    transition_in_->addInput(in);
}

void NodeHandle::manageOutput(OutputPtr out)
{
    if(!getUUID().empty()) {
        apex_assert_hard(out->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_outputs_.push_back(out);
    
    connectConnector(out.get());
    
    connections_[out.get()].emplace_back(out->messageProcessed.connect([this](Connectable*) { mightBeEnabled(); }));
    connections_[out.get()].emplace_back(out->connection_removed_to.connect([this](Connectable*) { mightBeEnabled(); }));
    connections_[out.get()].emplace_back(out->connection_added_to.connect([this](Connectable*) { mightBeEnabled(); }));
    connections_[out.get()].emplace_back(out->connectionEnabled.connect([this](bool) { mightBeEnabled(); }));
    
    connectorCreated(out);
    transition_out_->addOutput(out);
}

bool NodeHandle::isParameterInput(Input *in) const
{
    return  input_2_param_.find(in) != input_2_param_.end();
}

bool NodeHandle::isParameterOutput(Output *out) const
{
    return output_2_param_.find(out) != output_2_param_.end();
}

void NodeHandle::manageSlot(SlotPtr s)
{
    if(!getUUID().empty()) {
        apex_assert_hard(s->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_slots_.push_back(s);
    
    connectConnector(s.get());
    
    connectorCreated(s);
}

void NodeHandle::manageEvent(EventPtr t)
{
    if(!getUUID().empty()) {
        apex_assert_hard(t->getUUID().rootUUID() == getUUID().rootUUID());
    }
    
    external_events_.push_back(t);
    
    connectConnector(t.get());
    
    connectorCreated(t);
}

Connectable* NodeHandle::getConnector(const UUID &uuid) const
{
    Connectable* res = getConnectorNoThrow(uuid);
    if(!res) {
        throw std::logic_error(std::string("the connector type '") + uuid.type() + "' is unknown.");
    }
    return res;
}

Connectable* NodeHandle::getConnectorNoThrow(const UUID &uuid) const noexcept
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

Input* NodeHandle::getInput(const UUID& uuid) const
{
    for(InputPtr in : external_inputs_) {
        if(in->getUUID() == uuid) {
            return in.get();
        }
    }
    
    GraphPtr graph = std::dynamic_pointer_cast<Graph>(node_);
    if(graph) {
        return graph->getForwardedInputInternal(uuid).get();
    }
    
    return nullptr;
}

Output* NodeHandle::getOutput(const UUID& uuid) const
{
    for(OutputPtr out : external_outputs_) {
        if(out->getUUID() == uuid) {
            return out.get();
        }
    }
    
    GraphPtr graph = std::dynamic_pointer_cast<Graph>(node_);
    if(graph) {
        return graph->getForwardedOutputInternal(uuid).get();
    }
    
    return nullptr;
}


Slot* NodeHandle::getSlot(const UUID& uuid) const
{
    for(SlotPtr s : external_slots_) {
        if(s->getUUID() == uuid) {
            return s.get();
        }
    }
    
    GraphPtr graph = std::dynamic_pointer_cast<Graph>(node_);
    if(graph) {
        return graph->getForwardedSlotInternal(uuid).get();
    }
    
    return nullptr;
}


Event* NodeHandle::getEvent(const UUID& uuid) const
{
    for(EventPtr t : external_events_) {
        if(t->getUUID() == uuid) {
            return t.get();
        }
    }
    
    GraphPtr graph = std::dynamic_pointer_cast<Graph>(node_);
    if(graph) {
        return graph->getForwardedEventInternal(uuid).get();
    }
    
    return nullptr;
}

void NodeHandle::removeInput(const UUID &uuid)
{
    removeInput(getInput(uuid));
}

void NodeHandle::removeOutput(const UUID &uuid)
{
    removeOutput(getOutput(uuid));
}

void NodeHandle::removeSlot(const UUID &uuid)
{
    removeSlot(getSlot(uuid));
}

void NodeHandle::removeEvent(const UUID &uuid)
{
    removeEvent(getEvent(uuid));
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

std::vector<InputPtr> NodeHandle::getExternalInputs() const
{
    return external_inputs_;
}
std::vector<InputPtr> NodeHandle::getInternalInputs() const
{
    return internal_inputs_;
}



std::vector<OutputPtr> NodeHandle::getExternalOutputs() const
{
    return external_outputs_;
}
std::vector<OutputPtr> NodeHandle::getInternalOutputs() const
{
    return internal_outputs_;
}



std::vector<SlotPtr> NodeHandle::getExternalSlots() const
{
    return external_slots_;
}
std::vector<SlotPtr> NodeHandle::getInternalSlots() const
{
    return internal_slots_;
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

std::map<Input*,csapex::param::Parameter*>& NodeHandle::inputToParamMap()
{
    return input_2_param_;
}

std::map<Output*,csapex::param::Parameter*>& NodeHandle::outputToParamMap()
{
    return output_2_param_;
}


void NodeHandle::connectConnector(Connectable *c)
{
    connections_[c].emplace_back(c->connectionInProgress.connect(connectionInProgress));
    connections_[c].emplace_back(c->connectionStart.connect(connectionStart));
    connections_[c].emplace_back(c->connection_added_to.connect(connectionDone));
}


void NodeHandle::disconnectConnector(Connectable* c)
{
    for(auto& connection : connections_[c]) {
        connection.disconnect();
    }
    connections_[c].clear();
}

UUIDProvider* NodeHandle::getUUIDProvider()
{
    return uuid_provider_;
}
