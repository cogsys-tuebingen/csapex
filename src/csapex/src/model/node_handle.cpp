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
#include <csapex/signal/trigger.h>
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

    triggerNodeStateChanged();

    node_->stateChanged();

    node_->parameters_changed.connect(parametersChanged);
    node_->getParameterState()->parameter_set_changed->connect(parametersChanged);
    node_->getParameterState()->parameter_added->connect(std::bind(&NodeHandle::makeParameterConnectable, this, std::placeholders::_1));
    node_->getParameterState()->parameter_removed->connect(std::bind(&NodeHandle::makeParameterNotConnectable, this, std::placeholders::_1));
}

NodeHandle::~NodeHandle()
{
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

    nodeRemoved();
}

int NodeHandle::getLevel() const
{
    return level_;
}

void NodeHandle::setLevel(int level)
{
    level_ = level;
    for(InputPtr in : inputs_) {
        in->setLevel(level);
    }
    for(OutputPtr out : outputs_) {
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
    for(InputPtr in : inputs_) {
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
    return sink_ || outputs_.empty() || transition_out_->isSink();
}


void NodeHandle::stop()
{
    node_->abort();

    for(OutputPtr i : getAllOutputs()) {
        i->stop();
    }
    for(InputPtr i : getAllInputs()) {
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

void NodeHandle::triggerNodeStateChanged()
{
    nodeStateChanged();
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

        addInput(cin);
    }
    {
        OutputPtr cout = std::make_shared<StaticOutput>(uuid_provider_->makeDerivedUUID(getUUID(), std::string("out_") + p->name()));
        cout->setType(connection_types::makeEmpty<connection_types::GenericValueMessage<T> >());
        cout->setLabel(p->name());

        param_2_output_[p->name()] = cout;
        output_2_param_[cout.get()] = p;

        addOutput(cout);
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
        Trigger* trigger = addTrigger(t->name());
        addSlot(t->name(), std::bind(&param::TriggerParameter::trigger, t), false);
        node_->addParameterCallback(t.get(), std::bind(&Trigger::trigger, trigger));
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

Input* NodeHandle::addInput(ConnectionTypeConstPtr type, const std::string& label, bool dynamic, bool optional)
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

    addInput(c);

    return c.get();
}

Output* NodeHandle::addOutput(ConnectionTypeConstPtr type, const std::string& label, bool dynamic)
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

    addOutput(c);
    return c.get();
}

Slot* NodeHandle::addSlot(const std::string& label, std::function<void()> callback, bool active)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "slot");
    SlotPtr slot = std::make_shared<Slot>(callback, uuid, active);
    slot->setLabel(label);

    addSlot(slot);

    return slot.get();
}

Trigger* NodeHandle::addTrigger(const std::string& label)
{
    apex_assert_hard(uuid_provider_);
    UUID uuid = uuid_provider_->generateTypedUUID(getUUID(), "trigger");
    TriggerPtr trigger = std::make_shared<Trigger>(uuid);
    trigger->setLabel(label);

    addTrigger(trigger);

    return trigger.get();
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

void NodeHandle::removeOutput(Output* out)
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

void NodeHandle::removeSlot(Slot* s)
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

void NodeHandle::removeTrigger(Trigger* t)
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

void NodeHandle::addInput(InputPtr in)
{
    inputs_.push_back(in);

    connectConnector(in.get());

    connectorCreated(in);
    transition_in_->addInput(in);
}

void NodeHandle::addOutput(OutputPtr out)
{
    outputs_.push_back(out);

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

void NodeHandle::addSlot(SlotPtr s)
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

void NodeHandle::addTrigger(TriggerPtr t)
{
    triggers_.push_back(t);

    //PROBLEM: wait for all m slots to be done...
    // in the mean time, allow NO TICK; NO PROCESS;
    // BUT ALSO EXECUTE OTHER TRIGGERS OF THE SAME PROCESS / TICK...
    // solution: "lock" the NodeHandle, "unlock" it in signalHandled, once all
    //  TRIGGERs are done (not only this one!!!!!)

    auto connection_triggered = t->triggered.connect([this](){
        mightBeEnabled();
    });
    trigger_triggered_connections_.insert(std::make_pair(t.get(), connection_triggered));

    auto connection_handled = t->all_signals_handled.connect([this](){
        mightBeEnabled();
    });
    trigger_handled_connections_.insert(std::make_pair(t.get(), connection_handled));

    connectConnector(t.get());

    connectorCreated(t);
}

Connectable* NodeHandle::getConnector(const UUID &uuid) const
{
    std::string type = uuid.type();

    if(type == "in" || type == "relayin") {
        return getInput(uuid);
    } else if(type == "out" || type == "relayout") {
        return getOutput(uuid);
    } else if(type == "slot") {
        return getSlot(uuid);
    } else if(type == "trigger") {
        return getTrigger(uuid);
    } else {
        throw std::logic_error(std::string("the connector type '") + type + "' is unknown.");
    }
}

Input* NodeHandle::getInput(const UUID& uuid) const
{
    for(InputPtr in : inputs_) {
        if(in->getUUID() == uuid) {
            return in.get();
        }
    }

    GraphPtr graph = std::dynamic_pointer_cast<Graph>(node_);
    if(graph) {
        return graph->getForwardedInput(uuid).get();
    }

    return nullptr;
}

Output* NodeHandle::getOutput(const UUID& uuid) const
{
    for(OutputPtr out : outputs_) {
        if(out->getUUID() == uuid) {
            return out.get();
        }
    }

    GraphPtr graph = std::dynamic_pointer_cast<Graph>(node_);
    if(graph) {
        return graph->getForwardedOutput(uuid).get();
    }

    return nullptr;
}


Slot* NodeHandle::getSlot(const UUID& uuid) const
{
    for(SlotPtr s : slots_) {
        if(s->getUUID() == uuid) {
            return s.get();
        }
    }

    return nullptr;
}


Trigger* NodeHandle::getTrigger(const UUID& uuid) const
{
    for(TriggerPtr t : triggers_) {
        if(t->getUUID() == uuid) {
            return t.get();
        }
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

void NodeHandle::removeTrigger(const UUID &uuid)
{
    removeTrigger(getTrigger(uuid));
}

std::vector<ConnectablePtr> NodeHandle::getAllConnectors() const
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

std::vector<InputPtr> NodeHandle::getAllInputs() const
{
    return inputs_;
}

std::vector<OutputPtr> NodeHandle::getAllOutputs() const
{
    return outputs_;
}

std::vector<SlotPtr> NodeHandle::getAllSlots() const
{
    return slots_;
}

std::vector<TriggerPtr> NodeHandle::getAllTriggers() const
{
    return triggers_;
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
