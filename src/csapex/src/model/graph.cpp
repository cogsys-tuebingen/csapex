/// HEADER
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_modifier.h>
#include <csapex/profiling/timer.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/generic_vector_message.hpp>
#include <csapex/param/parameter_factory.h>
#include <csapex/param/bitset_parameter.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Graph::Graph()
    : transition_relay_in_(new InputTransition),
      transition_relay_out_(new OutputTransition),
      is_iterating_(false), is_initialized_(false)
{
    transition_relay_in_->setActivationFunction(delegate::Delegate0<>(this, &Graph::outputActivation));

    transition_relay_out_->messages_processed.connect(delegate::Delegate0<>(this, &Graph::inputActivation));
}

Graph::~Graph()
{
    clear();
}


void Graph::initialize(csapex::NodeHandle* node_handle, const UUID &uuid)
{
    Node::initialize(node_handle, uuid);

    if(node_handle->getUUIDProvider()) {
        setParent(node_handle->getUUIDProvider()->shared_from_this(), node_handle->getUUID().getAbsoluteUUID());
    }
}

void Graph::reset()
{
    Node::reset();

    resetActivity();

    continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();

    transition_relay_out_->reset();
    transition_relay_in_->reset();

    iterated_inputs_.clear();
}

void Graph::resetActivity()
{
    auto connections = connections_;
    for(ConnectionPtr c : connections) {
        TokenPtr t = c->getToken();
        if(t) {
            t->setActive(false);
        }
    }

    auto nodes = nodes_;
    for(NodeHandlePtr node : nodes) {
        node->setActive(false);
    }
}

void Graph::stateChanged()
{
    Node::stateChanged();


    if(!is_initialized_) {
        NodeStatePtr state = node_handle_->getNodeState();
        is_initialized_ = true;
    }
}

void Graph::clear()
{
    UUIDProvider::clearCache();

    auto connections = connections_;
    for(ConnectionPtr c : connections) {
        deleteConnection(c, true);
    }
    apex_assert_hard(connections_.empty());

    auto nodes = nodes_;
    for(NodeHandlePtr node : nodes) {
        deleteNode(node->getUUID(), true);
    }
    apex_assert_hard(nodes_.empty());

    node_component_.clear();

    node_parents_.clear();
    node_children_.clear();
}

void Graph::addNode(NodeHandlePtr nh)
{
    apex_assert_hard_msg(nh, "NodeHandle added is not null");
    nodes_.push_back(nh);

    node_parents_[nh.get()] = std::vector<NodeHandle*>();
    node_children_[nh.get()] = std::vector<NodeHandle*>();

    buildConnectedComponents();

    nodeAdded(nh);
}

std::vector<ConnectionPtr> Graph::getConnections()
{
    return connections_;
}

void Graph::deleteNode(const UUID& uuid, bool quiet)
{
    NodeHandle* node_handle = findNodeHandle(uuid);
    node_handle->stop();

    node_parents_[node_handle].clear();
    node_children_[node_handle].clear();

    node_parents_.erase(node_handle);
    node_children_.erase(node_handle);

    NodeHandlePtr removed;

    for(auto it = nodes_.begin(); it != nodes_.end();) {
        if((*it)->getUUID() == uuid) {
            removed = *it;
            it = nodes_.erase(it);

        } else {
            ++it;
        }
    }

    apex_assert_hard(removed);

    if(removed) {
        if(NodePtr node = removed->getNode().lock()) {
            if(GraphPtr child = std::dynamic_pointer_cast<Graph>(node)) {
                child->clear();
            }
        }

        if(!quiet) {
            nodeRemoved(removed);
            buildConnectedComponents();
        }
    }
}

int Graph::countNodes()
{
    return nodes_.size();
}


bool Graph::addConnection(ConnectionPtr connection, bool quiet)
{
    apex_assert_hard(connection);

    NodeHandle* n_from = findNodeHandleForConnector(connection->from()->getUUID());
    NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

    //apex_assert_hard(connection->from()->isConnectionPossible(connection->to()));

    //Connectable* from = findConnector(connection->from()->getUUID());
    //Connectable* to = findConnector(connection->to()->getUUID());

    connections_.push_back(connection);
    //from->addConnection(connection);
    //to->addConnection(connection);
    //from->connection_added_to(from);
    //to->connection_added_to(to);

    if(dynamic_cast<Output*>(connection->from())) {
        if(n_to != n_from && n_to != node_handle_ && n_from != node_handle_) {
            apex_assert_hard(n_to->getUUID().getAbsoluteUUID() != n_from->getUUID().getAbsoluteUUID());

            node_parents_[n_to].push_back(n_from);
            node_children_[n_from].push_back(n_to);

            if(!quiet) {
                buildConnectedComponents();
            }
        }
    }

    if(!quiet) {
        connectionAdded(connection.get());
    }
    return true;
}

void Graph::triggerConnectionsAdded()
{
    buildConnectedComponents();

    for(const ConnectionPtr& connection : connections_) {
        connectionAdded(connection.get());
    }
}

void Graph::deleteConnection(ConnectionPtr connection, bool quiet)
{
    apex_assert_hard(connection);

    auto out = connection->from();
    auto in = connection->to();

    out->removeConnection(in);

    out->fadeConnection(connection);
    in->fadeConnection(connection);

    for(std::vector<ConnectionPtr>::iterator c = connections_.begin(); c != connections_.end();) {
        if(*connection == **c) {
            Connectable* to = connection->to();
            to->setError(false);

            UUID from_uuid = connection->from()->getUUID();
            NodeHandle* n_from = nullptr;
            bool crossing = from_uuid.parentUUID() == getUUID();
            if(crossing) {
                n_from = dynamic_cast<NodeHandle*>(node_modifier_);
            } else {
                n_from = findNodeHandleForConnector(from_uuid);
            }
            NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

            if(dynamic_cast<Output*>(connection->from())) {
                // erase pointer from TO to FROM
                if(!crossing) {
                    if(n_from != n_to && n_from != node_handle_ && n_to != node_handle_) {
                        // if there are multiple edges, this only erases one entry
                        node_parents_[n_to].erase(std::find(node_parents_[n_to].begin(), node_parents_[n_to].end(), n_from));

                        // erase pointer from FROM to TO
                        node_children_[n_from].erase(std::find(node_children_[n_from].begin(), node_children_[n_from].end(), n_to));
                    }
                }
            }
            connections_.erase(c);

            if(!quiet) {
                buildConnectedComponents();

                connectionDeleted(connection.get());
                state_changed();
            }

            for(const auto& c : connections_) {
                apex_assert_hard(c);
            }

            return;

        } else {
            ++c;
        }
    }


    throw std::runtime_error("cannot delete connection");
}


void Graph::buildConnectedComponents()
{
    /* Find all connected sub components of this graph */
    //    std::map<Node*, int> old_node_component = node_component_;
    node_component_.clear();

    std::deque<NodeHandle*> unmarked;
    for(auto node : nodes_) {
        unmarked.push_back(node.get());
        node_component_[node.get()] = -1;
    }

    std::deque<NodeHandle*> Q;
    int component = 0;
    while(!unmarked.empty()) {
        // take a random unmarked node to start bfs from
        auto* start = unmarked.front();
        Q.push_back(start);

        node_component_[start] = component;

        while(!Q.empty()) {
            auto* front = Q.front();
            Q.pop_front();

            checkNodeState(front);

            unmarked.erase(std::find(unmarked.begin(), unmarked.end(), front));

            // iterate all neighbors
            std::vector<NodeHandle*> neighbors;
            for(auto* parent : node_parents_[front]) {
                neighbors.push_back(parent);
            }
            for(auto* child : node_children_[front]) {
                neighbors.push_back(child);
            }

            for(auto* neighbor : neighbors) {
                if(node_component_[neighbor] == -1) {
                    node_component_[neighbor] = component;
                    Q.push_back(neighbor);
                }
            }
        }

        ++component;
    }

    structureChanged(this);
}

void Graph::checkNodeState(NodeHandle *nh)
{
    // check if the node should be enabled
    nh->getInputTransition()->checkIfEnabled();
    nh->getOutputTransition()->checkIfEnabled();
    if(NodeWorker* worker = nh->getNodeWorker()) {
        worker->checkIO();
    }
}

int Graph::getComponent(const UUID &node_uuid) const
{
    NodeHandle* node = findNodeHandleNoThrow(node_uuid);
    if(!node) {
        return -1;
    }

    return node_component_.at(node);
}

Node* Graph::findNode(const UUID& uuid) const
{
    Node* node = findNodeNoThrow(uuid);
    if(node) {
        return node;
    }
    throw NodeNotFoundException(uuid.getFullName());
}



NodeHandle* Graph::findNodeHandle(const UUID& uuid) const
{
    //    if(uuid == getUUID()) {
    if(uuid.empty()) {
        return node_handle_;
    }

    NodeHandle* node_handle = findNodeHandleNoThrow(uuid);
    if(node_handle) {
        return node_handle;
    }
    throw NodeHandleNotFoundException(uuid.getFullName());
}

Node* Graph::findNodeNoThrow(const UUID& uuid) const noexcept
{
    NodeHandle* nh = findNodeHandleNoThrow(uuid);
    if(nh) {
        auto node = nh->getNode().lock();
        if(node) {
            return node.get();
        }
    }

    return nullptr;
}


NodeHandle* Graph::findNodeHandleNoThrow(const UUID& uuid) const noexcept
{
    if(uuid.empty()) {
        return node_handle_;
    }
    if(uuid.composite()) {
        UUID root = uuid.rootUUID();

        NodeHandle* root_nh = findNodeHandleNoThrow(root);
        if(root_nh) {
            NodePtr root_node = root_nh->getNode().lock();
            if(root_node) {
                GraphPtr graph = std::dynamic_pointer_cast<Graph>(root_node);
                if(graph) {
                    return graph->findNodeHandle(uuid.nestedUUID());
                }
            }
        }

    } else {
        for(const auto b : nodes_) {
            if(b->getUUID() == uuid) {
                return b.get();
            }
        }
    }

    return nullptr;
}

Node* Graph::findNodeForConnector(const UUID &uuid) const
{
    try {
        return findNode(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find node of connector \"") + uuid.getFullName() + ": " + e.what());
    }
}


NodeHandle* Graph::findNodeHandleForConnector(const UUID &uuid) const
{
    try {
        return findNodeHandle(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find handle of connector \"") + uuid.getFullName());
    }
}

NodeHandle* Graph::findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return findNodeHandleNoThrow(uuid.parentUUID());
}

NodeHandle* Graph::findNodeHandleWithLabel(const std::string& label) const
{
    for(const auto b : nodes_) {
        NodeStatePtr state = b->getNodeState();
        if(state) {
            if(state->getLabel() == label) {
                return b.get();
            }
        }
    }
    return nullptr;
}

std::vector<NodeHandle*> Graph::getAllNodeHandles()
{
    std::vector<NodeHandle*> node_handles;
    for(const auto& node : nodes_) {
        node_handles.push_back(node.get());
    }

    return node_handles;
}

Connectable* Graph::findConnector(const UUID &uuid)
{
    Connectable* res = findConnectorNoThrow(uuid);
    if(!res) {
        throw std::runtime_error(std::string("cannot find connector with UUID=") + uuid.getFullName());
    }
    return res;
}

Connectable* Graph::findConnectorNoThrow(const UUID &uuid) noexcept
{
    if(internal_slots_.find(uuid) != internal_slots_.end()) {
        return internal_slots_.at(uuid).get();
    }
    if(internal_events_.find(uuid) != internal_events_.end()) {
        return internal_events_.at(uuid).get();
    }

    NodeHandle* owner = findNodeHandleNoThrow(uuid.parentUUID());
    if(!owner) {

        return nullptr;
    }

    return owner->getConnectorNoThrow(uuid);
}

ConnectionPtr Graph::getConnectionWithId(int id)
{
    for(ConnectionPtr& connection : connections_) {
        if(connection->id() == id) {
            return connection;
        }
    }

    return nullptr;
}

ConnectionPtr Graph::getConnection(Connectable* from, Connectable* to)
{
    return getConnection(from->getUUID(), to->getUUID());
}

ConnectionPtr Graph::getConnection(const UUID &from, const UUID &to)
{
    for(ConnectionPtr& connection : connections_) {
        if(connection->from()->getUUID() == from && connection->to()->getUUID() == to) {
            return connection;
        }
    }

    std::cerr << "error: cannot get connection from " << from << " to " << to << std::endl;

    return nullptr;
}

int Graph::getConnectionId(ConnectionPtr c)
{
    if(c != nullptr) {
        return c->id();
    }

    return -1;
}

Graph::node_iterator Graph::beginNodes()
{
    return nodes_.begin();
}

const Graph::node_const_iterator Graph::beginNodes() const
{
    return nodes_.cbegin();
}


Graph::node_iterator Graph::endNodes()
{
    return nodes_.end();
}

const Graph::node_const_iterator Graph::endNodes() const
{
    return nodes_.cend();
}

void Graph::setup(NodeModifier &modifier)
{
    setupVariadic(modifier);

    activation_event_ = createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), makeUUID("event_activation"), "activation");
    deactivation_event_ = createInternalEvent(connection_types::makeEmpty<connection_types::AnyMessage>(), makeUUID("event_deactivation"), "deactivation");
}

void Graph::activation()
{
    if(activation_event_) {
        TokenDataConstPtr data(new connection_types::AnyMessage);
        TokenPtr token = std::make_shared<Token>(data);
        token->setActive(true);
        activation_event_->triggerWith(token);
    }
}

void Graph::deactivation()
{
    if(deactivation_event_) {
        deactivation_event_->trigger();
    }
}


void Graph::setupParameters(Parameterizable &params)
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
            Input* i = node_handle_->getInput(id);
            apex_assert_hard(i);

            bool iterate = iterated_inputs_param_->isSet(id.getFullName());
            setIterationEnabled(id, iterate);
        }
    });
}

void Graph::process(NodeModifier &node_modifier, Parameterizable &params,
                    std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation)
{
    continuation_ = continuation;

    // can fail...
    apex_assert_hard(transition_relay_out_->canStartSendingMessages());

    is_iterating_ = false;
    has_sent_current_iteration_ = false;

    for(Input* i : node_modifier.getMessageInputs()) {
        TokenDataConstPtr m = msg::getMessage(i);
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

    transition_relay_out_->sendMessages(node_handle_->isActive());

    outputActivation();
}

bool Graph::isAsynchronous() const
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

Input* Graph::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional)
{
    auto pair = addForwardingInput(type, label, optional);
    return node_handle_->getInput(pair.external);
}

InputPtr Graph::createInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool optional)
{
    InputPtr input = node_handle_->addInternalInput(type, internal_uuid, label, optional);

    transition_relay_in_->addInput(input);

    input->connectionInProgress.connect(internalConnectionInProgress);

    return input;
}


void Graph::removeVariadicInput(InputPtr input)
{
    OutputPtr relay = external_to_internal_outputs_[input->getUUID()];
    forwardingRemoved(relay);

    VariadicInputs::removeVariadicInput(input);

    relay_to_external_input_.erase(relay->getUUID());

    external_to_internal_outputs_.erase(input->getUUID());
    transition_relay_out_->removeOutput(relay);
}

RelayMapping Graph::addForwardingInput(const TokenDataConstPtr& type,
                                       const std::string& label, bool optional)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayout");
    UUID external_uuid = addForwardingInput(internal_uuid, type, label, optional);

    return {external_uuid, internal_uuid};
}

UUID  Graph::addForwardingInput(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label, bool optional)
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


Output* Graph::createVariadicOutput(TokenDataConstPtr type, const std::string& label)
{
    auto pair = addForwardingOutput(type, label);
    return node_handle_->getOutput(pair.external);
}


OutputPtr Graph::createInternalOutput(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    OutputPtr output = node_handle_->addInternalOutput(type, internal_uuid, label);

    transition_relay_out_->addOutput(output);

    output->connectionInProgress.connect(internalConnectionInProgress);

    return output;
}

void Graph::removeVariadicOutput(OutputPtr output)
{
    InputPtr relay = external_to_internal_inputs_[output->getUUID()];
    forwardingRemoved(relay);

    relay->message_set.disconnectAll();

    VariadicOutputs::removeVariadicOutput(output);

    relay_to_external_output_.erase(relay->getUUID());

    external_to_internal_inputs_.erase(output->getUUID());
    transition_relay_in_->removeInput(relay);
}

RelayMapping Graph::addForwardingOutput(const TokenDataConstPtr& type,
                                        const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayin");
    UUID external_uuid = addForwardingOutput(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID Graph::addForwardingOutput(const UUID& internal_uuid, const TokenDataConstPtr& type,  const std::string& label)
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



SlotPtr Graph::createInternalSlot(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, std::function<void (const TokenPtr& )> callback)
{
    SlotPtr slot = node_handle_->addInternalSlot(connection_types::makeEmpty<connection_types::AnyMessage>(), internal_uuid, label, callback);

    slot->connectionInProgress.connect(internalConnectionInProgress);

    internal_slots_[internal_uuid] = slot;

    return slot;
}

Slot* Graph::createVariadicSlot(TokenDataConstPtr type, const std::string& label, std::function<void(const TokenPtr&)> callback)
{
    auto pair = addForwardingSlot(type, label);
    return node_handle_->getSlot(pair.external);
}

void Graph::removeVariadicSlot(SlotPtr slot)
{
    EventPtr relay = external_to_internal_events_.at(slot->getUUID());
    external_to_internal_events_.erase(slot->getUUID());

    internal_events_.erase(relay->getUUID());

    forwardingRemoved(relay);

    VariadicSlots::removeVariadicSlot(slot);

    relay_to_external_slot_.erase(relay->getUUID());
}

RelayMapping Graph::addForwardingSlot(const TokenDataConstPtr& type, const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayevent");
    UUID external_uuid = addForwardingSlot(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID Graph::addForwardingSlot(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label)
{
    registerUUID(internal_uuid);

    EventPtr relay = createInternalEvent(type, internal_uuid, label);

    auto cb = [relay](const TokenConstPtr& data) {
        relay->triggerWith(std::make_shared<Token>(*data));
    };

    Slot* external_slot = VariadicSlots::createVariadicSlot(type, label, cb);

    crossConnectLabelChange(external_slot, relay.get());

    external_to_internal_events_[external_slot->getUUID()] = relay;

    relay_to_external_slot_[internal_uuid] = external_slot->getUUID();

    forwardingAdded(relay);

    return external_slot->getUUID();
}

EventPtr Graph::createInternalEvent(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label)
{
    EventPtr event = node_handle_->addInternalEvent(type, internal_uuid, label);

    event->connectionInProgress.connect(internalConnectionInProgress);

    internal_events_[internal_uuid] = event;

    //    event->triggered.connect([this]() {
    //        std::cerr << "tigger internal event" << std::endl;
    //    });

    return event;
}

Event* Graph::createVariadicEvent(TokenDataConstPtr type, const std::string& label)
{
    auto pair = addForwardingEvent(type, label);
    return node_handle_->getEvent(pair.external);
}

void Graph::removeVariadicEvent(EventPtr event)
{
    SlotPtr relay = external_to_internal_slots_.at(event->getUUID());
    external_to_internal_slots_.erase(event->getUUID());

    internal_slots_.erase(relay->getUUID());

    forwardingRemoved(relay);

    VariadicEvents::removeVariadicEvent(event);

    relay_to_external_event_.erase(relay->getUUID());
}

RelayMapping Graph::addForwardingEvent(const TokenDataConstPtr& type, const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayslot");
    UUID external_uuid = addForwardingEvent(internal_uuid, type, label);

    return {external_uuid, internal_uuid};
}

UUID Graph::addForwardingEvent(const UUID& internal_uuid, const TokenDataConstPtr& type, const std::string& label)
{
    registerUUID(internal_uuid);

    Event* external_event = VariadicEvents::createVariadicEvent(type, label);

    auto cb = [this, external_event](const TokenPtr& token){
        external_event->triggerWith(token);
    };

    SlotPtr relay = createInternalSlot(type, internal_uuid, label, cb);

    crossConnectLabelChange(external_event, relay.get());

    external_to_internal_slots_[external_event->getUUID()] = relay;

    relay_to_external_event_[internal_uuid] = external_event->getUUID();

    forwardingAdded(relay);

    return external_event->getUUID();
}

OutputPtr Graph::getRelayForInput(const UUID& external_uuid) const
{
    return external_to_internal_outputs_.at(external_uuid);
}
InputPtr Graph::getRelayForOutput(const UUID& external_uuid) const
{
    return external_to_internal_inputs_.at(external_uuid);
}
EventPtr Graph::getRelayForSlot(const UUID& external_uuid) const
{
    return external_to_internal_events_.at(external_uuid);
}
SlotPtr Graph::getRelayForEvent(const UUID& external_uuid) const
{
    return external_to_internal_slots_.at(external_uuid);
}

InputPtr Graph::getForwardedInputInternal(const UUID &internal_uuid) const
{
    return transition_relay_in_->getInput(internal_uuid);
}

OutputPtr Graph::getForwardedOutputInternal(const UUID &internal_uuid) const
{
    return transition_relay_out_->getOutput(internal_uuid);
}

SlotPtr Graph::getForwardedSlotInternal(const UUID &internal_uuid) const
{
    return internal_slots_.at(internal_uuid);
}

EventPtr Graph::getForwardedEventInternal(const UUID &internal_uuid) const
{
    return internal_events_.at(internal_uuid);
}

UUID Graph::getForwardedInputExternal(const UUID &internal_uuid) const
{
    return relay_to_external_output_.at(internal_uuid);
}

UUID Graph::getForwardedOutputExternal(const UUID &internal_uuid) const
{
    return relay_to_external_input_.at(internal_uuid);
}

UUID Graph::getForwardedSlotExternal(const UUID &internal_uuid) const
{
    return relay_to_external_slot_.at(internal_uuid);
}

UUID Graph::getForwardedEventExternal(const UUID &internal_uuid) const
{
    return relay_to_external_event_.at(internal_uuid);
}

std::vector<UUID> Graph::getInternalOutputs() const
{
    return transition_relay_out_->getOutputs();
}
std::vector<UUID> Graph::getInternalInputs() const
{
    return transition_relay_in_->getInputs();
}
std::vector<UUID> Graph::getInternalSlots() const
{
    std::vector<UUID> res;
    for(const auto& pair : internal_slots_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}
std::vector<UUID> Graph::getInternalEvents() const
{
    std::vector<UUID> res;
    for(const auto& pair : internal_events_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}

void Graph::setIterationEnabled(const UUID& external_input_uuid, bool enabled)
{
    if(enabled) {
        iterated_inputs_.insert(external_input_uuid);
        Input* i = node_handle_->getInput(external_input_uuid);
        OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

        TokenDataConstPtr vector_type = i->getType();
        if(vector_type->isContainer()) {
            o->setType(vector_type->nestedType());
        }

    } else {
        iterated_inputs_.erase(external_input_uuid);
    }
}

void Graph::removeInternalPorts()
{
    node_handle_->removeInternalPorts();
    internal_events_.clear();
    internal_slots_.clear();
}

void Graph::notifyMessagesProcessed()
{
    GeneratorNode::notifyMessagesProcessed();

    //    tryFinishProcessing();
    transition_relay_in_->notifyMessageProcessed();
}

void Graph::inputActivation()
{
    if(node_handle_->isSink() || !transition_relay_in_->areMessagesForwarded()) {
        tryFinishProcessing();
    }
}

void Graph::outputActivation()
{
    tryFinishProcessing();
}

void Graph::tryFinishProcessing()
{
    if(!node_handle_->isSink()) {
        if(!node_handle_->getOutputTransition()->canStartSendingMessages()) {
            return;
        }

    } else {
        if(!transition_relay_out_->areAllConnections(Connection::State::DONE)) {
            return;
        }
    }

    if(!node_handle_->isSink() && !has_sent_current_iteration_) {
        if(transition_relay_in_->isEnabled()) {
            apex_assert_hard(transition_relay_in_->isEnabled());
            apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());
            transition_relay_in_->forwardMessages();

            has_sent_current_iteration_ = true;
            if(is_iterating_) {
                transition_relay_in_->notifyMessageProcessed();
            }
        }
    }

    // done processing
    if(is_iterating_) {
        if(!transition_relay_out_->areAllConnections(Connection::State::DONE, Connection::State::READ)) {
            return;
        }

        if(!node_handle_->isSink()) {
            if(!has_sent_current_iteration_){
                return;
            }
        }


        if(iteration_index_ < iteration_count_) {
//            transition_relay_in_->notifyMessageProcessed();

            for(Input* i : node_modifier_->getMessageInputs()) {
                TokenDataConstPtr m = msg::getMessage(i);
                OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

                if(m->isContainer() && iterated_inputs_.find(i->getUUID()) != iterated_inputs_.end()) {
                    has_sent_current_iteration_ = false;
                    msg::publish(o.get(), m->nestedValue(iteration_index_));

                } else {
                    msg::publish(o.get(), m);
                }
            }

            ++iteration_index_;
            transition_relay_out_->sendMessages(node_handle_->isActive());

            return;

        } else {
            is_iterating_ = false;
        }
    }

    if(!node_handle_->isSink()) {
        if(!has_sent_current_iteration_) {
            return;
        }
    }

    transition_relay_in_->notifyMessageProcessed();

    if(node_handle_->isSource()) {
        updated();
    } else {
        if(continuation_) {
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
            continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();
        }
    }

}

std::string Graph::makeStatusString() const
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
