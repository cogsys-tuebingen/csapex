/// HEADER
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/event.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/timer.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/any_message.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Graph::Graph()
    : transition_relay_in_(new InputTransition),
      transition_relay_out_(new OutputTransition),
      is_initialized_(false)
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
        deleteConnection(c);
    }
    apex_assert_hard(connections_.empty());

    auto nodes = nodes_;
    for(NodeHandlePtr node : nodes) {
        deleteNode(node->getUUID());
    }
    apex_assert_hard(nodes_.empty());

    node_component_.clear();
    node_level_.clear();

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

void Graph::deleteNode(const UUID& uuid)
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
        nodeRemoved(removed);
        buildConnectedComponents();
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

void Graph::deleteConnection(ConnectionPtr connection)
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

            buildConnectedComponents();

            connectionDeleted(connection.get());
            state_changed();

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

    assignLevels();

    structureChanged(this);
}

void Graph::assignLevels()
{
    std::map<NodeHandle*, int> node_level;

    static const int NO_LEVEL = std::numeric_limits<int>::min();

    std::deque<NodeHandle*> unmarked;
    for(auto nh : nodes_) {
        if(nh->isSource() && nh->isSink()) {
            node_level[nh.get()] = 0;
        } else if(node_parents_[nh.get()].empty()) {
            node_level[nh.get()] = 0;
        } else {
            node_level[nh.get()] = NO_LEVEL;
            unmarked.push_back(nh.get());
        }
    }

    std::deque<NodeHandle*> gateways;

    // to assign a level, every parent must be known
    while(!unmarked.empty()) {
        NodeHandle* current = unmarked.front();
        unmarked.pop_front();

        int max_level = NO_LEVEL;
        for(NodeHandle* parent : node_parents_.at(current)) {
            int parent_level = node_level[parent];
            if(parent_level == NO_LEVEL) {
                max_level = NO_LEVEL;
                break;
            } else {
                if(parent_level > max_level) {
                    max_level = parent_level;
                }
            }
        }

        int max_dynamic_level = NO_LEVEL;
        bool has_dynamic_parent_output = false;
        bool has_dynamic_input = false;
        for(const auto& input : current->getExternalInputs()) {
            if(input->isDynamic()) {
                has_dynamic_input = true;
            }

            for(const auto& connection : input->getConnections()) {
                const auto& parent_output = connection->from();
                if(parent_output->isDynamic()) {
                    has_dynamic_parent_output = true;
                    NodeHandle* node = findNodeHandleForConnector(parent_output->getUUID());
                    int level = node_level.at(node);
                    //                    apex_assert_hard(level != NO_LEVEL);

                    if(level > max_dynamic_level) {
                        max_dynamic_level = level;
                    }
                }
            }
        }

        bool unknown_parent = max_level == NO_LEVEL;
        if(unknown_parent) {
            unmarked.push_back(current);

        } else {
            if(!has_dynamic_parent_output && !has_dynamic_input) {
                node_level[current] = max_level;

            } else if(has_dynamic_parent_output && !has_dynamic_input) {
                node_level[current] = max_level + 1;

            } else if(!has_dynamic_parent_output && has_dynamic_input) {
                node_level[current] = max_level - 1;
                gateways.push_back(current);

            } else if(has_dynamic_parent_output && has_dynamic_input) {
                node_level[current] = max_level;
            }
        }
    }

    for(auto node : nodes_) {
        node->setLevel(node_level[node.get()]);

        for(auto output : node->getExternalOutputs()) {
            if(output->isDynamic()) {
                DynamicOutput* dout = dynamic_cast<DynamicOutput*>(output.get());
                dout->clearCorrespondents();
            }
        }
    }

    for(NodeHandle* node : gateways) {
        DynamicOutput* correspondent = nullptr;

        // perform bfs to find the parent with a dynamic output
        std::deque<NodeHandle*> Q;
        std::set<NodeHandle*> visited;
        Q.push_back(node);
        while(!Q.empty()) {
            auto* current = Q.front();
            Q.pop_front();
            visited.insert(current);

            for(auto input : current->getExternalInputs()) {
                if(input->isConnected()) {
                    ConnectionPtr connection = input->getConnections().front();
                    Output* out = dynamic_cast<Output*>(connection->from());
                    if(out) {
                        auto* parent = findNodeHandleForConnector(out->getUUID());

                        if(out->isDynamic() && parent->getLevel() == node->getLevel()) {
                            correspondent = dynamic_cast<DynamicOutput*>(out);
                            Q.clear();
                            break;
                        }

                        if(visited.find(parent) == visited.end()) {
                            Q.push_back(parent);
                        }
                    }
                }
            }
        }

        if(correspondent) {
            for(auto input : node->getExternalInputs()) {
                if(input->isDynamic()) {
                    DynamicInput* di = dynamic_cast<DynamicInput*>(input.get());
                    di->setCorrespondent(correspondent);
                    correspondent->addCorrespondent(di);
                }
            }
        }
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

int Graph::getLevel(const UUID &node_uuid) const
{
    NodeHandle* node = findNodeHandleNoThrow(node_uuid);
    if(!node) {
        return 0;
    }

    return node->getLevel();
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
    apex_assert_hard(res);
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
}

void Graph::process(NodeModifier &node_modifier, Parameterizable &params,
                    std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation)
{
    continuation_ = continuation;

    apex_assert_hard(transition_relay_out_->canStartSendingMessages());
    for(Input* i : node_modifier.getMessageInputs()) {
        TokenDataConstPtr m = msg::getMessage(i);
        OutputPtr o = external_to_internal_outputs_.at(i->getUUID());

        msg::publish(o.get(), m);
    }
    transition_relay_out_->sendMessages(node_handle_->isActive());

    outputActivation();

    //    continuation_ = continuation;

    //    received_.clear();
    //    for(Output* o : node_modifier.getMessageOutputs()) {
    //        received_[o] = false;
    //    }

    //    for(Input* i : node_modifier.getMessageInputs()) {
    //        TokenDataConstPtr m = msg::getMessage(i);
    //        OutputPtr o = pass_on_inputs_[i];

    //        msg::publish(o.get(), m);
    //        o->commitMessages();
    //        o->publish();
    //    }
}

bool Graph::isAsynchronous() const
{
    return true;
}

Input* Graph::createVariadicInput(TokenDataConstPtr type, const std::string& label, bool optional)
{
    auto pair = addForwardingInput(type, label, optional);
    return node_handle_->getInput(pair.external);
}

InputPtr Graph::createInternalInput(const TokenDataConstPtr& type, const UUID &internal_uuid, const std::string& label, bool dynamic, bool optional)
{
    InputPtr input = node_handle_->addInternalInput(type, internal_uuid, label, dynamic, optional);

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

    OutputPtr relay = createInternalOutput(type, internal_uuid, label, false);

    external_to_internal_outputs_[external_input->getUUID()] = relay;

    relay_to_external_input_[internal_uuid] = external_input->getUUID();

    forwardingAdded(relay);

    return external_input->getUUID();
}


Output* Graph::createVariadicOutput(TokenDataConstPtr type, const std::string& label)
{
    auto pair = addForwardingOutput(type, label);
    return node_handle_->getOutput(pair.external);
}


OutputPtr Graph::createInternalOutput(const TokenDataConstPtr& type, const UUID& internal_uuid, const std::string& label, bool dynamic)
{
    OutputPtr output = node_handle_->addInternalOutput(type, internal_uuid, label, dynamic);

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

    InputPtr relay = createInternalInput(type, internal_uuid, label, false, true);

    std::weak_ptr<Output> external_output_weak = std::dynamic_pointer_cast<Output>(external_output->shared_from_this());
    relay->message_set.connect([this, external_output_weak, relay](Connectable*) {
        if(auto external_output = external_output_weak.lock()) {
            msg::publish(external_output.get(), relay->getToken()->getTokenData());
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

void Graph::removeInternalPorts()
{
    node_handle_->removeInternalPorts();
    internal_events_.clear();
    internal_slots_.clear();
}

void Graph::notifyMessagesProcessed()
{
    GeneratorNode::notifyMessagesProcessed();

    transition_relay_in_->notifyMessageProcessed();
}

void Graph::inputActivation()
{
    if(!transition_relay_in_->hasConnection()) {
        if(continuation_) {
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
            continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();
        }
    }
}

void Graph::outputActivation()
{
    if(transition_relay_in_->isEnabled() && node_handle_->getOutputTransition()->canStartSendingMessages()) {
        // if(node_handle_->isSource() || continuation_) {
        apex_assert_hard(transition_relay_in_->isEnabled());
        apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

        if(node_handle_->isSource()) {
            transition_relay_in_->forwardMessages();

            updated();

            if(node_handle_->isSink()) {
                notifyMessagesProcessed();
            }

        } else {
            if(transition_relay_in_->hasConnection()) {
                //apex_assert_hard(continuation_);
                if(continuation_) {
                    transition_relay_in_->forwardMessages();

                    continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
                    continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();

                    if(node_handle_->isSink()) {
                        notifyMessagesProcessed();
                    }
                }
            }
        }
        //        });

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
