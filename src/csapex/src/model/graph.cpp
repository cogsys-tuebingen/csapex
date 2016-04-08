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
#include <csapex/signal/trigger.h>
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

/// SYSTEM
#include <iostream>

using namespace csapex;

Graph::Graph()
    : transition_relay_in_(new InputTransition),
      transition_relay_out_(new OutputTransition),
      is_initialized_(false), output_active_(false)
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

    setParent(node_handle->getUUIDProvider()->shared_from_this(), node_handle->getUUID().getAbsoluteUUID());
}

void Graph::reset()
{
    Node::reset();

    continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();
    output_active_ = false;

    transition_relay_out_->reset();
    transition_relay_in_->reset();
}

void Graph::stateChanged()
{
    Node::stateChanged();


    if(!is_initialized_) {
        NodeStatePtr state = node_handle_->getNodeState();

        std::vector<std::string> output_uuids = state->getDictionaryEntry<std::vector<std::string>>("forwarding_outputs", {});
        // delete save entries -> will now be replaced.
        state->deleteDictionaryEntry("forwarding_outputs");

        for(const std::string& output_name : output_uuids) {
            ConnectionTypeConstPtr type = MessageFactory::createMessage(state->getDictionaryEntry<std::string>(output_name + "_type"));

            UUID internal = makeUUID(state->getDictionaryEntry<std::string>(output_name + "_uuid_internal"));
            UUID external = node_handle_->getUUIDProvider()->makeDerivedUUID(getUUID(), state->getDictionaryEntry<std::string>(output_name + "_uuid_external"));

            addForwardingOutput(internal, external, type,
                                state->getDictionaryEntry<std::string>(output_name + "_label"));
        }


        std::vector<std::string> input_uuids = state->getDictionaryEntry<std::vector<std::string>>("forwarding_inputs", {});
        // delete save entries -> will now be replaced.
        state->deleteDictionaryEntry("forwarding_inputs");

        for(const std::string& input_name : input_uuids) {
            ConnectionTypeConstPtr type = MessageFactory::createMessage(state->getDictionaryEntry<std::string>(input_name + "_type"));

            UUID internal = makeUUID(state->getDictionaryEntry<std::string>(input_name + "_uuid_internal"));
            UUID external = node_handle_->getUUIDProvider()->makeDerivedUUID(getUUID(), state->getDictionaryEntry<std::string>(input_name + "_uuid_external"));

            addForwardingInput(internal, external, type,
                               state->getDictionaryEntry<std::string>(input_name + "_label"),
                               state->getDictionaryEntry<bool>(input_name + "_optional"));
        }

        is_initialized_ = true;
    }
}

void Graph::clear()
{
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


bool Graph::addConnection(ConnectionPtr connection)
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

            buildConnectedComponents();
            verify();
        }
    }

    connectionAdded(connection.get());
    return true;
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
            verify();

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
        for(const auto& input : current->getAllInputs()) {
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

        for(auto output : node->getAllOutputs()) {
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

            for(auto input : current->getAllInputs()) {
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
            for(auto input : node->getAllInputs()) {
                if(input->isDynamic()) {
                    DynamicInput* di = dynamic_cast<DynamicInput*>(input.get());
                    di->setCorrespondent(correspondent);
                    correspondent->addCorrespondent(di);
                }
            }
        }
    }

}

void Graph::verify()
{
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
    NodeHandle* owner = findNodeHandle(uuid.parentUUID());
    apex_assert_hard(owner);

    std::string type = uuid.type();

    Connectable* result = owner->getConnector(uuid);
    apex_assert_hard(result);

    return result;
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

void Graph::setup(NodeModifier &/*modifier*/)
{

}

void Graph::process(NodeModifier &node_modifier, Parameterizable &params,
                    std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation)
{
    continuation_ = continuation;

    apex_assert_hard(transition_relay_out_->canStartSendingMessages());
    for(Input* i : node_modifier.getMessageInputs()) {
        ConnectionTypeConstPtr m = msg::getMessage(i);
        OutputPtr o = forward_inputs_.at(i);

        msg::publish(o.get(), m);
    }
    transition_relay_out_->sendMessages();

    //    continuation_ = continuation;

    //    received_.clear();
    //    for(Output* o : node_modifier.getMessageOutputs()) {
    //        received_[o] = false;
    //    }

    //    for(Input* i : node_modifier.getMessageInputs()) {
    //        ConnectionTypeConstPtr m = msg::getMessage(i);
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

std::pair<UUID, UUID> Graph::addForwardingInput(const ConnectionTypeConstPtr& type,
                                                const std::string& label, bool optional)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayout");
    UUID external_uuid = node_handle_->getUUIDProvider()->generateDerivedUUID(getUUID(), "in");

    return addForwardingInput(internal_uuid, external_uuid, type, label, optional);
}

std::pair<UUID, UUID> Graph::addForwardingInput(const UUID& internal_uuid, const UUID& external_uuid,
                                                const ConnectionTypeConstPtr& type,
                                                const std::string& label, bool optional)
{
    registerUUID(internal_uuid);
    node_handle_->getUUIDProvider()->registerUUID(external_uuid);

    InputPtr external_input = std::make_shared<Input>(external_uuid);
    external_input->setLabel(label);
    external_input->setOptional(optional);
    external_input->setType(type);

    node_handle_->addInput(external_input);

    OutputPtr relay = std::make_shared<StaticOutput>(internal_uuid);
    relay->setType(type);
    relay->setLabel(label);

    relay->connectionInProgress.connect(internalConnectionInProgress);

    transition_relay_out_->addOutput(relay);

    forward_inputs_[external_input.get()] = relay;

    relay_to_external_input_[internal_uuid] = external_uuid;


    NodeStatePtr state = node_handle_->getNodeState();

    std::string name = relay->getUUID().getFullName();

    std::vector<std::string> input_uuids = state->getDictionaryEntry<std::vector<std::string>>("forwarding_inputs", {});
    input_uuids.push_back(name);
    state->setDictionaryEntry("forwarding_inputs", input_uuids);

    state->setDictionaryEntry(name + "_uuid_external", external_input->getUUID().id());
    state->setDictionaryEntry(name + "_uuid_internal", name);
    state->setDictionaryEntry(name + "_type", external_input->getType()->typeName());
    state->setDictionaryEntry(name + "_label", external_input->getLabel());
    state->setDictionaryEntry(name + "_optional", external_input->isOptional());

    forwardingAdded(relay);

    return {external_uuid, internal_uuid};
}

std::pair<UUID, UUID> Graph::addForwardingOutput(const ConnectionTypeConstPtr& type,
                                                 const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayin");
    UUID external_uuid = node_handle_->getUUIDProvider()->generateDerivedUUID(getUUID(), "out");

    return addForwardingOutput(internal_uuid, external_uuid, type, label);
}

std::pair<UUID, UUID> Graph::addForwardingOutput(const UUID& internal_uuid, const UUID& external_uuid,
                                                 const ConnectionTypeConstPtr& type,
                                                 const std::string& label)
{
    registerUUID(internal_uuid);
    node_handle_->getUUIDProvider()->registerUUID(external_uuid);

    OutputPtr external_output = std::make_shared<StaticOutput>(external_uuid);
    external_output->setLabel(label);
    external_output->setType(type);


    node_handle_->addOutput(external_output);

    InputPtr relay = std::make_shared<Input>(internal_uuid);
    relay->setType(type);
    relay->setLabel(label);
    relay->setOptional(true);

    relay->connectionInProgress.connect(internalConnectionInProgress);

    forward_outputs_[external_output.get()] = relay;

    transition_relay_in_->addInput(relay);

    relay_to_external_output_[internal_uuid] = external_uuid;


    NodeStatePtr state = node_handle_->getNodeState();

    std::string name = relay->getUUID().getFullName();

    std::vector<std::string> output_uuids = state->getDictionaryEntry<std::vector<std::string>>("forwarding_outputs", {});
    output_uuids.push_back(name);
    state->setDictionaryEntry("forwarding_outputs", output_uuids);

    state->setDictionaryEntry(name + "_uuid_external", external_output->getUUID().id());
    state->setDictionaryEntry(name + "_uuid_internal", name);
    state->setDictionaryEntry(name + "_type", external_output->getType()->typeName());
    state->setDictionaryEntry(name + "_label", external_output->getLabel());

    forwardingAdded(relay);

    std::weak_ptr<Output> external_output_weak = external_output;
    relay->messageArrived.connect([this, external_output_weak, relay](Connectable*) {
        if(auto external_output = external_output_weak.lock()) {
            msg::publish(external_output.get(), relay->getMessage());
        }
    });

    return {external_uuid, internal_uuid};
}

std::pair<UUID, UUID> Graph::addForwardingSlot(const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relaytrigger");
    UUID external_uuid = node_handle_->getUUIDProvider()->generateDerivedUUID(getUUID(), "slot");

    return addForwardingSlot(internal_uuid, external_uuid, label);
}

std::pair<UUID, UUID> Graph::addForwardingSlot(const UUID& internal_uuid, const UUID& external_uuid,
                                                  const std::string& label)
{
    registerUUID(internal_uuid);
    node_handle_->getUUIDProvider()->registerUUID(external_uuid);


    TriggerPtr relay = std::make_shared<Trigger>(internal_uuid);
    relay->setLabel(label);

    relay->connectionInProgress.connect(internalConnectionInProgress);

    relay_trigger_[internal_uuid] = relay;

    auto cb = [this, relay]() {
        relay->trigger();
    };

    SlotPtr external_slot = std::make_shared<Slot>(cb, external_uuid, false);
    external_slot->setLabel(label);

    node_handle_->addSlot(external_slot);
    forward_slots_[external_slot.get()] = relay;

    relay_to_external_slot_[internal_uuid] = external_uuid;


    NodeStatePtr state = node_handle_->getNodeState();

    std::string name = relay->getUUID().getFullName();

    std::vector<std::string> output_uuids = state->getDictionaryEntry<std::vector<std::string>>("forwarding_slots", {});
    output_uuids.push_back(name);
    state->setDictionaryEntry("forwarding_slots", output_uuids);

    state->setDictionaryEntry(name + "_uuid_external", external_slot->getUUID().id());
    state->setDictionaryEntry(name + "_uuid_internal", name);
    state->setDictionaryEntry(name + "_type", external_slot->getType()->typeName());
    state->setDictionaryEntry(name + "_label", external_slot->getLabel());

    forwardingAdded(relay);

    return {external_uuid, internal_uuid};
}


std::pair<UUID, UUID> Graph::addForwardingTrigger(const std::string& label)
{
    UUID internal_uuid = generateDerivedUUID(UUID(),"relayslot");
    UUID external_uuid = node_handle_->getUUIDProvider()->generateDerivedUUID(getUUID(), "trigger");

    return addForwardingTrigger(internal_uuid, external_uuid, label);
}

std::pair<UUID, UUID> Graph::addForwardingTrigger(const UUID& internal_uuid, const UUID& external_uuid,
                                                  const std::string& label)
{
    registerUUID(internal_uuid);
    node_handle_->getUUIDProvider()->registerUUID(external_uuid);

    TriggerPtr external_trigger = std::make_shared<Trigger>(external_uuid);
    external_trigger->setLabel(label);

    node_handle_->addTrigger(external_trigger);

    auto cb = [this, external_trigger](){
        external_trigger->trigger();
    };

    SlotPtr relay = std::make_shared<Slot>(cb, internal_uuid, false);
    relay->setLabel(label);

    Slot* slot = relay.get();

    relay->connectionInProgress.connect(internalConnectionInProgress);
    relay->triggered.connect([this, slot](Trigger* source) {
        node_handle_->executionRequested([this, slot, source]() {
            slot->handleTrigger();
            source->signalHandled(slot);
        });
    });

    relay_slot_[internal_uuid] = relay;
    forward_triggers_[external_trigger.get()] = relay;

    relay_to_external_trigger_[internal_uuid] = external_uuid;


    NodeStatePtr state = node_handle_->getNodeState();

    std::string name = relay->getUUID().getFullName();

    std::vector<std::string> output_uuids = state->getDictionaryEntry<std::vector<std::string>>("forwarding_triggers", {});
    output_uuids.push_back(name);
    state->setDictionaryEntry("forwarding_triggers", output_uuids);

    state->setDictionaryEntry(name + "_uuid_external", external_trigger->getUUID().id());
    state->setDictionaryEntry(name + "_uuid_internal", name);
    state->setDictionaryEntry(name + "_type", external_trigger->getType()->typeName());
    state->setDictionaryEntry(name + "_label", external_trigger->getLabel());

    forwardingAdded(relay);

    return {external_uuid, internal_uuid};
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
    return relay_slot_.at(internal_uuid);
}

TriggerPtr Graph::getForwardedTriggerInternal(const UUID &internal_uuid) const
{
    return relay_trigger_.at(internal_uuid);
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

UUID Graph::getForwardedTriggerExternal(const UUID &internal_uuid) const
{
    return relay_to_external_trigger_.at(internal_uuid);
}

std::vector<UUID> Graph::getRelayOutputs() const
{
    std::vector<UUID> res;
    for(const auto& pair : forward_inputs_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}
std::vector<UUID> Graph::getRelayInputs() const
{
    std::vector<UUID> res;
    for(const auto& pair : forward_outputs_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}
std::vector<UUID> Graph::getRelaySlots() const
{
    std::vector<UUID> res;
    for(const auto& pair : forward_slots_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}
std::vector<UUID> Graph::getRelayTriggers() const
{
    std::vector<UUID> res;
    for(const auto& pair : forward_triggers_) {
        res.push_back(pair.second->getUUID());
    }
    return res;
}

void Graph::notifyMessagesProcessed()
{
    GeneratorNode::notifyMessagesProcessed();

    if(output_active_) {
        transition_relay_in_->notifyMessageProcessed();
        output_active_ = false;
    }
}

void Graph::inputActivation()
{
    if(!transition_relay_in_->hasEstablishedConnection()) {
        if(continuation_) {
            continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
            continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();
        }
    }
}

void Graph::outputActivation()
{
    if(transition_relay_in_->isEnabled() && node_handle_->getOutputTransition()->canStartSendingMessages()) {
        if(node_handle_->isSource() || continuation_) {
            publishSubGraphMessages();
        }
    }
}

void Graph::publishSubGraphMessages()
{
    apex_assert_hard(transition_relay_in_->isEnabled());
    apex_assert_hard(node_handle_->getOutputTransition()->canStartSendingMessages());

    //        node_handle_->executionRequested([this](){
    transition_relay_in_->forwardMessages();

    apex_assert_hard(!output_active_);
    output_active_ = true;

    if(node_handle_->isSource()) {
        updated();

    } else {
        apex_assert_hard(continuation_);
        continuation_([](csapex::NodeModifier& node_modifier, Parameterizable &parameters){});
        continuation_ = std::function<void (std::function<void (csapex::NodeModifier&, Parameterizable &)>)>();
    }
    //        });

    if(node_handle_->isSink()) {
        notifyMessagesProcessed();
    }
}

std::string Graph::makeStatusString() const
{
    std::stringstream ss;
    ss << "UUID: " << getUUID() << '\n';
    ss << "AUUID: " << getUUID().getAbsoluteUUID() << '\n';
    ss << "output_active_: " << output_active_ << '\n';
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
    ss << " - " << (transition_relay_out_->hasEstablishedConnection() ? "has" : "doesn't have") <<  " established connection\n";
    ss << " - outputs are " << (transition_relay_out_->areOutputsIdle() ? "idle" : "busy") << '\n';
    ss << "(right) transition_relay_in:\n";
    ss << " - " << (transition_relay_in_->isEnabled() ? "enabled" : "disabled") << '\n';
    ss << " - established connections: ";
    for(const ConnectionPtr& c : transition_relay_in_->getEstablishedConnections()) {
        printStatus(c);
        ss << '\t';
    }
    ss << '\n';
    ss << " - fading connections: ";
    for(const ConnectionPtr& c : transition_relay_in_->getFadingConnections()) {
        printStatus(c);
        ss << '\t';
    }
    ss << '\n';

    return ss.str();
}
