/// HEADER
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/msg/dynamic_input.h>
#include <csapex/msg/dynamic_output.h>
#include <csapex/signal/slot.h>
#include <csapex/signal/trigger.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_modifier.h>
#include <csapex/utility/timer.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input.h>
#include <csapex/msg/static_output.h>
#include <csapex/msg/bundled_connection.h>

using namespace csapex;

Graph::Graph()
{
}

Graph::~Graph()
{
    clear();
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

    /// assert that all connections have already been deleted
    apex_assert_hard(node_parents_[node_handle].empty());
    apex_assert_hard(node_children_[node_handle].empty());

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


    if(n_to != n_from) {
        node_parents_[n_to].push_back(n_from);
        node_children_[n_from].push_back(n_to);

        buildConnectedComponents();
        verify();
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

            NodeHandle* n_from = findNodeHandleForConnector(connection->from()->getUUID());
            NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

            // erase pointer from TO to FROM
            if(n_from != n_to) {
                // if there are multiple edges, this only erases one entry
                node_parents_[n_to].erase(std::find(node_parents_[n_to].begin(), node_parents_[n_to].end(), n_from));

                // erase pointer from FROM to TO
                node_children_[n_from].erase(std::find(node_children_[n_from].begin(), node_children_[n_from].end(), n_to));
            }
            connections_.erase(c);

            buildConnectedComponents();
            verify();
            connectionDeleted(connection.get());
            stateChanged();

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
    NodeHandle* node_handle = findNodeHandleNoThrow(uuid);
    if(node_handle) {
        return node_handle;
    }
    throw NodeHandleNotFoundException(uuid.getFullName());
}

Node* Graph::findNodeNoThrow(const UUID& uuid) const noexcept
{
    for(auto nh : nodes_) {
        if(nh->getUUID() == uuid) {
            auto node = nh->getNode().lock();
            if(node) {
                return node.get();
            }
        }
    }

    return nullptr;
}


NodeHandle* Graph::findNodeHandleNoThrow(const UUID& uuid) const noexcept
{
    for(const auto b : nodes_) {
        if(b->getUUID() == uuid) {
            return b.get();
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

    Connectable* result;
    if(type == "in") {
        result = owner->getInput(uuid);
    } else if(type == "out") {
        result = owner->getOutput(uuid);
    } else if(type == "slot") {
        result = owner->getSlot(uuid);
    } else if(type == "trigger") {
        result = owner->getTrigger(uuid);
    } else {
        throw std::logic_error(std::string("the connector type '") + type + "' is unknown.");
    }

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
    for(ConnectionPtr& connection : connections_) {
        if(connection->from() == from && connection->to() == to) {
            return connection;
        }
    }

    std::cerr << "error: cannot get connection from " << from->getUUID() << " to " << to->getUUID() << std::endl;

    return nullptr;
}

ConnectionPtr Graph::getConnection(const UUID &from, const UUID &to)
{
    for(ConnectionPtr& connection : connections_) {
        if(connection->from()->getUUID() == from && connection->to() ->getUUID()== to) {
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

void Graph::process(Parameterizable &params,  std::function<void (std::function<void ()>)> continuation)
{
    continuation_ = continuation;

    received_.clear();
    for(Output* o : modifier_->getMessageOutputs()) {
        received_[o] = false;
    }

    for(Input* i : modifier_->getMessageInputs()) {
        ConnectionTypeConstPtr m = msg::getMessage(i);
        OutputPtr o = pass_on_inputs_[i];

        msg::publish(o.get(), m);
        o->commitMessages();
        o->publish();
    }
}

bool Graph::isAsynchronous() const
{
    return true;
}

Input* Graph::passOutInput(Input *internal)
{
    Input* parent = modifier_->addInput(internal->getType(), internal->getLabel(), false, false);

    std::string name = "relay" + std::to_string(pass_on_inputs_.size());
    OutputPtr relay = std::make_shared<StaticOutput>(makeDerivedUUID(modifier_->getUUID(),name));
    relay->setType(internal->getType());
    relay->setLabel(internal->getLabel());

    NodeHandle* nh = findNodeHandleForConnector(internal->getUUID());
    BundledConnection::connect(relay.get(), internal, nh->getInputTransition());
    pass_on_inputs_[parent] = relay;

    return parent;
}


Output* Graph::passOutOutput(Output *internal)
{
    Output* parent = modifier_->addOutput(internal->getType(), internal->getLabel(), false);

    pass_on_outputs_[internal] = parent;

    internal->messageSent.connect([this, parent, internal](Connectable*) {
        msg::publish(parent, internal->getMessage());
        received_[parent] = true;

        for(auto pair : received_) {
            if(!pair.second) {
                return;
            }
        }

        continuation_([](){});
    });

    return parent;
}
