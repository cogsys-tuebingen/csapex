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
#include <csapex/model/node_worker.h>
#include <csapex/utility/timer.h>

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
    for(ConnectionPtr c : getConnections()) {
        deleteConnection(c);
    }

    for(NodeWorker* node : getAllNodeWorkers()) {
        deleteNode(node->getUUID());
    }

    uuids_.clear();
    connections_.clear();

    nodes_.clear();
    node_component_.clear();
    node_level_.clear();

    node_parents_.clear();
    node_children_.clear();
}

std::string Graph::makeUUIDPrefix(const std::string& name)
{
    int& last_id = uuids_[name];
    ++last_id;

    std::stringstream ss;
    ss << name << "_" << last_id;

    return ss.str();
}

void Graph::addNode(NodeWorker::Ptr node_worker)
{
    nodes_.push_back(node_worker);
    node_parents_[node_worker.get()] = std::vector<NodeWorker*>();
    node_children_[node_worker.get()] = std::vector<NodeWorker*>();

    buildConnectedComponents();

    node_worker->checkParameters();

    node_worker->panic.connect(panic);

    nodeAdded(node_worker);
}

std::vector<ConnectionPtr> Graph::getConnections()
{
    return connections_;
}

void Graph::deleteNode(const UUID& uuid)
{
    NodeWorker* node_worker = findNodeWorker(uuid);

    node_worker->stop();

    /// assert that all connections have already been deleted
    apex_assert_hard(node_parents_[node_worker].empty());
    apex_assert_hard(node_children_[node_worker].empty());

    node_parents_.erase(node_worker);
    node_children_.erase(node_worker);

    NodeWorker::Ptr removed;

    for(std::vector<NodeWorker::Ptr>::iterator it = nodes_.begin(); it != nodes_.end();) {
        if((*it)->getUUID() == uuid) {
            removed = *it;
            it = nodes_.erase(it);

        } else {
            ++it;
        }
    }

    if(removed) {
        nodeRemoved(removed);
        buildConnectedComponents();
    }
}

int Graph::countNodes()
{
    return nodes_.size();
}



void Graph::foreachNode(std::function<void (NodeWorker*)> f)
{
    for(NodeWorker::Ptr b : nodes_) {
        f(b.get());
    }
}

void Graph::foreachNode(std::function<void (NodeWorker*)> f, std::function<bool (NodeWorker*)> pred)
{
    for(NodeWorker::Ptr b : nodes_) {
        if(pred(b.get())) {
            f(b.get());
        }
    }
}

bool Graph::addConnection(ConnectionPtr connection)
{
    NodeWorker* n_from = findNodeWorkerForConnector(connection->from()->getUUID());
    NodeWorker* n_to = findNodeWorkerForConnector(connection->to()->getUUID());

    apex_assert_hard(connection->from()->isConnectionPossible(connection->to()));

    Connectable* from = findConnector(connection->from()->getUUID());
    Connectable* to = findConnector(connection->to()->getUUID());

    connections_.push_back(connection);
    from->addConnection(connection);
    to->addConnection(connection);


    if(n_to != n_from) {
        node_parents_[n_to].push_back(n_from);
        node_children_[n_from].push_back(n_to);

        buildConnectedComponents();
        verify();
    }

    connectionAdded(connection.get());
    from->connection_added_to(from);
    to->connection_added_to(to);
    return true;
}

void Graph::deleteConnection(ConnectionPtr connection)
{
    if(!connection) {
        return;
    }
    auto out = connection->from();
    auto in = connection->to();

    out->removeConnection(in);

    out->fadeConnection(connection);
    in->fadeConnection(connection);

    for(std::vector<ConnectionPtr>::iterator c = connections_.begin(); c != connections_.end();) {
        if(*connection == **c) {
            Connectable* to = connection->to();
            to->setError(false);

            NodeWorker* n_from = findNodeWorkerForConnector(connection->from()->getUUID());
            NodeWorker* n_to = findNodeWorkerForConnector(connection->to()->getUUID());

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

    std::deque<NodeWorker*> unmarked;
    for(NodeWorker::Ptr node : nodes_) {
        unmarked.push_back(node.get());
        node_component_[node.get()] = -1;
    }

    std::deque<NodeWorker*> Q;
    int component = 0;
    while(!unmarked.empty()) {
        // take a random unmarked node to start bfs from
        NodeWorker* start = unmarked.front();
        Q.push_back(start);

        node_component_[start] = component;

        while(!Q.empty()) {
            NodeWorker* front = Q.front();
            Q.pop_front();

            unmarked.erase(std::find(unmarked.begin(), unmarked.end(), front));

            // iterate all neighbors
            std::vector<NodeWorker*> neighbors;
            for(NodeWorker* parent : node_parents_[front]) {
                neighbors.push_back(parent);
            }
            for(NodeWorker* child : node_children_[front]) {
                neighbors.push_back(child);
            }

            for(NodeWorker* neighbor : neighbors) {
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
    std::map<NodeWorker*, int> node_level;

    static const int NO_LEVEL = std::numeric_limits<int>::min();

    std::deque<NodeWorker*> unmarked;
    for(NodeWorker::Ptr node : nodes_) {
        if(node->isSource() && node->isSink()) {
            node_level[node.get()] = 0;
        } else if(node_parents_[node.get()].empty()) {
            node_level[node.get()] = 0;
        } else {
            node_level[node.get()] = NO_LEVEL;
            unmarked.push_back(node.get());
        }
    }

    std::deque<NodeWorker*> gateways;

    // to assign a level, every parent must be known
    while(!unmarked.empty()) {
        NodeWorker* current = unmarked.front();
        unmarked.pop_front();

        int max_level = NO_LEVEL;
        for(NodeWorker* parent : node_parents_.at(current)) {
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
                    NodeWorker* node = findNodeWorkerForConnector(parent_output->getUUID());
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

    for(NodeWorker::Ptr node : nodes_) {
        node->setLevel(node_level[node.get()]);

        for(auto output : node->getAllInputs()) {
            if(output->isDynamic()) {
                DynamicOutput* dout = dynamic_cast<DynamicOutput*>(output.get());
                dout->clearCorrespondents();
            }
        }
    }

    for(NodeWorker* node : gateways) {
        DynamicOutput* correspondent = nullptr;

        // perform bfs to find the parent with a dynamic output
        std::deque<NodeWorker*> Q;
        std::set<NodeWorker*> visited;
        Q.push_back(node);
        while(!Q.empty()) {
            NodeWorker* current = Q.front();
            Q.pop_front();
            visited.insert(current);

            for(auto input : current->getAllInputs()) {
                if(input->isConnected()) {
                    ConnectionPtr connection = input->getConnections().front();
                    Output* out = dynamic_cast<Output*>(connection->from());
                    if(out) {
                        NodeWorker* parent = findNodeWorkerForConnector(out->getUUID());

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
    NodeWorker* node = findNodeWorkerNoThrow(node_uuid);
    if(!node) {
        return -1;
    }

    return node_component_.at(node);
}

int Graph::getLevel(const UUID &node_uuid) const
{
    NodeWorker* node = findNodeWorkerNoThrow(node_uuid);
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

NodeWorker* Graph::findNodeWorker(const UUID& uuid) const
{
    NodeWorker* node_worker = findNodeWorkerNoThrow(uuid);
    if(node_worker) {
        return node_worker;
    }
    throw NodeWorkerNotFoundException(uuid.getFullName());
}

Node* Graph::findNodeNoThrow(const UUID& uuid) const
{
    for(NodeWorker::Ptr worker : nodes_) {
        if(worker->getUUID() == uuid) {
            auto node = worker->getNode().lock();
            if(node) {
                return node.get();
            }
        }
    }

    return nullptr;
}


NodeWorker* Graph::findNodeWorkerNoThrow(const UUID& uuid) const
{
    for(const NodeWorker::Ptr b : nodes_) {
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

NodeWorker* Graph::findNodeWorkerForConnector(const UUID &uuid) const
{
    try {
        return findNodeWorker(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find worker of connector \"") + uuid.getFullName());
    }
}

std::vector<NodeWorker*> Graph::getAllNodeWorkers()
{
    std::vector<NodeWorker*> node_workers;
    for(const NodeWorkerPtr& node : nodes_) {
        node_workers.push_back(node.get());
    }

    return node_workers;
}

Connectable* Graph::findConnector(const UUID &uuid)
{
    NodeWorker* owner = findNodeWorker(uuid.parentUUID());
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
