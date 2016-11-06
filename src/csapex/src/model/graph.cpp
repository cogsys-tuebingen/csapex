/// HEADER
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/model/connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/graph/edge.h>

/// SYSTEM
#include <iostream>

using namespace csapex;

Graph::Graph()
{
}

Graph::~Graph()
{
    clear();
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

    auto vertecies = vertecies_;
    for(graph::VertexPtr vertex : vertecies) {
        NodeHandlePtr node = vertex->getNodeHandle();
        node->setActive(false);
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

    auto vertecies = vertecies_;
    for(graph::VertexPtr vertex : vertecies) {
        NodeHandlePtr node = vertex->getNodeHandle();
        deleteNode(node->getUUID(), true);
    }
    apex_assert_hard(vertecies_.empty());

    node_component_.clear();

    node_parents_.clear();
    node_children_.clear();
}

void Graph::addNode(NodeHandlePtr nh)
{
    apex_assert_hard_msg(nh, "NodeHandle added is not null");
    graph::VertexPtr vertex = std::make_shared<graph::Vertex>(nh);
    vertecies_.push_back(vertex);

    nh->setVertex(vertex);

    sources_.insert(vertex.get());
    sinks_.insert(vertex.get());

    node_parents_[vertex.get()] = std::vector<graph::Vertex*>();
    node_children_[vertex.get()] = std::vector<graph::Vertex*>();

    analyzeGraph();

    vertex_added(vertex);
}

std::vector<ConnectionPtr> Graph::getConnections()
{
    return connections_;
}

void Graph::deleteNode(const UUID& uuid, bool quiet)
{
    NodeHandle* node_handle = findNodeHandle(uuid);
    node_handle->stop();

    graph::VertexPtr vertex = node_handle->getVertex();

    node_parents_[vertex.get()].clear();
    node_children_[vertex.get()].clear();

    node_parents_.erase(vertex.get());
    node_children_.erase(vertex.get());

    graph::VertexPtr removed;

    for(auto it = vertecies_.begin(); it != vertecies_.end();) {
        NodeHandlePtr node = (*it)->getNodeHandle();
        if(node->getUUID() == uuid) {
            removed = *it;
            it = vertecies_.erase(it);

        } else {
            ++it;
        }
    }

    apex_assert_hard(removed);

    if(removed) {
        //        if(NodePtr node = removed->getNode().lock()) {
        //            if(GraphPtr child = std::dynamic_pointer_cast<Graph>(node)) {
        //                child->clear();
        //            }
        //        }

        if(!quiet) {
            vertex_removed(removed);
            analyzeGraph();
        }
    }
}

int Graph::countNodes()
{
    return vertecies_.size();
}


bool Graph::addConnection(ConnectionPtr connection, bool quiet)
{
    apex_assert_hard(connection);
    connections_.push_back(connection);

    if(!dynamic_cast<Event*>(connection->from()) && !dynamic_cast<Slot*>(connection->to())) {
        NodeHandle* n_from = findNodeHandleForConnector(connection->from()->getUUID());
        NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

        if(n_to != n_from) {
            apex_assert_hard(n_to->getUUID().getAbsoluteUUID() != n_from->getUUID().getAbsoluteUUID());

            graph::VertexPtr v_from = n_from->getVertex();
            graph::VertexPtr v_to = n_to->getVertex();

            node_parents_[v_to.get()].push_back(v_from.get());
            node_children_[v_from.get()].push_back(v_to.get());

            sources_.erase(v_to.get());
            sinks_.erase(v_from.get());

            if(!quiet) {
                analyzeGraph();
            }
        }
    }

    if(!quiet) {
        connectionAdded(connection.get());
    }
    return true;
}

void Graph::deleteConnection(ConnectionPtr connection, bool quiet)
{
    apex_assert_hard(connection);

    if(connection->isDetached()) {
        auto c = std::find(connections_.begin(), connections_.end(), connection);
        if(c != connections_.end()) {
            connections_.erase(c);
        }
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

            UUID from_uuid = connection->from()->getUUID();
            NodeHandle* n_from = findNodeHandleForConnector(from_uuid);
            NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

            if(!dynamic_cast<Event*>(connection->from()) && !dynamic_cast<Slot*>(connection->to())) {
                // erase pointer from TO to FROM
                if(n_from != n_to) {
                    graph::VertexPtr v_from = n_from->getVertex();
                    graph::VertexPtr v_to = n_to->getVertex();

                    // if there are multiple edges, this only erases one entry
                    node_parents_[v_to.get()].erase(std::find(node_parents_[v_to.get()].begin(), node_parents_[v_to.get()].end(), v_from.get()));

                    // erase pointer from FROM to TO
                    node_children_[v_from.get()].erase(std::find(node_children_[v_from.get()].begin(), node_children_[v_from.get()].end(), v_to.get()));

                    if(!n_from->getOutputTransition()->hasConnection()) {
                        sinks_.insert(v_from.get());
                    }
                    if(!n_to->getInputTransition()->hasConnection()) {
                        sources_.insert(v_to.get());
                    }

                }
            }
            connections_.erase(c);

            if(!quiet) {
                connectionDeleted(connection.get());
                state_changed();

                analyzeGraph();
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

void Graph::triggerConnectionsAdded()
{
    analyzeGraph();

    for(const ConnectionPtr& connection : connections_) {
        connectionAdded(connection.get());
    }
}

void Graph::analyzeGraph()
{
    buildConnectedComponents();

    calculateDepths();

    structureChanged(this);
}

void Graph::buildConnectedComponents()
{
    /* Find all connected sub components of this graph */
    node_component_.clear();

    std::deque<graph::Vertex*> unmarked;
    for(auto vertex : vertecies_) {
        unmarked.push_back(vertex.get());
        node_component_[vertex.get()] = -1;
    }

    std::deque<graph::Vertex*> Q;
    int component = 0;
    while(!unmarked.empty()) {
        // take a random unmarked node to start bfs from
        auto* start = unmarked.front();
        Q.push_back(start);

        node_component_[start] = component;

        while(!Q.empty()) {
            graph::Vertex* front = Q.front();
            Q.pop_front();

            checkNodeState(front->getNodeHandle().get());

            unmarked.erase(std::find(unmarked.begin(), unmarked.end(), front));

            // iterate all neighbors
            std::vector<graph::Vertex*> neighbors;
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
}

void Graph::calculateDepths()
{
    // start DFSs at each source. assign each node:
    // - depth: the minimum distance to any source
    // - joining: true, iff more than one path leads from any source to a node

    // initialize
    for(const graph::VertexPtr& vertex: vertecies_) {
        vertex->getNodeCharacteristics().is_joining_vertex = false;
        vertex->getNodeCharacteristics().is_joining_vertex_counterpart = false;

        vertex->getNodeCharacteristics().is_combined_by_joining_vertex = false;
        vertex->getNodeCharacteristics().is_leading_to_joining_vertex = false;
    }

    std::set<graph::Vertex*> joins;

    node_depth_.clear();
    // init node_depth_ and find merging nodes
    for(const graph::Vertex* source : sources_) {
        node_depth_[source] = 0;

        std::deque<const graph::Vertex*> Q;
        Q.push_back(source);
        while(!Q.empty()) {
            const graph::Vertex* top = Q.back();
            Q.pop_back();

            for(auto* child : node_children_[top]) {
                auto it = node_depth_.find(child);
                if(it == node_depth_.end()) {
                    node_depth_[child] = std::numeric_limits<int>::max();
                    Q.push_back(child);

                } else  {
                    // already visited
                    joins.insert(child);
                }
            }
        }
    }

    // populate node_depth_ with minimal depths
    for(const graph::Vertex* source : sources_) {
        node_depth_[source] = 0;

        std::deque<const graph::Vertex*> Q;
        Q.push_back(source);
        while(!Q.empty()) {
            const graph::Vertex* top = Q.back();
            Q.pop_back();

            int depth = node_depth_.at(top);
            for(auto* child : node_children_[top]) {
                auto it = node_depth_.find(child);
                if(it == node_depth_.end()) {
                    node_depth_[child] = depth + 1;
                    Q.push_back(child);

                } else  {
                    // already visited
                    if(depth + 1 < it->second) {
                        it->second = depth + 1;
                        Q.push_back(child);
                    }
                }
            }
        }
    }

    for(const auto& pair : node_depth_) {
        const graph::Vertex* vertex = pair.first;
        int depth = pair.second;
        vertex->getNodeCharacteristics().depth = depth;

        //        int min_child_depth = std::numeric_limits<int>::max();
        //        for(NodeHandle* child : node_children_.at(nh)) {
        //            min_child_depth = std::min(min_child_depth, node_depth_.at(child));
        //        }
        //    nh->getNodeCharacteristics().is_vertex_separator = depth < min_child_depth;

    }

    for(graph::Vertex* vertex : joins) {
        vertex->getNodeCharacteristics().is_joining_vertex = true;

        // find counterparts
        //        int level = node_depth_.at(nh);

        //        const auto& parents = node_parents_.at(nh);
        //        int min_level = std::accumulate(parents.begin(), parents.end(), std::numeric_limits<int>::max(),
        //                                        [this](int level, NodeHandle* parent){ return std::min(level, node_depth_.at(parent)); });

        int min_level = vertex->getNodeCharacteristics().depth;

        // iterate all parents, until we find a single common parent
        // if no such parent exists, this joining node has no counterpart

        struct CompareNH: public std::binary_function<graph::Vertex*, graph::Vertex*, bool> {
            bool operator()(const graph::Vertex* lhs, const graph::Vertex* rhs) const {
                return lhs->getNodeCharacteristics().depth > rhs->getNodeCharacteristics().depth;
            }
        };
        //std::set<NodeHandle*, CompareNH> Q(parents.begin(), parents.end());
        std::deque<graph::Vertex*> Q({ vertex });
        std::set<graph::Vertex*> done;
        std::set<graph::Vertex*> minimum;

        bool is_closed = false;

        while(!Q.empty()) {
            graph::Vertex* top = *Q.begin();

            if(min_level > 0 && Q.size() == 1 && minimum.size() == 1) {
                // if the only minimum is not a source and the queue has only the current element -> closed
                is_closed  = true;
                (*minimum.begin())->getNodeCharacteristics().is_joining_vertex_counterpart = true;
                break;
            }
            done.insert(top);
            Q.erase(Q.begin());
            for(graph::Vertex* parent : node_parents_.at(top)) {
                if(done.find(parent) != done.end()) {
                    continue;
                }
                if(std::find(Q.begin(), Q.end(), parent) == Q.end()) {
                    Q.push_back(parent);
                    std::sort(Q.begin(), Q.end(), CompareNH());
                }

                int level = parent->getNodeCharacteristics().depth;
                if(level < min_level) {
                    min_level = level;
                    minimum.clear();
                }
                if(level == min_level){
                    minimum.insert(parent);
                }
            }
        }

        if(min_level == 0 && minimum.size() == 1) {
            // if exactly one source exists -> also closed
            is_closed = true;
            graph::Vertex* top = *minimum.begin();
            top->getNodeCharacteristics().is_joining_vertex_counterpart = true;
            done.erase(top);
        }

        for(graph::Vertex* top : done) {
            if(top != vertex) {
                top->getNodeCharacteristics().is_leading_to_joining_vertex = true;
            }
        }
        if(is_closed ){
            for(graph::Vertex* top : done) {
                if(top != vertex) {
                    top->getNodeCharacteristics().is_combined_by_joining_vertex = true;
                }
            }
        }

//        todo:
//        - remember for each of vertex join counterpart, which children are enclosed -> only send NoMessage to non-enclosed children.
    }

    for(graph::VertexPtr& vertex : vertecies_) {
        NodeCharacteristics& c = vertex->getNodeCharacteristics();
        c.is_vertex_separator = !c.is_leading_to_joining_vertex;
    }
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

    graph::VertexPtr vertex = node->getVertex();

    auto pos = node_component_.find(vertex.get());
    if(pos == node_component_.end()) {
        return -1;
    } else {
        return pos->second;
    }
}

int Graph::getDepth(const UUID &node_uuid) const
{
    NodeHandle* node = findNodeHandleNoThrow(node_uuid);
    if(!node) {
        return -1;
    }

    graph::VertexPtr vertex = node->getVertex();

    auto pos = node_depth_.find(vertex.get());
    if(pos == node_depth_.end()) {
        return -1;
    } else {
        return pos->second;
    }
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
        for(const auto vertex : vertecies_) {
            NodeHandlePtr nh = vertex->getNodeHandle();
            if(nh->getUUID() == uuid) {
                return nh.get();
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
    for(const auto vertex : vertecies_) {
        NodeHandlePtr nh = vertex->getNodeHandle();
        NodeStatePtr state = nh->getNodeState();
        if(state) {
            if(state->getLabel() == label) {
                return nh.get();
            }
        }
    }
    return nullptr;
}

std::vector<NodeHandle*> Graph::getAllNodeHandles()
{
    std::vector<NodeHandle*> node_handles;
    for(const auto& vertex : vertecies_) {
        NodeHandlePtr nh = vertex->getNodeHandle();
        node_handles.push_back(nh.get());
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

Graph::vertex_iterator Graph::beginVertices()
{
    return vertecies_.begin();
}

const Graph::vertex_const_iterator Graph::beginVertices() const
{
    return vertecies_.cbegin();
}


Graph::vertex_iterator Graph::endVertices()
{
    return vertecies_.end();
}

const Graph::vertex_const_iterator Graph::endVertices() const
{
    return vertecies_.cend();
}

