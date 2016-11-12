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
    : in_transaction_(false)
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
            t->setActivityModifier(ActivityModifier::NONE);
        }
    }

    auto vertecies = vertices_;
    for(graph::VertexPtr vertex : vertecies) {
        NodeHandlePtr node = vertex->getNodeHandle();
        node->setActive(false);
    }
}

void Graph::clear()
{
    UUIDProvider::clearCache();

    beginTransaction();

    auto connections = connections_;
    for(ConnectionPtr c : connections) {
        deleteConnection(c);
    }
    apex_assert_hard(connections_.empty());

    auto vertecies = vertices_;
    for(graph::VertexPtr vertex : vertecies) {
        NodeHandlePtr node = vertex->getNodeHandle();
        deleteNode(node->getUUID());
    }
    apex_assert_hard(vertices_.empty());

    finalizeTransaction();
}

void Graph::addNode(NodeHandlePtr nh)
{
    apex_assert_hard_msg(nh, "NodeHandle added is not null");
    graph::VertexPtr vertex = std::make_shared<graph::Vertex>(nh);
    vertices_.push_back(vertex);

    nh->setVertex(vertex);

    sources_.insert(vertex);
    sinks_.insert(vertex);

    vertex_added(vertex);
    if(!in_transaction_) {
        analyzeGraph();
    }

}

std::vector<ConnectionPtr> Graph::getConnections()
{
    return connections_;
}

void Graph::deleteNode(const UUID& uuid)
{
    NodeHandle* node_handle = findNodeHandle(uuid);
    node_handle->stop();


    graph::VertexPtr removed;

    for(auto it = vertices_.begin(); it != vertices_.end();) {
        NodeHandlePtr node = (*it)->getNodeHandle();
        if(node->getUUID() == uuid) {
            removed = *it;
            vertices_.erase(it);

            break;

        } else {
            ++it;
        }
    }

    sources_.erase(removed);
    sinks_.erase(removed);

    apex_assert_hard(removed);
    apex_assert_hard(removed == node_handle->getVertex());

    //        if(NodePtr node = removed->getNode().lock()) {
    //            if(GraphPtr child = std::dynamic_pointer_cast<Graph>(node)) {
    //                child->clear();
    //            }
    //        }

    vertex_removed(removed);
    if(!in_transaction_) {
        analyzeGraph();
    }
}

int Graph::countNodes()
{
    return vertices_.size();
}


bool Graph::addConnection(ConnectionPtr connection)
{
    apex_assert_hard(connection);
    connections_.push_back(connection);

    if(!std::dynamic_pointer_cast<Event>(connection->from()) && !std::dynamic_pointer_cast<Slot>(connection->to())) {
        NodeHandle* n_from = findNodeHandleForConnector(connection->from()->getUUID());
        NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

        if(n_to != n_from) {
            apex_assert_hard(n_to->getUUID().getAbsoluteUUID() != n_from->getUUID().getAbsoluteUUID());

            graph::VertexPtr v_from = n_from->getVertex();
            graph::VertexPtr v_to = n_to->getVertex();

            v_from->addChild(v_to);
            v_to->addParent(v_from);

            sources_.erase(v_to);
            sinks_.erase(v_from);
        }
    }

    connection_added(connection.get());
    if(!in_transaction_) {
        analyzeGraph();
    }
    return true;
}

void Graph::deleteConnection(ConnectionPtr connection)
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

    out->removeConnection(in.get());

    out->fadeConnection(connection);
    in->fadeConnection(connection);

    for(std::vector<ConnectionPtr>::iterator c = connections_.begin(); c != connections_.end();) {
        if(*connection == **c) {
            ConnectablePtr to = connection->to();
            to->setError(false);

            UUID from_uuid = connection->from()->getUUID();
            NodeHandle* n_from = findNodeHandleForConnector(from_uuid);
            NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

            if(!std::dynamic_pointer_cast<Event>(connection->from()) && !std::dynamic_pointer_cast<Slot>(connection->to())) {
                // erase pointer from TO to FROM
                if(n_from != n_to) {
                    graph::VertexPtr v_from = n_from->getVertex();
                    graph::VertexPtr v_to = n_to->getVertex();

                    if(v_from && v_to) {

                        bool still_connected = false;
                        for(ConnectionPtr c : n_from->getOutputTransition()->getConnections()) {
                            if(c->isDetached()) {
                                continue;
                            }
                            ConnectablePtr to = c->to();
                            apex_assert_hard(to);
                            if(NodeHandlePtr child = std::dynamic_pointer_cast<NodeHandle>(to->getOwner())) {
                                if(child.get() == n_to) {
                                    still_connected = true;
                                    break;
                                }
                            }
                        }
                        if(!still_connected) {
                            v_to->removeParent(v_from.get());
                            v_from->removeChild(v_to.get());
                        }

                        if(!n_from->getOutputTransition()->hasConnection()) {
                            sinks_.insert(v_from);
                        }
                        if(!n_to->getInputTransition()->hasConnection()) {
                            sources_.insert(v_to);
                        }
                    }
                }
            }

            connections_.erase(c);

            connection_removed(connection.get());
            if(!in_transaction_) {
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

void Graph::beginTransaction()
{
    in_transaction_ = true;
}

void Graph::finalizeTransaction()
{
    in_transaction_ = false;

    analyzeGraph();
}

void Graph::analyzeGraph()
{
    buildConnectedComponents();

    calculateDepths();

    state_changed();
}

void Graph::buildConnectedComponents()
{
    /* Find all connected sub components of this graph */
    std::deque<graph::Vertex*> unmarked;
    for(auto vertex : vertices_) {
        unmarked.push_back(vertex.get());
        vertex->getNodeCharacteristics().component = -1;
    }

    std::deque<graph::Vertex*> Q;
    int component = 0;
    while(!unmarked.empty()) {
        // take a random unmarked node to start bfs from
        auto* start = unmarked.front();
        Q.push_back(start);

        start->getNodeCharacteristics().component = component;

        while(!Q.empty()) {
            graph::Vertex* front = Q.front();
            Q.pop_front();

            checkNodeState(front->getNodeHandle().get());

            auto it = std::find(unmarked.begin(), unmarked.end(), front);
            if(it == unmarked.end()) {
                continue;
            }
            unmarked.erase(it);

            // iterate all neighbors
            std::vector<graph::Vertex*> neighbors;
            for(auto parent : front->getParents()) {
                neighbors.push_back(parent.get());
            }
            for(auto child : front->getChildren()) {
                neighbors.push_back(child.get());
            }

            for(auto* neighbor : neighbors) {
                if(neighbor->getNodeCharacteristics().component == -1) {
                    neighbor->getNodeCharacteristics().component = component;
                    Q.push_back(neighbor);
                }
            }
        }

        ++component;
    }
}

std::set<graph::Vertex *> Graph::findVerticesThatJoinStreams()
{
    std::set<graph::Vertex*> joins;

    for(graph::VertexPtr vertex : vertices_) {
        vertex->getNodeCharacteristics().depth = -1;
    }

    // init node_depth_ and find merging nodes
    for(const graph::VertexPtr source : sources_) {
        source->getNodeCharacteristics().depth = 0;

        std::deque<const graph::Vertex*> Q;
        Q.push_back(source.get());
        while(!Q.empty()) {
            const graph::Vertex* top = Q.back();
            Q.pop_back();

            for(auto child : top->getChildren()) {
                if(child->getNodeCharacteristics().depth < 0) {
                    child->getNodeCharacteristics().depth = std::numeric_limits<int>::max();
                    Q.push_back(child.get());

                } else  {
                    // already visited -> joins two or more "streams"
                    joins.insert(child.get());
                }
            }
        }
    }

    return joins;
}

std::set<graph::Vertex *> Graph::findVerticesThatNeedMessages()
{
    std::set<graph::Vertex*> vertices_that_need_messages;

    for(const graph::VertexPtr v : vertices_) {
        for(const ConnectionPtr c : v->getNodeHandle()->getOutputTransition()->getConnections()) {
            if(c->to()->isVirtual()) {
                vertices_that_need_messages.insert(v.get());
                break;
            }
        }
    }

    return vertices_that_need_messages;
}

void Graph::calculateDepths()
{
    // start DFSs at each source. assign each node:
    // - depth: the minimum distance to any source
    // - joining: true, iff more than one path leads from any source to a node

    // initialize
    for(const graph::VertexPtr& vertex: vertices_) {
        NodeCharacteristics& characteristics =  vertex->getNodeCharacteristics();
        characteristics.is_joining_vertex = false;
        characteristics.is_joining_vertex_counterpart = false;

        characteristics.is_combined_by_joining_vertex = false;
        characteristics.is_leading_to_joining_vertex = false;
        characteristics.is_leading_to_essential_vertex = false;
    }

    std::set<graph::Vertex*> essentials = findVerticesThatNeedMessages();

    for(const graph::Vertex* essential : essentials) {
        essential->getNodeCharacteristics().is_leading_to_essential_vertex = true;

        std::deque<const graph::Vertex*> Q;
        Q.push_back(essential);
        while(!Q.empty()) {
            const graph::Vertex* top = Q.back();
            Q.pop_back();

            for(auto parent : top->getParents()) {
                NodeCharacteristics& characteristics = parent->getNodeCharacteristics();
                if(!characteristics.is_leading_to_essential_vertex) {
                    characteristics.is_leading_to_essential_vertex = true;
                    Q.push_back(parent.get());
                }
            }
        }
    }

    std::set<graph::Vertex*> joins = findVerticesThatJoinStreams();

    // populate node_depth_ with minimal depths
    for(const graph::VertexPtr source : sources_) {
        source->getNodeCharacteristics().depth = 0;

        std::deque<const graph::Vertex*> Q;
        Q.push_back(source.get());
        while(!Q.empty()) {
            const graph::Vertex* top = Q.back();
            Q.pop_back();

            int top_depth = top->getNodeCharacteristics().depth;
            for(auto child : top->getChildren()) {
                int& child_depth = child->getNodeCharacteristics().depth;
                if(child_depth == std::numeric_limits<int>::max()) {
                    child_depth = top_depth + 1;
                    Q.push_back(child.get());

                } else  {
                    // already visited
                    if(top_depth + 1 < child_depth) {
                        child_depth = top_depth + 1;
                        Q.push_back(child.get());
                    }
                }
            }
        }
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
            for(graph::VertexPtr parent : top->getParents()) {
                if(done.find(parent.get()) != done.end()) {
                    continue;
                }
                if(std::find(Q.begin(), Q.end(), parent.get()) == Q.end()) {
                    Q.push_back(parent.get());
                    // TODO: implement this with a better datastructure, maybe priority queue...
                    std::sort(Q.begin(), Q.end(), CompareNH());
                }

                int level = parent->getNodeCharacteristics().depth;
                if(level < min_level) {
                    min_level = level;
                    minimum.clear();
                }
                if(level == min_level){
                    minimum.insert(parent.get());
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

    for(graph::VertexPtr& vertex : vertices_) {
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
    return vertex->getNodeCharacteristics().component;
}

int Graph::getDepth(const UUID &node_uuid) const
{
    NodeHandle* node = findNodeHandleNoThrow(node_uuid);
    if(!node) {
        return -1;
    }

    graph::VertexPtr vertex = node->getVertex();
    return vertex->getNodeCharacteristics().depth;
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


NodeHandle *Graph::findNodeHandleNoThrow(const UUID& uuid) const noexcept
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
        for(const auto vertex : vertices_) {
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
    for(const auto vertex : vertices_) {
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
    for(const auto& vertex : vertices_) {
        NodeHandlePtr nh = vertex->getNodeHandle();
        node_handles.push_back(nh.get());
    }

    return node_handles;
}

ConnectablePtr Graph::findConnector(const UUID &uuid)
{
    ConnectablePtr res = findConnectorNoThrow(uuid);
    if(!res) {
        throw std::runtime_error(std::string("cannot find connector with UUID=") + uuid.getFullName());
    }
    return res;
}

ConnectablePtr Graph::findConnectorNoThrow(const UUID &uuid) noexcept
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
    return vertices_.begin();
}

const Graph::vertex_const_iterator Graph::beginVertices() const
{
    return vertices_.cbegin();
}


Graph::vertex_iterator Graph::endVertices()
{
    return vertices_.end();
}

const Graph::vertex_const_iterator Graph::endVertices() const
{
    return vertices_.cend();
}

