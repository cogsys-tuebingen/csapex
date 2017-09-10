/// HEADER
#include <csapex/model/graph/graph_local.h>

/// PROJECT
#include <csapex/model/connection.h>
#include <csapex/model/node_facade.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/model/node.h>
#include <csapex/model/node_facade_local.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/graph/edge.h>
#include <csapex/model/node.h>
#include <csapex/model/subgraph_node.h>

using namespace csapex;

GraphLocal::GraphLocal()
    : in_transaction_(false)
{

}

GraphLocal::~GraphLocal()
{
    clear();
}

AUUID GraphLocal::getAbsoluteUUID() const
{
    return nf_.lock()->getUUID().getAbsoluteUUID();
}

void GraphLocal::setNodeFacade(NodeFacadeWeakPtr nf)
{
    nf_ = nf;
}

void GraphLocal::resetActivity()
{
    auto connections = edges_;
    for(ConnectionPtr c : connections) {
        TokenPtr t = c->getToken();
        if(t) {
            t->setActivityModifier(ActivityModifier::NONE);
        }
    }

    auto vertices = vertices_;
    for(graph::VertexPtr vertex : vertices) {
        NodeFacadePtr node = vertex->getNodeFacade();
        node->setActive(false);
    }
}

void GraphLocal::clear()
{
    UUIDProvider::clearCache();

    beginTransaction();

    auto connections = edges_;
    for(ConnectionPtr c : connections) {
        deleteConnection(c);
    }
    apex_assert_hard(edges_.empty());

    auto vertices = vertices_;
    for(graph::VertexPtr vertex : vertices) {
        NodeFacadePtr node = vertex->getNodeFacade();
        deleteNode(node->getUUID());
    }
    apex_assert_hard(vertices_.empty());

    finalizeTransaction();
}

void GraphLocal::addNode(NodeFacadePtr nf)
{
    apex_assert_hard_msg(nf, "NodeFacade added is not null");
    graph::VertexPtr vertex = std::make_shared<graph::Vertex>(nf);
    vertices_.push_back(vertex);

    nf->getNodeHandle()->setVertex(vertex);

    sources_.insert(vertex);
    sinks_.insert(vertex);

    vertex_added(vertex);
    if(!in_transaction_) {
        analyzeGraph();
    }

}

std::vector<ConnectionPtr> GraphLocal::getConnections()
{
    return edges_;
}

void GraphLocal::deleteNode(const UUID& uuid)
{
    NodeHandle* node_handle = findNodeHandle(uuid);
    node_handle->stop();


    graph::VertexPtr removed;

    for(auto it = vertices_.begin(); it != vertices_.end();) {
        NodeFacadePtr node = (*it)->getNodeFacade();
        if(node->getUUID() == uuid) {
            removed = *it;
            vertices_.erase(it);

            break;

        } else {
            ++it;
        }
    }

    apex_assert_hard(removed);
    apex_assert_hard(removed == node_handle->getVertex());

    sources_.erase(removed);
    sinks_.erase(removed);

    for(const graph::VertexPtr& source : sources_) {
        apex_assert_neq(source, removed);
    }
    for(const graph::VertexPtr& sink : sinks_) {
        apex_assert_neq(sink, removed);
    }
    for(const graph::VertexPtr& remaining : vertices_) {
        apex_assert_neq(remaining, removed);
    }

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

int GraphLocal::countNodes()
{
    return vertices_.size();
}


bool GraphLocal::addConnection(ConnectionPtr connection)
{
    apex_assert_hard(connection);
    edges_.push_back(connection);

    connection_observations_[connection.get()].push_back(connection->connection_changed.connect([this]() {
        if(!in_transaction_) {
            analyzeGraph();
        }
    }));

    if(!std::dynamic_pointer_cast<Event>(connection->from()) && !std::dynamic_pointer_cast<Slot>(connection->to())) {
        NodeHandle* n_from = findNodeHandleForConnector(connection->from()->getUUID());
        NodeHandle* n_to = findNodeHandleForConnector(connection->to()->getUUID());

        if(n_to != n_from) {
            apex_assert_hard(n_to->getUUID().getAbsoluteUUID() != n_from->getUUID().getAbsoluteUUID());

            graph::VertexPtr v_from = n_from->getVertex();
            graph::VertexPtr v_to = n_to->getVertex();

            if(v_from && v_to) {
                v_from->addChild(v_to);
                v_to->addParent(v_from);

                sources_.erase(v_to);
                sinks_.erase(v_from);
            }

        }
    }

    connection_added(connection.get());
    if(!in_transaction_) {
        analyzeGraph();
    }
    return true;
}

void GraphLocal::deleteConnection(ConnectionPtr connection)
{
    apex_assert_hard(connection);

    connection_observations_[connection.get()].clear();

    if(connection->isDetached()) {
        auto c = std::find(edges_.begin(), edges_.end(), connection);
        if(c != edges_.end()) {
            edges_.erase(c);
        }
        return;
    }

    auto out = connection->from();
    auto in = connection->to();

    out->removeConnection(in.get());

    for(std::vector<ConnectionPtr>::iterator c = edges_.begin(); c != edges_.end();) {
        if(*connection == **c) {
            ConnectablePtr to = connection->to();

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
                            // verify that v_from is from this graph
                            for(const graph::VertexPtr& vertex : vertices_) {
                                if(vertex == v_from) {
                                    sinks_.insert(v_from);
                                    break;
                                }
                            }
                        }
                        if(!n_to->getInputTransition()->hasConnection()) {
                            // verify that v_to is from this graph
                            for(const graph::VertexPtr& vertex : vertices_) {
                                if(vertex == v_to) {
                                    sources_.insert(v_to);
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            edges_.erase(c);

            connection_removed(connection.get());
            if(!in_transaction_) {
                analyzeGraph();
            }
            for(const auto& c : edges_) {
                apex_assert_hard(c);
            }

            return;

        } else {
            ++c;
        }
    }


    throw std::runtime_error("cannot delete connection");
}

void GraphLocal::beginTransaction()
{
    in_transaction_ = true;
}

void GraphLocal::finalizeTransaction()
{
    in_transaction_ = false;

    analyzeGraph();
}

void GraphLocal::analyzeGraph()
{
    buildConnectedComponents();

    calculateDepths();

    state_changed();
}

void GraphLocal::buildConnectedComponents()
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

            checkNodeState(front->getNodeFacade()->getNodeHandle().get());

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

std::set<graph::Vertex *> GraphLocal::findVerticesThatJoinStreams()
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

std::set<graph::Vertex *> GraphLocal::findVerticesThatNeedMessages()
{
    std::set<graph::Vertex*> vertices_that_need_messages;

    for(const graph::VertexPtr v : vertices_) {
        if(v->getNodeFacade()->isProcessingNothingMessages()) {
            vertices_that_need_messages.insert(v.get());
            break;
        }
        for(const ConnectionPtr c : v->getNodeFacade()->getNodeHandle()->getOutputTransition()->getConnections()) {
            if(c->to()->isEssential()) {
                vertices_that_need_messages.insert(v.get());
                break;
            }
        }
    }

    return vertices_that_need_messages;
}

void GraphLocal::calculateDepths()
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


void GraphLocal::checkNodeState(NodeHandle *nh)
{
    // check if the node should be enabled
    nh->getInputTransition()->checkIfEnabled();
    nh->getOutputTransition()->checkIfEnabled();
    if(NodeWorker* worker = nh->getNodeWorker()) {
        worker->triggerTryProcess();
    }
}

int GraphLocal::getComponent(const UUID &node_uuid) const
{
    NodeHandle* node = findNodeHandleNoThrow(node_uuid);
    if(!node) {
        return -1;
    }

    graph::VertexPtr vertex = node->getVertex();
    return vertex->getNodeCharacteristics().component;
}

int GraphLocal::getDepth(const UUID &node_uuid) const
{
    NodeHandle* node = findNodeHandleNoThrow(node_uuid);
    if(!node) {
        return -1;
    }

    graph::VertexPtr vertex = node->getVertex();
    return vertex->getNodeCharacteristics().depth;
}

// *** NODE **** /

Node* GraphLocal::findNode(const UUID& uuid) const
{
    Node* node = findNodeNoThrow(uuid);
    if(node) {
        return node;
    }
    throw NodeNotFoundException(uuid.getFullName());
}


Node* GraphLocal::findNodeNoThrow(const UUID& uuid) const noexcept
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


Node* GraphLocal::findNodeForConnector(const UUID &uuid) const
{
    try {
        return findNode(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find node of connector \"") + uuid.getFullName() + ": " + e.what());
    }
}


// *** NODE HANDLE **** /

NodeHandle* GraphLocal::findNodeHandleForConnector(const UUID &uuid) const
{
    try {
        return findNodeHandle(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find handle of connector \"") + uuid.getFullName());
    }
}


NodeHandle* GraphLocal::findNodeHandle(const UUID& uuid) const
{
    if(uuid.empty()) {
        NodeFacadePtr nf = nf_.lock();
        apex_assert_hard(nf);
        apex_assert_hard(nf->getNodeHandle()->guard_ == -1);
        return nf->getNodeHandle().get();
    }
    NodeHandle* node_handle = findNodeHandleNoThrow(uuid);
    if(node_handle) {
        apex_assert_hard(node_handle->guard_ == -1);
        return node_handle;
    }
    throw NodeHandleNotFoundException(uuid.getFullName());
}

NodeHandle *GraphLocal::findNodeHandleNoThrow(const UUID& uuid) const noexcept
{
    if(uuid.empty()) {
        NodeFacadePtr nf = nf_.lock();
        apex_assert_hard(nf);
        apex_assert_hard(nf->getNodeHandle()->guard_ == -1);
        return nf->getNodeHandle().get();
    }
    if(uuid.composite()) {
        UUID root = uuid.rootUUID();

        NodeHandle* root_nh = findNodeHandleNoThrow(root);
        if(root_nh) {
            NodePtr root_node = root_nh->getNode().lock();
            if(root_node) {
                SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(root_node);
                if(graph) {
                    return graph->getGraph()->findNodeHandle(uuid.nestedUUID());
                }
            }
        }

    } else {
        for(const auto vertex : vertices_) {
            NodeFacadePtr nh = vertex->getNodeFacade();
            if(nh->getUUID() == uuid) {
                return nh->getNodeHandle().get();
            }
        }
    }

    return nullptr;
}

NodeHandle* GraphLocal::findNodeHandleForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return findNodeHandleNoThrow(uuid.parentUUID());
}

NodeHandle* GraphLocal::findNodeHandleWithLabel(const std::string& label) const
{
    for(const auto vertex : vertices_) {
        NodeFacadePtr nh = vertex->getNodeFacade();
        NodeStatePtr state = nh->getNodeState();
        if(state) {
            if(state->getLabel() == label) {
                return nh->getNodeHandle().get();
            }
        }
    }
    return nullptr;
}



// *** NODE FACADE **** /


NodeFacadePtr GraphLocal::findNodeFacadeForConnector(const UUID &uuid) const
{
    try {
        return findNodeFacade(uuid.parentUUID());

    } catch(const std::exception& e) {
        throw std::runtime_error(std::string("cannot find handle of connector \"") + uuid.getFullName());
    }
}


NodeFacadePtr GraphLocal::findNodeFacade(const UUID& uuid) const
{
    if(uuid.empty()) {
        NodeFacadePtr nf = nf_.lock();
        apex_assert_hard(nf);
        apex_assert_hard(nf->getNodeHandle()->guard_ == -1);
        return nf;
    }
    NodeFacadePtr node_facade = findNodeFacadeNoThrow(uuid);
    if(node_facade) {
        return node_facade;
    }
    throw NodeFacadeNotFoundException(uuid.getFullName());
}

NodeFacadePtr GraphLocal::findNodeFacadeNoThrow(const UUID& uuid) const noexcept
{
    if(uuid.empty()) {
        NodeFacadePtr nf = nf_.lock();
        apex_assert_hard(nf);
        apex_assert_hard(nf->getNodeHandle()->guard_ == -1);
        return nf;
    }
    if(uuid.composite()) {
        UUID root = uuid.rootUUID();

        NodeFacadePtr root_nf = findNodeFacadeNoThrow(root);
        if(root_nf) {
            NodePtr root_node = root_nf->getNodeHandle()->getNode().lock();
            if(root_node) {
                SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(root_node);
                if(graph) {
                    return graph->getGraph()->findNodeFacade(uuid.nestedUUID());
                }
            }
        }

    } else {
        for(const auto vertex : vertices_) {
            NodeFacadePtr nf = vertex->getNodeFacade();
            if(nf->getUUID() == uuid) {
                return nf;
            }
        }
    }

    return nullptr;
}

NodeFacadePtr GraphLocal::findNodeFacadeForConnectorNoThrow(const UUID &uuid) const noexcept
{
    return findNodeFacadeNoThrow(uuid.parentUUID());
}

NodeFacadePtr GraphLocal::findNodeFacadeWithLabel(const std::string& label) const
{
    for(const auto vertex : vertices_) {
        NodeFacadePtr nf = vertex->getNodeFacade();
        if(nf->getLabel() == label) {
            return nf;
        }
    }

    return nullptr;
}

// ******* /


Graph* GraphLocal::findSubgraph(const UUID& uuid) const
{
    NodeHandle* nh = findNodeHandle(uuid);
    if(SubgraphNodePtr graph = std::dynamic_pointer_cast<SubgraphNode>(nh->getNode().lock())) {
        return graph->getGraph().get();
    }

    throw std::runtime_error(std::string("cannot find graph \"") + uuid.getFullName() + "\"");
}

std::vector<NodeHandle*> GraphLocal::getAllNodeHandles()
{
    std::vector<NodeHandle*> node_handles;
    for(const auto& vertex : vertices_) {
        NodeFacadePtr nh = vertex->getNodeFacade();
        node_handles.push_back(nh->getNodeHandle().get());
    }

    return node_handles;
}

std::vector<NodeFacadePtr> GraphLocal::getAllNodeFacades()
{
    std::vector<NodeFacadePtr> node_facades;
    for(const graph::VertexPtr& vertex : vertices_) {
        NodeFacadePtr nf = vertex->getNodeFacade();
        node_facades.push_back(nf);
    }

    return node_facades;
}
std::vector<NodeFacadeLocalPtr> GraphLocal::getAllLocalNodeFacades()
{
    std::vector<NodeFacadeLocalPtr> node_facades;
    for(const graph::VertexPtr& vertex : vertices_) {
        NodeFacadeLocalPtr nf = std::dynamic_pointer_cast<NodeFacadeLocal>(vertex->getNodeFacade());
        apex_assert_hard(nf);
        node_facades.push_back(nf);
    }

    return node_facades;
}

ConnectablePtr GraphLocal::findConnectable(const UUID &uuid)
{
    ConnectorPtr connector = findConnector(uuid);
    return std::dynamic_pointer_cast<Connectable>(connector);
}

ConnectorPtr GraphLocal::findConnector(const UUID &uuid)
{
    ConnectorPtr res = findConnectorNoThrow(uuid);
    if(!res) {
        throw std::runtime_error(std::string("cannot find connector with UUID=") + uuid.getFullName());
    }
    return res;
}

ConnectorPtr GraphLocal::findConnectorNoThrow(const UUID &uuid) noexcept
{
    NodeHandle* owner = findNodeHandleNoThrow(uuid.parentUUID());
    if(!owner) {
        return nullptr;
    }

    return owner->getConnectorNoThrow(uuid);
}

ConnectionPtr GraphLocal::getConnectionWithId(int id)
{
    for(ConnectionPtr& connection : edges_) {
        if(connection->id() == id) {
            return connection;
        }
    }

    return nullptr;
}

ConnectionPtr GraphLocal::getConnection(const UUID &from, const UUID &to)
{
    for(ConnectionPtr& connection : edges_) {
        if(connection->from()->getUUID() == from && connection->to()->getUUID() == to) {
            return connection;
        }
    }

    return nullptr;
}

Graph::vertex_iterator GraphLocal::begin()
{
    return vertices_.begin();
}

const Graph::vertex_const_iterator GraphLocal::begin() const
{
    return vertices_.cbegin();
}


Graph::vertex_iterator GraphLocal::end()
{
    return vertices_.end();
}

const Graph::vertex_const_iterator GraphLocal::end() const
{
    return vertices_.cend();
}

