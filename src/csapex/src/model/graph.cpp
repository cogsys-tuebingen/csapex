/// HEADER
#include <csapex/model/graph.h>

/// PROJECT
#include <csapex/command/add_connection.h>
#include <csapex/command/add_connector.h>
#include <csapex/command/add_node.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/meta.h>
#include <csapex/command/meta.h>
#include <csapex/core/settings.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/connectable.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/node.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/qt_helper.hpp>
#include <csapex/utility/stream_interceptor.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;

Graph::Graph(Settings& settings)
    : settings_(settings), dispatcher_(NULL)
{
    timer_ = new QTimer();
    timer_->setInterval(1000. / 30.);
    timer_->start();

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(tick()));
}

Graph::~Graph()
{

}

Settings& Graph::getSettings() const
{
    return settings_;
}

void Graph::init(CommandDispatcher *dispatcher)
{
    dispatcher_ = dispatcher;
    assert(dispatcher_);
}

std::string Graph::makeUUIDPrefix(const std::string& name)
{
    int& last_id = uuids_[name];
    ++last_id;

    std::stringstream ss;
    ss << name << "_" << last_id;

    return ss.str();
}

void Graph::addNode(Node::Ptr node)
{
    assert(!node->getUUID().empty());
    assert(!node->getType().empty());
    assert(dispatcher_);

    nodes_.push_back(node);
    node_parents_[node.get()] = std::vector<Node*>();
    node_children_[node.get()] = std::vector<Node*>();

    node->setCommandDispatcher(dispatcher_);
    node->getNodeWorker()->makeThread();

    QObject::connect(this, SIGNAL(sig_tick()), node->getNodeWorker(), SLOT(checkParameters()));

    buildConnectedComponents();

    Q_EMIT nodeAdded(node);
}

void Graph::deleteNode(const UUID& uuid)
{
    Node* node = findNode(uuid);

    node->stop();

    /// assert that all connections have already been deleted
    assert(node_parents_[node].empty());
    assert(node_children_[node].empty());

    node_parents_.erase(node);
    node_children_.erase(node);

    Node::Ptr removed;

    for(std::vector<Node::Ptr>::iterator it = nodes_.begin(); it != nodes_.end();) {
        if((*it)->getUUID() == node->getUUID()) {
            removed = *it;
            it = nodes_.erase(it);

        } else {
            ++it;
        }
    }

    if(removed) {
        Q_EMIT nodeRemoved(removed);
        buildConnectedComponents();
    }
}

int Graph::countNodes()
{
    return nodes_.size();
}



void Graph::foreachNode(boost::function<void (Node*)> f, boost::function<bool (Node*)> pred)
{
    Q_FOREACH(Node::Ptr b, nodes_) {
        if(pred(b.get())) {
            f(b.get());
        }
    }
}

bool Graph::addConnection(Connection::Ptr connection)
{
    if(connection->from()->tryConnect(connection->to())) {
        Connectable* from = findConnector(connection->from()->getUUID());
        Connectable* to = findConnector(connection->to()->getUUID());

        connections_.push_back(connection);

        Node* n_from = findNodeForConnector(connection->from()->getUUID());
        Node* n_to = findNodeForConnector(connection->to()->getUUID());

        node_parents_[n_to].push_back(n_from);
        node_children_[n_from].push_back(n_to);

        if(node_component_[n_from] == node_component_[n_to]) {
            // if both parents are already in the same component
            // we need to have the same seq no

            int highest_seq_no = -1;
            // search all parents of the target for the highest seq no
            for(int i = 0; i < n_to->countInputs(); ++i) {
                ConnectorIn* input = n_to->getInput(i);
                Node* ni = findNodeForConnector(input->getSource()->getUUID());

                for(int j = 0; j < ni->countOutputs(); ++j) {
                    ConnectorOut* output = ni->getOutput(j);
                    if(output->sequenceNumber() > highest_seq_no) {
                        highest_seq_no = output->sequenceNumber();
                    }
                }
            }
            if(highest_seq_no != -1) {
                std::cerr << "setting the sequence numbers:\n";
                for(int i = 0; i < n_to->countInputs(); ++i) {
                    ConnectorIn* input = n_to->getInput(i);
                    std::cerr << " - " << input->getUUID().getFullName() << " from #" << input->sequenceNumber() << " to #" << highest_seq_no << std::endl;
                    input->setSequenceNumber(highest_seq_no);
                }
            }
        }

        buildConnectedComponents();
        verify();



        Q_EMIT connectionAdded(connection.get());
        Q_EMIT from->connectionDone();
        Q_EMIT to->connectionDone();
        return true;
    }

    std::cerr << "cannot connect " << connection->from()->getUUID() << " (" <<( connection->from()->isInput() ? "i": "o" )<< ") to " << connection->to()->getUUID() << " (" <<( connection->to()->isInput() ? "i": "o" )<< ")"  << std::endl;
    return false;
}

void Graph::deleteConnection(Connection::Ptr connection)
{
    connection->from()->removeConnection(connection->to());

    for(std::vector<Connection::Ptr>::iterator c = connections_.begin(); c != connections_.end();) {
        if(*connection == **c) {
            Connectable* to = connection->to();
            to->setError(false);

            Node* n_from = findNodeForConnector(connection->from()->getUUID());
            Node* n_to = findNodeForConnector(connection->to()->getUUID());

            // erase pointer from TO to FROM
            // if there are multiple edges, this only erases one entry
            node_parents_[n_to].erase(std::find(node_parents_[n_to].begin(), node_parents_[n_to].end(), n_from));

            // erase pointer from FROM to TO
            node_children_[n_from].erase(std::find(node_children_[n_from].begin(), node_children_[n_from].end(), n_to));

            connections_.erase(c);

            buildConnectedComponents();
            verify();
            Q_EMIT connectionDeleted(connection.get());
        } else {
            ++c;
        }
    }

    Q_EMIT stateChanged();

}


void Graph::buildConnectedComponents()
{
    /* Find all connected sub components of this graph */
    //    std::map<Node*, int> old_node_component = node_component_;
    node_component_.clear();

    std::deque<Node*> unmarked;
    Q_FOREACH(Node::Ptr node, nodes_) {
        unmarked.push_back(node.get());
        node_component_[node.get()] = -1;
    }

    std::deque<Node*> Q;
    int component = 0;
    while(!unmarked.empty()) {
        // take a random unmarked node to start bfs from
        Node* start = unmarked.front();
        Q.push_back(start);

        node_component_[start] = component;

        while(!Q.empty()) {
            Node* front = Q.front();
            Q.pop_front();

            unmarked.erase(std::find(unmarked.begin(), unmarked.end(), front));

            // iterate all neighbors
            std::vector<Node*> neighbors;
            Q_FOREACH(Node* parent, node_parents_[front]) {
                neighbors.push_back(parent);
            }
            Q_FOREACH(Node* child, node_children_[front]) {
                neighbors.push_back(child);
            }

            Q_FOREACH(Node* neighbor, neighbors) {
                if(node_component_[neighbor] == -1) {
                    node_component_[neighbor] = component;
                    Q.push_back(neighbor);
                }
            }
        }

        ++component;
    }

    Q_EMIT structureChanged(this);
}

void Graph::verify()
{
    verifyAsync();
}

void Graph::verifyAsync()
{
    /* Foreach node look for paths to every other node.
     * If there are two or more paths from one node to another
     *   and on of them contains an async edge, make all others
     *   temporary async
     */

    Q_FOREACH(Node::Ptr node, nodes_) {
        for(int i = 0; i < node->countInputs(); ++i) {
            node->getInput(i)->setTempAsync(false);
        }
    }

    Q_FOREACH(Node::Ptr node, nodes_) {
        std::deque<Node*> Q;
        std::map<Node*,bool> has_async_input;

        Node* current = node.get();

        Q.push_back(current);

        while(!Q.empty()) {
            Node* front = Q.front();
            Q.pop_front();

            bool visited = has_async_input.find(front) != has_async_input.end();
            if(!visited) {
                has_async_input[front] = false;
            }

            for(int i = 0; i < front->countOutputs(); ++i) {
                ConnectorOut* output = front->getOutput(i);
                for(ConnectorOut::TargetIterator in = output->beginTargets(); in != output->endTargets(); ++in) {
                    ConnectorIn* input = *in;

                    Node* next_node = findNodeForConnector(input->getUUID());

                    if(input->isAsync() || has_async_input[front]) {
                        has_async_input[next_node] = true;
                    }

                    Q.push_back(next_node);
                }
            }
        }


        for(int i = 0; i < current->countOutputs(); ++i) {
            ConnectorOut* output = current->getOutput(i);

            for(ConnectorOut::TargetIterator in = output->beginTargets(); in != output->endTargets(); ++in) {
                ConnectorIn* input = *in;

                Node* next_node = findNodeForConnector(input->getUUID());

                if(!input->isAsync()) {
                    bool a = has_async_input[next_node];
                    input->setTempAsync(a);
                }
            }
        }
    }
}

void Graph::stop()
{
    Q_FOREACH(Node::Ptr node, nodes_) {
        node->disable();
    }
    Q_FOREACH(Node::Ptr node, nodes_) {
        node->stop();
    }

    nodes_.clear();
}

bool Graph::isPaused() const
{
    return !timer_->isActive();
}

void Graph::setPause(bool pause)
{
    Q_FOREACH(Node::Ptr node, nodes_) {
        node->pause(pause);
    }
    if(pause) {
        timer_->stop();
    } else {
        timer_->start();
    }
}

void Graph::clearBlock()
{
    setPause(true);
    Q_FOREACH(Node::Ptr node, nodes_) {
        node->clearBlock();
    }
    setPause(false);
}

Command::Ptr Graph::clear()
{
    command::Meta::Ptr clear(new command::Meta("Clear Graph"));

    Q_FOREACH(Node::Ptr node, nodes_) {
        Command::Ptr cmd(new command::DeleteNode(node->getUUID()));
        clear->add(cmd);
    }

    return clear;
}

void Graph::reset()
{
    stop();

    settings_.setProcessingAllowed(true);

    uuids_.clear();
    connections_.clear();
}

int Graph::getComponent(const UUID &node_uuid) const
{
    Node* node = findNodeNoThrow(node_uuid);
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

    std::cerr << "cannot find box \"" << uuid << "\n";
    std::cerr << "available nodes:\n";
    Q_FOREACH(Node::Ptr n, nodes_) {
        std::cerr << n->getUUID() << '\n';
    }
    std::cerr << std::endl;
    throw std::runtime_error("cannot find box");
}

Node* Graph::findNodeNoThrow(const UUID& uuid) const
{
    Q_FOREACH(Node::Ptr b, nodes_) {
        if(b->getUUID() == uuid) {
            return b.get();
        }
    }

    Q_FOREACH(Node::Ptr b, nodes_) {
        //        Group::Ptr grp = boost::dynamic_pointer_cast<Group> (b);
        //        if(grp) {
        //            Node* tmp = grp->getSubGraph()->findNodeNoThrow(uuid);
        //            if(tmp) {
        //                return tmp;
        //            }
        //        }
    }

    return NULL;
}

Node* Graph::findNodeForConnector(const UUID &uuid) const
{
    UUID l = UUID::NONE;
    UUID r = UUID::NONE;
    uuid.split(UUID::namespace_separator, l, r);

    try {
        return findNode(l);

    } catch(const std::exception& e) {
        std::cerr << "error: cannot find owner of connector '" << uuid << "'\n";

        Q_FOREACH(Node::Ptr n, nodes_) {
            std::cerr << "node: " << n->getUUID() << "\n";
            std::cerr << "inputs: " << "\n";
            Q_FOREACH(ConnectorIn* in, n->getInputs()) {
                std::cerr << "\t" << in->getUUID() << "\n";
            }
            std::cerr << "outputs: " << "\n";
            Q_FOREACH(ConnectorOut* out, n->getOutputs()) {
                std::cerr << "\t" << out->getUUID() << "\n";
            }
        }

        std::cerr << std::flush;

        throw std::runtime_error(std::string("cannot find owner of connector \"") + uuid.getFullName());
    }
}

Connectable* Graph::findConnector(const UUID &uuid)
{
    UUID l = UUID::NONE;
    UUID r = UUID::NONE;
    uuid.split(UUID::namespace_separator, l, r);

    Node* owner = findNode(l);
    assert(owner);

    Connectable* result = owner->getConnector(uuid);;

    assert(result);

    return result;
}

Connection::Ptr Graph::getConnectionWithId(int id)
{
    BOOST_FOREACH(Connection::Ptr& connection, connections_) {
        if(connection->id() == id) {
            return connection;
        }
    }

    return ConnectionNullPtr;
}

Connection::Ptr Graph::getConnection(Connection::Ptr c)
{
    BOOST_FOREACH(Connection::Ptr& connection, connections_) {
        if(*connection == *c) {
            return connection;
        }
    }

    std::cerr << "error: cannot get connection for " << *c << std::endl;

    return ConnectionNullPtr;
}

int Graph::getConnectionId(Connection::Ptr c)
{
    Connection::Ptr internal = getConnection(c);

    if(internal != ConnectionNullPtr) {
        return internal->id();
    }

    return -1;
}
Command::Ptr Graph::deleteConnectionByIdCommand(int id)
{
    Q_FOREACH(const Connection::Ptr& connection, connections_) {
        if(connection->id() == id) {
            return Command::Ptr(new command::DeleteConnection(connection->from(), connection->to()));
        }
    }

    return Command::Ptr();
}

Command::Ptr Graph::deleteConnectionFulcrumCommand(int connection, int fulcrum)
{
    return Command::Ptr(new command::DeleteFulcrum(connection, fulcrum));
}

Command::Ptr Graph::deleteAllConnectionFulcrumsCommand(int connection)
{
    command::Meta::Ptr meta(new command::Meta("Delete All Connection Fulcrums"));

    if(connection >= 0) {
        int n = getConnectionWithId(connection)->getFulcrumCount();
        for(int i = n - 1; i >= 0; --i) {
            meta->add(deleteConnectionFulcrumCommand(connection, i));
        }
    }

    return meta;
}

Command::Ptr Graph::deleteAllConnectionFulcrumsCommand(Connection::Ptr connection)
{
    return deleteAllConnectionFulcrumsCommand(getConnectionId(connection));
}


Command::Ptr Graph::deleteConnectionById(int id)
{
    Command::Ptr cmd(deleteConnectionByIdCommand(id));

    return cmd;
}


void Graph::tick()
{
    //    Q_FOREACH(Node::Ptr n, nodes_) {
    //        if(n->isEnabled()) {
    //            n->tick();
    //        }
    //    }
    Q_EMIT sig_tick();

    Q_FOREACH(const Connection::Ptr& connection, connections_) {
        connection->tick();
    }
}
