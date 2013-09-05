/// HEADER
#include <csapex/graph.h>

/// PROJECT
#include "ui_designer.h"
#include <csapex/connector.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/command_meta.h>
#include <csapex/command_add_box.h>
#include <csapex/command_move_box.h>
#include <csapex/command_delete_box.h>
#include <csapex/command_add_connection.h>
#include <csapex/command_delete_connection.h>
#include <csapex/command_add_connector.h>
#include <csapex/command_delete_connector.h>
#include <csapex/command_add_fulcrum.h>
#include <csapex/command_delete_fulcrum.h>
#include <csapex/command_move_fulcrum.h>
#include <csapex/selector_proxy.h>
#include <csapex/command_dispatcher.h>
#include <csapex/box_manager.h>
#include <csapex/box.h>
#include <csapex/box_group.h>
#include <csapex/qt_helper.hpp>
#include <csapex/command_meta.h>
#include <csapex/command_delete_box.h>
#include <csapex/stream_interceptor.h>

/// SYSTEM
#include <boost/foreach.hpp>
#include <QResizeEvent>
#include <QMenu>
#include <QScrollBar>
#include <QFileDialog>


using namespace csapex;

const std::string Graph::namespace_separator = ":/:";

Graph::Ptr Graph::root_;

Graph::Graph()
{
    QObject::connect(&CommandDispatcher::instance(), SIGNAL(stateChanged()), this, SIGNAL(stateChanged()));

    timer_ = new QTimer();
    timer_->setInterval(100);
    timer_->start();

    QObject::connect(timer_, SIGNAL(timeout()), this, SLOT(tick()));
}

Graph::~Graph()
{

}

Graph::Ptr Graph::root()
{
    return root_;
}

void Graph::setRoot(Graph::Ptr root)
{
    root_ = root;
}

void Graph::addBox(Box::Ptr box)
{
    assert(!box->UUID().empty());

    boxes_.push_back(box);

    QObject::connect(box.get(), SIGNAL(moveSelectionToBox(Box*)), this, SLOT(moveSelectionToBox(Box*)));

    Q_EMIT boxAdded(box.get());
}

void Graph::deleteBox(const std::string& uuid)
{
    Box::Ptr box = Graph::root()->findBox(uuid);

    box->stop();

    std::vector<Box::Ptr>::iterator it = std::find(boxes_.begin(), boxes_.end(), box);
    if(it != boxes_.end()) {
        boxes_.erase(it);
    }

    Q_EMIT boxDeleted(box.get());

    box->deleteLater();
}

command::Meta::Ptr Graph::moveSelectionToBoxCommands(const std::string& box_uuid)
{
    command::Meta::Ptr meta(new command::Meta);

    std::vector<Box*> selected;

    command::Meta::Ptr add_connectors(new command::Meta);
    command::Meta::Ptr add_connections(new command::Meta);

    std::map<Box*, std::string> box2uuid;


    foreach(Box::Ptr b, boxes_) {
        // iterate selected boxes
        if(b->isSelected()) {
            selected.push_back(b.get());
            std::cerr << "selected: " << b->UUID() << std::endl;

            // move this box top the sub graph
            SelectorProxy::Ptr selector = BoxManager::instance().getSelector(b->getType());
            Memento::Ptr state = b->getState();

            std::string uuid = box_uuid + namespace_separator + b->UUID();
            box2uuid[b.get()] = uuid;

            meta->add(Command::Ptr(new command::DeleteBox(b->UUID())));
            meta->add(Command::Ptr(new command::AddBox(selector, b->pos(), state, box_uuid, uuid)));
        }
    }

    int next_connector_sub_id = 0;

    foreach(Box::Ptr b, boxes_) {
        if(b->isSelected()) {
            // iterate all connections for this selected box
            // connections leading to a selected box will be kept
            // connections leading to a non-selected box must be split

            // for each connection to split:
            // - create pseudo connector for the sub graph
            // - split original connection using the new pseudo connector

            foreach(ConnectorIn* in, b->input) {
                if(in->isConnected()) {
                    Connector* target = in->getConnected();
                    Box* owner = target->getBox();
                    bool is_selected = false;
                    foreach(Box* b, selected) {
                        is_selected |= (b == owner);
                    }
                    bool is_external = !is_selected;
                    // internal connections are done by the next loop
                    // external connections should be split
                    if(is_external) {
                        std::cerr << "split connection between " << in->UUID() << " and " << target->UUID() << std::endl;

                        std::string new_connector_uuid = Connector::makeUUID(box_uuid, Connector::TYPE_MISC, next_connector_sub_id++);

                        // create new separator connector
                        add_connectors->add(Command::Ptr(new command::AddConnector(box_uuid, true, new_connector_uuid, true)));
                        // connect old output to the new connector
                        add_connections->add(Command::Ptr(new command::AddConnection(target->UUID(), new_connector_uuid)));
                        // connect the new connector to the old input
                        add_connections->add(Command::Ptr(new command::AddConnection(new_connector_uuid, box_uuid + Graph::namespace_separator + in->UUID())));
                    }
                }
            }
            foreach(ConnectorOut* out, b->output) {
                bool external_connector_added = false;
                std::string new_connector_uuid;

                for(std::vector<ConnectorIn*>::iterator it = out->beginTargets(); it != out->endTargets(); ++it) {
                    ConnectorIn* in = *it;
                    Box* owner = in->getBox();
                    bool is_selected = false;
                    foreach(Box* b, selected) {
                        is_selected |= (b == owner);
                    }
                    bool is_external = !is_selected;
                    if(is_external) {
                        // external connections are split
                        std::cerr << "split connection between " << in->UUID() << " and " << out->UUID() << std::endl;

                        if(!external_connector_added) {
                            external_connector_added = true;
                            new_connector_uuid = Connector::makeUUID(box_uuid, Connector::TYPE_MISC, next_connector_sub_id++);
                            add_connectors->add(Command::Ptr(new command::AddConnector(box_uuid, false, new_connector_uuid, true)));
                        }

                        add_connections->add(Command::Ptr(new command::AddConnection(new_connector_uuid, in->UUID())));
                        add_connections->add(Command::Ptr(new command::AddConnection(box_uuid + Graph::namespace_separator + out->UUID(), new_connector_uuid)));

                    } else {
                        // internal connections are kept
                        std::cerr << "keep connection between " << in->UUID() << " and " << out->UUID() << std::endl;

                        add_connections->add(Command::Ptr(new command::AddConnection(box_uuid + Graph::namespace_separator + out->UUID(), box_uuid + Graph::namespace_separator + in->UUID())));
                    }
                }
            }

        }
    }

    meta->add(add_connectors);
    meta->add(add_connections);

    return meta;
}

void Graph::moveSelectionToBox(Box *box)
{
    if(!box->hasSubGraph()) {
        return;
    }

    const std::string box_uuid = box->UUID();

    command::Meta::Ptr meta = moveSelectionToBoxCommands(box_uuid);

    if(meta->commands() > 0) {
        CommandDispatcher::executeLater(meta);
    }
}

void Graph::moveSelectedBoxes(const QPoint& delta)
{
    command::Meta::Ptr meta(new command::Meta);

    foreach(Box::Ptr b, boxes_) {
        if(b->isSelected()) {
            meta->add(Command::Ptr(new command::MoveBox(b.get(), b->pos())));
        }
    }

    foreach(const Connection::Ptr& connection, connections) {
        if(connection->from()->getBox()->isSelected() && connection->to()->getBox()->isSelected()) {
            int n = connection->getFulcrumCount();
            for(int i = 0; i < n; ++i) {
                QPoint pos = connection->getFulcrum(i);
                meta->add(Command::Ptr(new command::MoveFulcrum(connection->id(), i, pos - delta, pos)));
            }
        }
    }

    CommandDispatcher::execute(meta);
}

bool Graph::addConnection(Connection::Ptr connection)
{
    if(connection->from()->tryConnect(connection->to())) {
        connections.push_back(connection);

        Q_EMIT connectionAdded(connection.get());
        return true;
    }

    return false;
}

void Graph::deleteConnection(Connection::Ptr connection)
{
    connection->from()->removeConnection(connection->to());

    for(std::vector<Connection::Ptr>::iterator c = connections.begin(); c != connections.end();) {
        if(*connection == **c) {
            connections.erase(c);
            connection->to()->setError(false);

            Q_EMIT connectionDeleted(connection.get());
        } else {
            ++c;
        }
    }

    Q_EMIT stateChanged();
}

bool Graph::isDirty()
{
    return CommandDispatcher::instance().isDirty();
}

void Graph::stop()
{
    foreach(Box::Ptr box, boxes_) {
        box->stop();
    }

    boxes_.clear();
}


bool Graph::canUndo()
{
    return CommandDispatcher::instance().canUndo();
}

bool Graph::canRedo()
{
    return CommandDispatcher::instance().canRedo();
}


void Graph::undo()
{
    CommandDispatcher::instance().undo();
}

void Graph::redo()
{
    CommandDispatcher::instance().redo();
}

void Graph::clear()
{
    command::Meta::Ptr clear(new command::Meta);

    foreach(Box::Ptr box, boxes_) {
        Command::Ptr cmd(new command::DeleteBox(box->UUID()));
        clear->add(cmd);
    }

    if(clear->commands() > 0) {
        CommandDispatcher::execute(clear);
    }
}

void Graph::reset()
{
    boxes_.clear();
    connectors_.clear();
    connections.clear();
}

Graph::Ptr Graph::findSubGraph(const std::string& uuid)
{
    Box::Ptr bg = findBox(uuid);
    assert(bg->hasSubGraph());

    return bg->getSubGraph();
}

Box::Ptr Graph::findBox(const std::string &uuid, const std::string& ns)
{
    if(uuid.find(namespace_separator) != uuid.npos && ns.empty()) {
        return findBox(uuid, uuid);
    }

    foreach(Box::Ptr b, boxes_) {
        if(b->UUID() == uuid) {
            return b;
        }
    }

    if(ns.find(namespace_separator) != ns.npos) {
        std::string parent = ns.substr(0, ns.find(namespace_separator));
        std::string rest = ns.substr(ns.find(namespace_separator)+namespace_separator.length());

        Box::Ptr meta = findBox(parent);
        assert(meta->hasSubGraph());

        return meta->getSubGraph()->findBox(uuid, rest);
    }

    return Box::NullPtr;
}

Box::Ptr Graph::findConnectorOwner(const std::string &uuid, const std::string& ns)
{
    if(uuid.find(namespace_separator) != uuid.npos && ns.empty()) {
        return findConnectorOwner(uuid, uuid);
    }

    if(ns.find(namespace_separator) != ns.npos) {
        std::string parent = ns.substr(0, ns.find(namespace_separator));
        std::string rest = ns.substr(ns.find(namespace_separator)+namespace_separator.length());

        Box::Ptr meta = findBox(parent);
        assert(meta->hasSubGraph());

        return meta->getSubGraph()->findConnectorOwner(uuid, rest);
    }

    foreach(Box::Ptr b, boxes_) {
        if(b->getInput(uuid)) {
            return b;
        }
        if(b->getOutput(uuid)) {
            return b;
        }
    }

    std::cerr << "error: cannot find owner of connector '" << uuid << "'\n";

    foreach(Box::Ptr b, boxes_) {
        std::cerr << "box: " << b->UUID() << "\n";
        std::cerr << "inputs: " << "\n";
        foreach(ConnectorIn* in, b->input) {
            std::cerr << "\t" << in->UUID() << "\n";
        }
        std::cerr << "outputs: " << "\n";
        foreach(ConnectorOut* out, b->output) {
            std::cerr << "\t" << out->UUID() << "\n";
        }
    }

    std::cerr << std::flush;

    return Box::NullPtr;
}

Connector* Graph::findConnector(const std::string &uuid, const std::string &ns)
{
    Box::Ptr owner = findConnectorOwner(uuid, ns);
    assert(owner);

    Connector* result = NULL;
    result = owner->getInput(uuid);

    if(result == NULL) {
        result = owner->getOutput(uuid);
    }

    assert(result);

    return result;
}

bool Graph::handleConnectionSelection(int id, bool add)
{
    if(id != -1) {
        if(add) {
            if(isConnectionWithIdSelected(id)) {
                deselectConnectionById(id);
            } else {
                selectConnectionById(id, true);
            }
        } else {
            if(isConnectionWithIdSelected(id)) {
                if(noSelectedConnections() == 1) {
                    deselectConnectionById(id);
                } else {
                    selectConnectionById(id);
                }
            } else {
                selectConnectionById(id);
            }
        }
        return false;

    } else if(!add) {
        deselectConnections();
        return false;
    }

    return true;
}

void Graph::handleBoxSelection(Box* box, bool add)
{
    if(box != NULL) {
        if(add) {
            if(box->isSelected()) {
                box->setSelected(false);
            } else {
                selectBox(box, true);
            }
        } else {
            if(box->isSelected()) {
                deselectBoxes();
                if(noSelectedBoxes() != 1) {
                    selectBox(box);
                }
            } else {
                selectBox(box);
            }
        }
    } else if(!add) {
        deselectBoxes();
    }
}


Connection::Ptr Graph::getConnectionWithId(int id)
{
    BOOST_FOREACH(Connection::Ptr& connection, connections) {
        if(connection->id() == id) {
            return connection;
        }
    }

    return Connection::NullPtr;
}

Connection::Ptr Graph::getConnection(Connection::Ptr c)
{
    BOOST_FOREACH(Connection::Ptr& connection, connections) {
        if(*connection == *c) {
            return connection;
        }
    }

    std::cerr << "error: cannot get connection for " << *c << std::endl;

    return Connection::NullPtr;
}

int Graph::getConnectionId(Connection::Ptr c)
{
    Connection::Ptr internal = getConnection(c);

    if(internal != Connection::NullPtr) {
        return internal->id();
    }

    std::cerr << "error: cannot get connection id for " << *c << std::endl;

    return -1;
}

int Graph::noSelectedConnections()
{
    int c = 0;
    foreach(const Connection::Ptr& connection, connections) {
        if(connection->isSelected()) {
            ++c;
        }
    }

    return c;
}

void Graph::deselectConnections()
{
    BOOST_FOREACH(Connection::Ptr& connection, connections) {
        connection->setSelected(false);
    }
}

Command::Ptr Graph::deleteConnectionByIdCommand(int id)
{
    foreach(const Connection::Ptr& connection, connections) {
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
    command::Meta::Ptr meta(new command::Meta);
    int n = Graph::root()->getConnectionWithId(connection)->getFulcrumCount();
    for(int i = n - 1; i >= 0; --i) {
        meta->add(Graph::root()->deleteConnectionFulcrumCommand(connection, i));
    }

    return meta;
}

Command::Ptr Graph::deleteAllConnectionFulcrumsCommand(Connection::Ptr connection)
{
    return deleteAllConnectionFulcrumsCommand(getConnectionId(connection));
}


void Graph::deleteConnectionById(int id)
{
    Command::Ptr cmd(deleteConnectionByIdCommand(id));

    if(cmd) {
        CommandDispatcher::execute(cmd);
    }
}

void Graph::deleteSelectedConnections()
{
    command::Meta::Ptr meta(new command::Meta);

    foreach(const Connection::Ptr& connection, connections) {
        if(isConnectionWithIdSelected(connection->id())) {
            meta->add(Command::Ptr(new command::DeleteConnection(connection->from(), connection->to())));
        }
    }

    deselectConnections();

    CommandDispatcher::execute(meta);
}

void Graph::selectConnectionById(int id, bool add)
{
    if(!add) {
        BOOST_FOREACH(Connection::Ptr& connection, connections) {
            connection->setSelected(false);
        }
    }
    Connection::Ptr c = getConnectionWithId(id);
    if(c != Connection::NullPtr) {
        c->setSelected(true);
    }
}


void Graph::deselectConnectionById(int id)
{
    BOOST_FOREACH(Connection::Ptr& connection, connections) {
        if(connection->id() == id) {
            connection->setSelected(false);
        }
    }
}


bool Graph::isConnectionWithIdSelected(int id)
{
    if(id < 0) {
        return false;
    }

    foreach(const Connection::Ptr& connection, connections) {
        if(connection->id() == id) {
            return connection->isSelected();
        }
    }

    std::stringstream ss; ss << "no connection with id " << id;
    throw std::runtime_error(ss.str());
}

void Graph::deleteSelectedBoxes()
{
    command::Meta::Ptr meta(new command::Meta);

    foreach(Box::Ptr b, boxes_) {
        if(b->isSelected()) {
            meta->add(Command::Ptr(new command::DeleteBox(b->UUID())));
        }
    }

    deselectBoxes();

    CommandDispatcher::execute(meta);
}

void Graph::groupSelectedBoxes()
{
    // TODO: grouping does not work recursively, because groups cannot yet be deleted / created via commands
    QPoint tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    foreach(Box::Ptr b, boxes_) {
        if(b->isSelected()) {
            QPoint pos = b->pos();
            if(pos.x() < tl.x()) {
                tl.setX(pos.x());
            }
            if(pos.y() < tl.y()) {
                tl.setY(pos.y());
            }
        }
    }

    SelectorProxy::Ptr selector = BoxManager::instance().getSelector("::meta");
    const std::string& box = BoxManager::instance().makeUUID(selector->getType());

    command::Meta::Ptr meta(new command::Meta());
    meta->add(command::AddBox::Ptr(new command::AddBox(selector, tl, Memento::NullPtr, "", box)));
    meta->add(moveSelectionToBoxCommands(box));

    CommandDispatcher::execute(meta);
}


void Graph::selectAll()
{
    foreach(Box::Ptr b, boxes_) {
        b->setSelected(true);
    }
}

void Graph::clearSelection()
{
    foreach(Box::Ptr b, boxes_) {
        b->setSelected(false);
    }
}

void Graph::toggleBoxSelection(Box *box)
{
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();

    handleBoxSelection(box, shift);
}


int Graph::noSelectedBoxes()
{
    int c = 0;

    foreach(Box::Ptr b, boxes_) {
        if(b->isSelected()) {
            ++c;
        }
    }

    return c;
}

void Graph::boxMoved(Box *box, int dx, int dy)
{
    if(box->isSelected() && box->hasFocus()) {
        foreach(Box::Ptr b, boxes_) {
            if(b.get() != box && b->isSelected()) {
                b->move(b->x() + dx, b->y() + dy);
            }
        }
        foreach(const Connection::Ptr& connection, connections) {
            if(connection->from()->getBox()->isSelected() && connection->to()->getBox()->isSelected()) {
                int n = connection->getFulcrumCount();
                for(int i = 0; i < n; ++i) {
                    connection->moveFulcrum(i, connection->getFulcrum(i) + QPoint(dx,dy));
                }
            }
        }
    }
}


void Graph::deselectBoxes()
{
    foreach(Box::Ptr b, boxes_) {
        if(b->isSelected()) {
            b->setSelected(false);
        }
    }
}

void Graph::selectBox(Box *box, bool add)
{
    assert(!box->isSelected());

    if(!add) {
        deselectBoxes();
    }

    box->setSelected(true);
}

void Graph::tick()
{
    foreach(Box::Ptr b, boxes_) {
        b->tick();
    }
    foreach(const Connection::Ptr& connection, connections) {
        connection->tick();
    }
}
