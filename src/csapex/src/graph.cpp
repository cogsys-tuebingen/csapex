/// HEADER
#include <csapex/graph.h>

/// PROJECT
#include "ui_designer.h"
#include <csapex/connector.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/command_meta.h>
#include <csapex/command_add_box.h>
#include <csapex/command_delete_box.h>
#include <csapex/command_add_connection.h>
#include <csapex/command_add_connection_forwarding.h>
#include <csapex/command_delete_connection.h>
#include <csapex/command_add_connector.h>
#include <csapex/command_delete_connector.h>
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

void Graph::addBox(Box *box)
{
    boxes_.push_back(box);

    box->setGraph(this);

    QObject::connect(box, SIGNAL(moveSelectionToBox(Box*)), this, SLOT(moveSelectionToBox(Box*)));

    Q_EMIT boxAdded(box);
}

void Graph::deleteBox(Box *box)
{
    box->stop();

    std::vector<Box*>::iterator it = std::find(boxes_.begin(), boxes_.end(), box);
    if(it != boxes_.end()) {
        boxes_.erase(it);
    }

    Q_EMIT boxDeleted(box);

    box->deleteLater();
}

void Graph::moveSelectionToBox(Box *box)
{
    if(!box->hasSubGraph()) {
        return;
    }

    Graph* sub_graph = box->getSubGraph();
    assert(sub_graph);

    command::Meta::Ptr meta(new command::Meta);

    std::vector<Box*> selected;

    command::Meta::Ptr add_connectors(new command::Meta);
    command::Meta::Ptr add_connections(new command::Meta);

    std::map<Box*, std::string> box2uuid;

    foreach(Box* b, boxes_) {
        // iterate selected boxes
        if(b->isSelected()) {
            selected.push_back(b);
            std::cerr << "selected: " << b->UUID() << std::endl;

            // move this box top the sub graph
            SelectorProxy::Ptr selector = BoxManager::instance().getSelector(b->getType());
            Memento::Ptr state = b->getState();

            std::string uuid = box->UUID() + namespace_separator + b->UUID();
            box2uuid[b] = uuid;

            meta->add(Command::Ptr(new command::DeleteBox(b)));
            meta->add(Command::Ptr(new command::AddBox(*sub_graph, selector, b->pos(), state, uuid)));
        }
    }

    foreach(Box* b, boxes_) {
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

                        std::string new_connector_uuid = Connector::makeUUID(box->UUID(), Connector::TYPE_MISC, box->nextInputId());

                        // create new separator connector
                        add_connectors->add(Command::Ptr(new command::AddConnector(box, true, new_connector_uuid, true)));
                        // connect old output to the new connector
                        add_connections->add(Command::Ptr(new command::AddConnection(box, target->UUID(), new_connector_uuid)));
                        // connect the new connector to the old input
                        add_connections->add(Command::Ptr(new command::AddConnection(box, new_connector_uuid, box->UUID() + Graph::namespace_separator + in->UUID())));
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
                            new_connector_uuid = Connector::makeUUID(box->UUID(), Connector::TYPE_MISC, box->nextOutputId());
                            add_connectors->add(Command::Ptr(new command::AddConnector(box, false, new_connector_uuid, true)));
                        }

                        add_connections->add(Command::Ptr(new command::AddConnection(box, new_connector_uuid, in->UUID())));
                        add_connections->add(Command::Ptr(new command::AddConnection(box, box->UUID() + Graph::namespace_separator + out->UUID(), new_connector_uuid)));

                    } else {
                        // internal connections are kept
                        std::cerr << "keep connection between " << in->UUID() << " and " << out->UUID() << std::endl;

                        add_connections->add(Command::Ptr(new command::AddConnection(box, box->UUID() + Graph::namespace_separator + out->UUID(), box->UUID() + Graph::namespace_separator + in->UUID())));
                    }
                }
            }

        }
    }

    meta->add(add_connectors);
    meta->add(add_connections);

    if(meta->commands() > 0) {
        CommandDispatcher::executeLater(meta);
    }
}

bool Graph::addConnection(Connection::Ptr connection)
{
    std::cerr << "try to make connection between " << connection->from()->UUID() << " and " << connection->to()->UUID() << std::endl;
    if(connection->from()->tryConnect(connection->to())) {
        std::cerr << "make connection between " << connection->from()->UUID() << " and " << connection->to()->UUID() << std::endl;
        connections.push_back(connection);

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

    foreach(Box* box, boxes_) {
        Command::Ptr cmd(new command::DeleteBox(box));
        clear->add(cmd);
    }

    if(clear->commands() > 0) {
        CommandDispatcher::execute(clear);
    }
}


Box* Graph::findBox(const std::string &uuid, const std::string& ns)
{
    if(uuid.find(namespace_separator) != uuid.npos && ns.empty()) {
        return findBox(uuid, uuid);
    }

    foreach(Box* b, boxes_) {
        if(b->UUID() == uuid) {
            return b;
        }
    }

    if(ns.find(namespace_separator) != ns.npos) {
        std::string parent = ns.substr(0, ns.find(namespace_separator));
        std::string rest = ns.substr(ns.find(namespace_separator)+namespace_separator.length());

        Box* meta = findBox(parent);
        BoxGroup* bm = dynamic_cast<BoxGroup*> (meta);

        assert(bm);

        return bm->getSubGraph()->findBox(uuid, rest);
    }

    return NULL;
}

Box* Graph::findConnectorOwner(const std::string &uuid, const std::string& ns)
{
    if(uuid.find(namespace_separator) != uuid.npos && ns.empty()) {
        return findConnectorOwner(uuid, uuid);
    }

    if(ns.find(namespace_separator) != ns.npos) {
        std::string parent = ns.substr(0, ns.find(namespace_separator));
        std::string rest = ns.substr(ns.find(namespace_separator)+namespace_separator.length());

        Box* meta = findBox(parent);
        assert(meta);

        BoxGroup* bm = dynamic_cast<BoxGroup*> (meta);

        assert(bm);

        return bm->getSubGraph()->findConnectorOwner(uuid, rest);
    }

    foreach(Box* b, boxes_) {
        if(b->getInput(uuid)) {
            return b;
        }
        if(b->getOutput(uuid)) {
            return b;
        }
    }

    std::cerr << "error: cannot find owner of connector '" << uuid << "'\n";

    foreach(Box* b, boxes_) {
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

    return NULL;
}

Connector* Graph::findConnector(const std::string &uuid, const std::string &ns)
{
    Box* owner = findConnectorOwner(uuid, ns);
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

void Graph::deleteConnectionById(int id)
{
    foreach(const Connection::Ptr& connection, connections) {
        if(connection->id() == id) {
            CommandDispatcher::execute(Command::Ptr(new command::DeleteConnection(connection->from(), connection->to())));
            return;
        }
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

    foreach(Box* b, boxes_) {
        if(b->isSelected()) {
            meta->add(Command::Ptr(new command::DeleteBox(b)));
        }
    }

    deselectBoxes();

    CommandDispatcher::execute(meta);
}



void Graph::toggleBoxSelection(Box *box)
{
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();

    handleBoxSelection(box, shift);
}


int Graph::noSelectedBoxes()
{
    int c = 0;

    foreach(Box* b, boxes_) {
        if(b->isSelected()) {
            ++c;
        }
    }

    return c;
}

void Graph::boxMoved(Box *box, int dx, int dy)
{
    if(box->isSelected() && box->hasFocus()) {
        foreach(Box* b, boxes_) {
            if(b != box && b->isSelected()) {
                b->move(b->x() + dx, b->y() + dy);
            }
        }
    }
}


void Graph::deselectBoxes()
{
    foreach(Box* b, boxes_) {
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
    foreach(Box* b, boxes_) {
        b->tick();
    }
    foreach(const Connection::Ptr& connection, connections) {
        connection->tick();
    }
}
