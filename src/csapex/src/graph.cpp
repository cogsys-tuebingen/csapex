/// HEADER
#include <csapex/graph.h>

/// PROJECT
#include "ui_designer.h"
#include <csapex/connector.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/command_delete_connection.h>
#include <csapex/selector_proxy.h>
#include <csapex/command_dispatcher.h>
#include <csapex/box.h>
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

    Q_EMIT boxAdded(box);
}

void Graph::deleteBox(Box *box)
{
    box->stop();

    std::vector<Box*>::iterator it = std::find(boxes_.begin(), boxes_.end(), box);
    if(it != boxes_.end()) {
        boxes_.erase(it);
    }
    box->deleteLater();

    Q_EMIT boxDeleted(box);
}

bool Graph::addConnection(Connection::Ptr connection)
{
    if(connection->from()->tryConnect(connection->to())) {
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


Box* Graph::findBox(const std::string &uuid)
{
    foreach(Box* b, boxes_) {
        if(b->UUID() == uuid) {
            return b;
        }
    }

    return NULL;
}

Box* Graph::findConnectorOwner(const std::string &uuid)
{
    foreach(Box* b, boxes_) {
        if(b->getInput(uuid)) {
            return b;
        }
        if(b->getOutput(uuid)) {
            return b;
        }
    }

    return NULL;
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
