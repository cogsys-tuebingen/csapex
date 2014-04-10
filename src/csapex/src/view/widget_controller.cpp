/// HEADER
#include <csapex/view/widget_controller.h>

/// PROJECT
#include <csapex/command/delete_node.h>
#include <csapex/command/delete_connection.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/manager/box_manager.h>
#include <csapex/view/designer.h>
#include <csapex/view/box.h>
#include <csapex/command/move_box.h>
#include <csapex/model/connectable.h>
#include <csapex/command/move_fulcrum.h>
#include <csapex/view/port.h>
#include <csapex/command/dispatcher.h>

/// SYSTEM
#include <QApplication>
#include <boost/bind/protect.hpp>

using namespace csapex;

WidgetController::WidgetController(Graph::Ptr graph)
    : graph_(graph), box_selection_(graph, this), connection_selection_(graph, this), designer_(NULL)
{

}

Box* WidgetController::getBox(const UUID &node_id)
{
    return graph_->findNode(node_id)->getBox();
}

Graph::Ptr WidgetController::getGraph()
{
    return graph_;
}

void WidgetController::setDesigner(Designer *designer)
{
    designer_ = designer;
}

void WidgetController::setCommandDispatcher(CommandDispatcher* dispatcher)
{
    dispatcher_ = dispatcher;
    box_selection_.setCommandDispatcher(dispatcher);
    connection_selection_.setCommandDispatcher(dispatcher);
}

void WidgetController::nodeAdded(Node::Ptr node)
{
    if(designer_) {
        Box* box = BoxManager::instance().makeBox(node);
        QObject::connect(box, SIGNAL(moveRequest(Box*,QPoint)), &box_selection_, SLOT(moveSelectedBoxes(Box*, QPoint)));

        designer_->addBox(box);
    }
}

void WidgetController::nodeRemoved(NodePtr node)
{
    if(designer_) {
        Box* box = getBox(node->getUUID());
        designer_->removeBox(box);
    }
}

void WidgetController::foreachBox(boost::function<void (Box*)> f, boost::function<bool (Box*)> pred)
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        Box* b = getBox(n->getUUID());
        if(pred(b)) {
            f(b);
        }
    }
}



/**
  * BOX SELECTION
  **/

BoxSelectionManager::BoxSelectionManager(GraphPtr graph, WidgetController *widget_ctrl)
    : SelectionManager(graph, widget_ctrl)
{

}

void BoxSelectionManager::moveSelectedBoxes(Box*, const QPoint& delta)
{
    command::Meta::Ptr meta(new command::Meta("Move Selected Boxes"));

    Q_FOREACH(Node::Ptr node, graph_->nodes_) {
        Box* b = widget_ctrl_->getBox(node->getUUID());
        if(b->isSelected()) {
            meta->add(Command::Ptr(new command::MoveBox(b, b->pos())));
        }
    }

    Q_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
        Port* fromp = connection->from()->getPort();
        Port* top = connection->to()->getPort();
        if(!fromp || !top) {
            continue;
        }
        if(fromp->isSelected() && top->isSelected()) {
            int n = connection->getFulcrumCount();
            for(int i = 0; i < n; ++i) {
                const Connection::Fulcrum& f = connection->getFulcrum(i);
                meta->add(Command::Ptr(new command::MoveFulcrum(connection->id(), i, f.pos - delta, f.pos)));
            }
        }
    }

    dispatcher_->execute(meta);
}

void BoxSelectionManager::clearSelection()
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        n->getBox()->setSelected(false);
    }
    Q_EMIT selectionChanged();
}

void BoxSelectionManager::selectAll()
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        n->getBox()->setSelected(true);
    }
    Q_EMIT selectionChanged();
}

void BoxSelectionManager::toggleBoxSelection(Box *box)
{
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();

    handleNodeSelection(box->getNode(), shift);
}

void BoxSelectionManager::handleNodeSelection(Node* node, bool add)
{
    if(node != NULL) {
        if(add) {
            if(node->getBox()->isSelected()) {
                node->getBox()->setSelected(false);
            } else {
                selectNode(node, true);
            }
        } else {
            if(node->getBox()->isSelected()) {
                deselectNodes();
                if(countSelectedNodes() != 1) {
                    selectNode(node->getBox()->getNode());
                }
            } else {
                selectNode(node->getBox()->getNode());
            }
        }
    } else if(!add) {
        deselectNodes();
    }
}



Command::Ptr BoxSelectionManager::deleteSelectedNodesCmd()
{
    command::Meta::Ptr meta(new command::Meta("Delete Selected Nodes"));

    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        if(n->getBox()->isSelected()) {
            meta->add(Command::Ptr(new command::DeleteNode(n->getUUID())));
        }
    }

    deselectNodes();

    return meta;
}


void BoxSelectionManager::deselectNodes()
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        if(n->getBox()->isSelected()) {
            n->getBox()->setSelected(false);
        }
    }
    Q_EMIT selectionChanged();
}

void BoxSelectionManager::selectNode(Node *node, bool add)
{
    assert(!node->getBox()->isSelected());

    if(!add) {
        deselectNodes();
    }

    node->getBox()->setSelected(true);

    Q_EMIT selectionChanged();
}

int BoxSelectionManager::countSelectedNodes()
{
    int c = 0;

    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        if(n->getBox()->isSelected()) {
            ++c;
        }
    }

    return c;
}

void BoxSelectionManager::fillContextMenuForSelection(QMenu *menu, std::map<QAction *, boost::function<void ()> > &handler)
{
    bool has_minimized = false;
    bool has_maximized = false;

    Q_FOREACH(Node::Ptr b, graph_->nodes_) {
        if(b->getBox()->isSelected()) {
            if(b->getBox()->isMinimizedSize()) {
                has_minimized = true;
            } else {
                has_maximized = true;
            }
        }
    }

    boost::function<bool(Box*)> pred_selected = boost::bind(&Box::isSelected, _1);

    if(has_minimized) {
        QAction* max = new QAction("maximize all", menu);
        max->setIcon(QIcon(":/maximize.png"));
        max->setIconVisibleInMenu(true);
        handler[max] = boost::bind(&WidgetController::foreachBox, widget_ctrl_, boost::protect(boost::bind(&Box::minimizeBox, _1, false)), pred_selected);
        menu->addAction(max);
    }

    if(has_maximized){
        QAction* max = new QAction("minimize all", menu);
        max->setIcon(QIcon(":/minimize.png"));
        max->setIconVisibleInMenu(true);
        handler[max] = boost::bind(&WidgetController::foreachBox, widget_ctrl_, boost::protect(boost::bind(&Box::minimizeBox, _1, true)), pred_selected);
        menu->addAction(max);
    }

    menu->addSeparator();

    QAction* term = new QAction("terminate thread", menu);
    term->setIcon(QIcon(":/stop.png"));
    term->setIconVisibleInMenu(true);
    handler[term] = boost::bind(&WidgetController::foreachBox, widget_ctrl_, boost::protect(boost::bind(&Box::killContent, _1)), pred_selected);
    menu->addAction(term);

    QAction* prof = new QAction("profiling", menu);
    prof->setIcon(QIcon(":/profiling.png"));
    prof->setIconVisibleInMenu(true);
    handler[prof] = boost::bind(&WidgetController::foreachBox, widget_ctrl_, boost::protect(boost::bind(&Box::showProfiling, _1)), pred_selected);
    menu->addAction(prof);

    menu->addSeparator();

    QAction* del = new QAction("delete all", menu);
    del->setIcon(QIcon(":/close.png"));
    del->setIconVisibleInMenu(true);
    handler[del] = boost::bind(&CommandDispatcher::execute, dispatcher_, boost::bind(boost::bind(&BoxSelectionManager::deleteSelectedNodesCmd, this)));
    menu->addAction(del);
}


void BoxSelectionManager::boxMoved(Box *box, int dx, int dy)
{
    if(box->isSelected() && box->hasFocus()) {
        Q_FOREACH(Node::Ptr n, graph_->nodes_) {
            Box* b = n->getBox();
            if(b != box && b->isSelected()) {
                b->move(b->x() + dx, b->y() + dy);
            }
        }
        Q_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
            Port* fromp = connection->from()->getPort();
            Port* top = connection->to()->getPort();
            if(!fromp || !top) {
                continue;
            }
            if(fromp->isSelected() && top->isSelected()) {
                int n = connection->getFulcrumCount();
                for(int i = 0; i < n; ++i) {
                    connection->moveFulcrum(i, connection->getFulcrum(i).pos + QPoint(dx,dy));
                }
            }
        }
    }
}

/**
  * CONNECTION SELECTION
  **/


ConnectionSelectionManager::ConnectionSelectionManager(GraphPtr graph, WidgetController *widget_ctrl)
    : SelectionManager(graph, widget_ctrl)
{

}

int ConnectionSelectionManager::countSelectedConnections()
{
    int c = 0;

    Q_FOREACH(Connection::Ptr n, graph_->connections_) {
        if(n->isSelected()) {
            ++c;
        }
    }

    return c;
}


bool ConnectionSelectionManager::handleConnectionSelection(int id, bool add)
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
                if(countSelectedConnections() == 1) {
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


Command::Ptr ConnectionSelectionManager::deleteSelectedConnectionsCmd()
{
    command::Meta::Ptr meta(new command::Meta("Delete Selected Connections"));

    Q_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
        if(isConnectionWithIdSelected(connection->id())) {
            meta->add(Command::Ptr(new command::DeleteConnection(connection->from(), connection->to())));
        }
    }

    deselectConnections();

    return meta;
}

void ConnectionSelectionManager::selectConnectionById(int id, bool add)
{
    if(!add) {
        Q_FOREACH(Connection::Ptr connection, graph_->connections_) {
            connection->setSelected(false);
        }
    }
    Connection::Ptr c = graph_->getConnectionWithId(id);
    if(c != ConnectionNullPtr) {
        c->setSelected(true);
    }
    Q_EMIT selectionChanged();
}


void ConnectionSelectionManager::deselectConnections()
{
    Q_FOREACH(Connection::Ptr connection, graph_->connections_) {
        connection->setSelected(false);
    }
    Q_EMIT selectionChanged();
}


void ConnectionSelectionManager::deselectConnectionById(int id)
{
    Q_FOREACH(Connection::Ptr connection, graph_->connections_) {
        if(connection->id() == id) {
            connection->setSelected(false);
        }
    }
    Q_EMIT selectionChanged();
}


bool ConnectionSelectionManager::isConnectionWithIdSelected(int id)
{
    if(id < 0) {
        return false;
    }

    Q_FOREACH(const Connection::Ptr connection, graph_->connections_) {
        if(connection->id() == id) {
            return connection->isSelected();
        }
    }

    std::stringstream ss; ss << "no connection with id " << id;
    throw std::runtime_error(ss.str());
}
