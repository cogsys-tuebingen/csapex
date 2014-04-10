/// HEADER
#include <csapex/model/box_selection_model.h>

/// COMPONENT
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
#include <csapex/view/widget_controller.h>

/// SYSTEM
#include <QApplication>
#include <boost/bind/protect.hpp>

using namespace csapex;

BoxSelectionModel::BoxSelectionModel(GraphPtr graph, WidgetController *widget_ctrl)
    : SelectionModel(graph, widget_ctrl)
{

}

void BoxSelectionModel::moveSelectedBoxes(Box*, const QPoint& delta)
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

void BoxSelectionModel::clearSelection()
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        widget_ctrl_->getBox(n->getUUID())->setSelected(false);
    }
    Q_EMIT selectionChanged();
}

void BoxSelectionModel::selectAll()
{
    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        widget_ctrl_->getBox(n->getUUID())->setSelected(true);
    }
    Q_EMIT selectionChanged();
}

void BoxSelectionModel::toggleSelection(Box *box)
{
    bool shift = Qt::ShiftModifier == QApplication::keyboardModifiers();

    handleSelection(box->getNode(), shift);
}

void BoxSelectionModel::handleSelection(Node* node, bool add)
{
    if(node != NULL) {
        if(add) {
            if(widget_ctrl_->getBox(node->getUUID())->isSelected()) {
                widget_ctrl_->getBox(node->getUUID())->setSelected(false);
            } else {
                select(node, true);
            }
        } else {
            if(widget_ctrl_->getBox(node->getUUID())->isSelected()) {
                clearSelection();
                if(countSelected() != 1) {
                    select(widget_ctrl_->getBox(node->getUUID())->getNode());
                }
            } else {
                select(widget_ctrl_->getBox(node->getUUID())->getNode());
            }
        }
    } else if(!add) {
        clearSelection();
    }
}



Command::Ptr BoxSelectionModel::deleteSelectedCommand()
{
    command::Meta::Ptr meta(new command::Meta("Delete Selected Nodes"));

    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        if(widget_ctrl_->getBox(n->getUUID())->isSelected()) {
            meta->add(Command::Ptr(new command::DeleteNode(n->getUUID())));
        }
    }

    clearSelection();

    return meta;
}

void BoxSelectionModel::select(Node *node, bool add)
{
    assert(!widget_ctrl_->getBox(node->getUUID())->isSelected());

    if(!add) {
        clearSelection();
    }

    widget_ctrl_->getBox(node->getUUID())->setSelected(true);

    Q_EMIT selectionChanged();
}

int BoxSelectionModel::countSelected()
{
    int c = 0;

    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        if(widget_ctrl_->getBox(n->getUUID())->isSelected()) {
            ++c;
        }
    }

    return c;
}

void BoxSelectionModel::fillContextMenuForSelection(QMenu *menu, std::map<QAction *, boost::function<void ()> > &handler)
{
    bool has_minimized = false;
    bool has_maximized = false;

    Q_FOREACH(Node::Ptr n, graph_->nodes_) {
        Box* b = widget_ctrl_->getBox(n->getUUID());
        if(b->isSelected()) {
            if(b->isMinimizedSize()) {
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
    handler[del] = boost::bind(&CommandDispatcher::execute, dispatcher_, boost::bind(boost::bind(&BoxSelectionModel::deleteSelectedCommand, this)));
    menu->addAction(del);
}


void BoxSelectionModel::mimicBoxMovement(Box *box, int dx, int dy)
{
    if(box->isSelected() && box->hasFocus()) {
        Q_FOREACH(Node::Ptr n, graph_->nodes_) {
            Box* b = widget_ctrl_->getBox(n->getUUID());
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
