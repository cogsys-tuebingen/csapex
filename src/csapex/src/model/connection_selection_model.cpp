/// HEADER
#include <csapex/model/connection_selection_model.h>

/// COMPONENT
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

using namespace csapex;

ConnectionSelectionModel::ConnectionSelectionModel(GraphPtr graph, WidgetController *widget_ctrl)
    : SelectionModel(graph, widget_ctrl)
{

}

int ConnectionSelectionModel::countSelected()
{
    int c = 0;

    Q_FOREACH(Connection::Ptr n, graph_->connections_) {
        if(n->isSelected()) {
            ++c;
        }
    }

    return c;
}


void ConnectionSelectionModel::handleSelection(int id, bool add)
{
    if(id != -1) {
        if(add) {
            if(isConnectionWithIdSelected(id)) {
                deselectConnectionById(id);
            } else {
                select(id, true);
            }
        } else {
            if(isConnectionWithIdSelected(id)) {
                if(countSelected() == 1) {
                    deselectConnectionById(id);
                } else {
                    select(id);
                }
            } else {
                select(id);
            }
        }
        return;

    } else if(!add) {
        clearSelection();
        return;
    }

    return;
}


Command::Ptr ConnectionSelectionModel::deleteSelectedCommand()
{
    command::Meta::Ptr meta(new command::Meta("Delete Selected Connections"));

    Q_FOREACH(const Connection::Ptr& connection, graph_->connections_) {
        if(isConnectionWithIdSelected(connection->id())) {
            meta->add(Command::Ptr(new command::DeleteConnection(connection->from(), connection->to())));
        }
    }

    clearSelection();

    return meta;
}

void ConnectionSelectionModel::select(int id, bool add)
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


void ConnectionSelectionModel::clearSelection()
{
    Q_FOREACH(Connection::Ptr connection, graph_->connections_) {
        connection->setSelected(false);
    }
    Q_EMIT selectionChanged();
}


void ConnectionSelectionModel::deselectConnectionById(int id)
{
    Q_FOREACH(Connection::Ptr connection, graph_->connections_) {
        if(connection->id() == id) {
            connection->setSelected(false);
        }
    }
    Q_EMIT selectionChanged();
}


bool ConnectionSelectionModel::isConnectionWithIdSelected(int id)
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
