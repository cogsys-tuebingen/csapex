/// HEADER
#include <csapex/command/delete_connection.h>

/// COMPONENT
#include <csapex/command/command.h>

#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;

DeleteConnection::DeleteConnection(Connectable* a, Connectable* b)
{
    from = dynamic_cast<ConnectorOut*>(a);
    if(from) {
        to = dynamic_cast<ConnectorIn*>(b);
    } else {
        from = dynamic_cast<ConnectorOut*>(b);
        to = dynamic_cast<ConnectorIn*>(a);
    }
    assert(from);
    assert(to);

    from_uuid = from->getUUID();
    to_uuid = to->getUUID();
}

std::string DeleteConnection::getType() const
{
    return "DeleteConnection";
}

std::string DeleteConnection::getDescription() const
{
    return std::string("deleted connection between ") + from_uuid + " and " + to_uuid;
}


bool DeleteConnection::doExecute()
{
    Connection::Ptr connection(new Connection(from, to));

    connection_id = graph_->getConnectionId(connection);
    remove_fulcrums = graph_->deleteAllConnectionFulcrumsCommand(connection);

    if(Command::executeCommand(graph_, remove_fulcrums)) {
        graph_->deleteConnection(connection);
    }

    return true;
}

bool DeleteConnection::doUndo()
{
    if(!refresh()) {
        return false;
    }
    graph_->addConnection(Connection::Ptr(new Connection(from, to, connection_id)));

    return Command::undoCommand(graph_, remove_fulcrums);
}

bool DeleteConnection::doRedo()
{
    if(!refresh()) {
        throw std::runtime_error("cannot redo DeleteConnection");
    }
    return doExecute();
}

bool DeleteConnection::refresh()
{
    Node* from_node = graph_->findNodeForConnector(from_uuid);
    Node* to_node = graph_->findNodeForConnector(to_uuid);

    from = NULL;
    to = NULL;

    if(!from_node || !to_node) {
        return false;
    }

    from = from_node->getOutput(from_uuid);
    to = to_node->getInput(to_uuid);

    assert(from);
    assert(to);

    return true;
}
