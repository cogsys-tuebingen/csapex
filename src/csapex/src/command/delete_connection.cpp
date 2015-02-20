/// HEADER
#include <csapex/command/delete_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_worker.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/model/node.h>
#include <csapex/model/connection.h>

/// SYSTEM


using namespace csapex;
using namespace csapex::command;

DeleteConnection::DeleteConnection(Connectable* a, Connectable* b)
    : Meta("delete connection and fulcrums"), from_uuid(UUID::NONE), to_uuid(UUID::NONE)
{
    if((a->isOutput() && b->isInput())) {
        from = a;
        to =  b;

    } else if(a->isInput() && b->isOutput()) {
        from = b;
        to =  a;
    }
    apex_assert_hard(from);
    apex_assert_hard(to);

    from_uuid = from->getUUID();
    to_uuid = to->getUUID();
}

std::string DeleteConnection::getType() const
{
    return "DeleteConnection";
}

std::string DeleteConnection::getDescription() const
{
    return std::string("deleted connection between ") + from_uuid.getFullName() + " and " + to_uuid.getFullName();
}


bool DeleteConnection::doExecute()
{
    const auto& graph = graph_worker_->getGraph();
    Connection::Ptr connection = graph->getConnection(from, to);

    connection_id = graph->getConnectionId(connection);

    locked = false;
    clear();
    add(graph->deleteAllConnectionFulcrumsCommand(connection));
    locked = true;

    if(Meta::doExecute()) {
        graph->deleteConnection(connection);
    }

    return true;
}

bool DeleteConnection::doUndo()
{
    if(!refresh()) {
        return false;
    }
    graph_worker_->getGraph()->addConnection(Connection::Ptr(new Connection(from, to, connection_id)));

    return Meta::doUndo();
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
    NodeWorker* from_worker = graph_worker_->getGraph()->findNodeWorkerForConnector(from_uuid);
    NodeWorker* to_worker = graph_worker_->getGraph()->findNodeWorkerForConnector(to_uuid);

    from = nullptr;
    to = nullptr;

    if(!from_worker || !to_worker) {
        return false;
    }

    from = from_worker->getConnector(from_uuid);
    to = to_worker->getConnector(to_uuid);

    apex_assert_hard(from);
    apex_assert_hard(to);

    return true;
}
