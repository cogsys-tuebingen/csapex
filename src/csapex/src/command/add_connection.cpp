/// HEADER
#include <csapex/command/add_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/bundled_connection.h>

using namespace csapex;
using namespace csapex::command;

AddConnection::AddConnection(const UUID& from_uuid, const UUID& to_uuid)
    : from_uuid(from_uuid), to_uuid(to_uuid)
{
}

std::string AddConnection::getType() const
{
    return "AddConnection";
}

std::string AddConnection::getDescription() const
{
    return std::string("added a connection between ") + from_uuid.getFullName() + " and " + to_uuid.getFullName();
}

bool AddConnection::doUndo()
{
    refresh();

    const auto& graph = graph_;
    graph->deleteConnection(graph->getConnection(from_uuid, to_uuid));

    return true;
}

bool AddConnection::doRedo()
{
    refresh();
    return doExecute();
}
