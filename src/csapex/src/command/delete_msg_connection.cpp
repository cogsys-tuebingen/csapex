/// HEADER
#include <csapex/command/delete_msg_connection.h>

/// PROJECT
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/graph_facade.h>

using namespace csapex;
using namespace csapex::command;

DeleteMessageConnection::DeleteMessageConnection(const AUUID &parent_uuid, Output *a, Input *b)
    : DeleteConnection(parent_uuid, a, b)
{

}

bool DeleteMessageConnection::doUndo()
{
    Graph* graph = getGraph();

    ConnectablePtr from = graph->findConnector(from_uuid);
    ConnectablePtr to = graph->findConnector(to_uuid);

    OutputPtr output = std::dynamic_pointer_cast<Output>(from);
    InputPtr input = std::dynamic_pointer_cast<Input>(to);

    ConnectionPtr c = DirectConnection::connect(output, input, connection_id);
    c->setActive(active_);
    graph->addConnection(c);

    return Meta::doUndo();
}

