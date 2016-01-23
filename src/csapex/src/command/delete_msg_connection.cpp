/// HEADER
#include <csapex/command/delete_msg_connection.h>

/// PROJECT
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/graph_facade.h>

using namespace csapex;
using namespace csapex::command;

DeleteMessageConnection::DeleteMessageConnection(const UUID &parent_uuid, Output *a, Input *b)
    : DeleteConnection(parent_uuid, a, b)
{

}

bool DeleteMessageConnection::doUndo()
{
    Graph* graph = getGraph();

    Connectable* from = graph->findConnector(from_uuid);
    Connectable* to = graph->findConnector(to_uuid);

    NodeHandle* from_nh = graph->findNodeHandleForConnector(from_uuid);
    NodeHandle* to_nh = graph->findNodeHandleForConnector(to_uuid);

    Output* output = dynamic_cast<Output*>(from);
    Input* input = dynamic_cast<Input*>(to);

    graph->addConnection(BundledConnection::connect(
                             output, input,
                             from_nh->getOutputTransition(),
                             to_nh->getInputTransition(),
                             connection_id));

    return Meta::doUndo();
}

