/// HEADER
#include <csapex/command/delete_msg_connection.h>

/// PROJECT
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_worker.h>

using namespace csapex;
using namespace csapex::command;

DeleteMessageConnection::DeleteMessageConnection(Output *a, Input *b)
    : DeleteConnection(a, b)
{

}

bool DeleteMessageConnection::doUndo()
{
    auto graph = graph_;
    Connectable* from = graph->findConnector(from_uuid);
    Connectable* to = graph->findConnector(to_uuid);

    NodeWorker* from_nw = graph->findNodeWorkerForConnector(from_uuid);
    NodeWorker* to_nw = graph->findNodeWorkerForConnector(to_uuid);

    Output* output = dynamic_cast<Output*>(from);
    Input* input = dynamic_cast<Input*>(to);

    graph->addConnection(BundledConnection::connect(
                             output, input,
                             from_nw->getOutputTransition(),
                             to_nw->getInputTransition(),
                             connection_id));

    return Meta::doUndo();
}

