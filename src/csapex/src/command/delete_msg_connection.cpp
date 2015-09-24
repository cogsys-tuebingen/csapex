/// HEADER
#include <csapex/command/delete_msg_connection.h>

/// PROJECT
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/graph.h>

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

    Output* output = dynamic_cast<Output*>(from);
    Input* input = dynamic_cast<Input*>(to);

    graph->addConnection(BundledConnection::connect(output, input, connection_id));

    return Meta::doUndo();
}

