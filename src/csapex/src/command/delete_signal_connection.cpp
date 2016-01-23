/// HEADER
#include <csapex/command/delete_signal_connection.h>

/// PROJECT
#include <csapex/signal/signal_connection.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>

using namespace csapex;
using namespace csapex::command;

DeleteSignalConnection::DeleteSignalConnection(const UUID& parent_uuid, Trigger *a, Slot *b)
    : DeleteConnection(parent_uuid, a, b)
{

}

bool DeleteSignalConnection::doUndo()
{
    Graph* graph = getGraph();

    Connectable* from = graph->findConnector(from_uuid);
    Connectable* to = graph->findConnector(to_uuid);

    Trigger* trigger = dynamic_cast<Trigger*>(from);
    Slot* slot = dynamic_cast<Slot*>(to);

    graph->addConnection(SignalConnection::connect(trigger, slot, connection_id));

    return Meta::doUndo();
}
