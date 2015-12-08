/// HEADER
#include <csapex/command/delete_signal_connection.h>

/// PROJECT
#include <csapex/signal/signal_connection.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/graph.h>

using namespace csapex;
using namespace csapex::command;

DeleteSignalConnection::DeleteSignalConnection(Trigger *a, Slot *b)
    : DeleteConnection(a, b)
{

}

bool DeleteSignalConnection::doUndo()
{
    auto graph = graph_;
    Connectable* from = graph->findConnector(from_uuid);
    Connectable* to = graph->findConnector(to_uuid);

    Trigger* trigger = dynamic_cast<Trigger*>(from);
    Slot* slot = dynamic_cast<Slot*>(to);

    graph->addConnection(SignalConnection::connect(trigger, slot, connection_id));

    return Meta::doUndo();
}
