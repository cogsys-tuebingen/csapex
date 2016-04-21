/// HEADER
#include <csapex/command/add_signal_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>
#include <csapex/signal/signal_connection.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/graph_facade.h>

using namespace csapex;
using namespace csapex::command;

AddSignalConnection::AddSignalConnection(const AUUID& parent_uuid, const UUID& from_uuid, const UUID& to_uuid)
    : AddConnection(parent_uuid, from_uuid, to_uuid), from(nullptr), to(nullptr)
{
}

bool AddSignalConnection::doExecute()
{
    if(from == nullptr) {
        refresh();
    }

    Graph* graph = getGraph();

    return graph->addConnection(SignalConnection::connect(from, to));
}

void AddSignalConnection::refresh()
{
    Graph* graph = getGraph();

    Connectable* f = graph->findConnector(from_uuid);
    Connectable* t = graph->findConnector(to_uuid);

    if((f->isOutput() && t->isInput())) {
        from = dynamic_cast<Event*>(f);
        to =  dynamic_cast<Slot*>(t);

    } else if(f->isInput() && t->isOutput()) {
        from = dynamic_cast<Event*>(t);
        to =  dynamic_cast<Slot*>(f);

    } else {
        throw std::runtime_error(std::string("cannot connect ") +
                                 from_uuid.getFullName() + "(" + (f->isOutput() ? "o" : "") + (f->isInput() ? "i" : "") + "/" + (f->canOutput() ? "o" : "") + (f->canInput() ? "i" : "")  +
                                 ") to " +
                                 to_uuid.getFullName() + "(" + (t->isOutput() ? "o" : "") + (t->isInput() ? "i" : "") + "/" + (t->canOutput() ? "o" : "") + (t->canInput() ? "i" : "")  + ")");
    }

    apex_assert_hard(from);
    apex_assert_hard(to);
}
