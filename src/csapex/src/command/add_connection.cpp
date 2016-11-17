/// HEADER
#include <csapex/command/add_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/direct_connection.h>

using namespace csapex;
using namespace csapex::command;

AddConnection::AddConnection(const AUUID& parent_uuid, const UUID& from_uuid, const UUID& to_uuid, bool active)
    : Command(parent_uuid), from_uuid(from_uuid), to_uuid(to_uuid), from(nullptr), to(nullptr), active(active)
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

    Graph* graph = getGraph();
    graph->deleteConnection(graph->getConnection(from_uuid, to_uuid));

    return true;
}

bool AddConnection::doRedo()
{
    refresh();
    return doExecute();
}


bool AddConnection::doExecute()
{
    if(from == nullptr) {
        refresh();
    }

    Graph* graph = getGraph();

    ConnectionPtr c = DirectConnection::connect(from, to);
    c->setActive(active);

    return graph->addConnection(c);
}

void AddConnection::refresh()
{
    Graph* graph = getGraph();

    ConnectablePtr f = graph->findConnector(from_uuid);
    ConnectablePtr t = graph->findConnector(to_uuid);

    apex_assert_hard((f->isOutput() && t->isInput()));
    from = std::dynamic_pointer_cast<Output>(f);
    to =  std::dynamic_pointer_cast<Input>(t);

    apex_assert_hard(from);
    apex_assert_hard(to);
}
