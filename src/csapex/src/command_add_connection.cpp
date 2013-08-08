/// HEADER
#include <csapex/command_add_connection.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/box.h>
#include <csapex/graph.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;

AddConnection::AddConnection(Connector* a, Connector* b)
{
    from = dynamic_cast<ConnectorOut*>(a);
    if(from) {
        to = dynamic_cast<ConnectorIn*>(b);
    } else {
        from = dynamic_cast<ConnectorOut*>(b);
        to = dynamic_cast<ConnectorIn*>(a);
    }
    assert(from);
    assert(to);

    from_uuid = from->UUID();
    to_uuid = to->UUID();
}

bool AddConnection::execute(Graph& graph)
{
    return graph.addConnection(Connection::Ptr(new Connection(from, to)));
}

bool AddConnection::undo(Graph& graph)
{
    refresh(graph);

    graph.deleteConnection(Connection::Ptr(new Connection(from, to)));

    return true;
}

bool AddConnection::redo(Graph& graph)
{
    refresh(graph);
    return execute(graph);
}

void AddConnection::refresh(Graph& graph)
{
    from = graph.findConnectorOwner(from_uuid)->getOutput(from_uuid);
    to = graph.findConnectorOwner(to_uuid)->getInput(to_uuid);

    assert(from);
    assert(to);
}
