/// HEADER
#include <csapex/command_delete_connection.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/graph.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;

DeleteConnection::DeleteConnection(Connector* a, Connector* b)
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

bool DeleteConnection::execute(Graph& graph)
{
    graph.deleteConnection(Connection::Ptr(new Connection(from, to)));

    return true;
}

bool DeleteConnection::undo(Graph& graph)
{
    if(!refresh(graph)) {
        return false;
    }
    return graph.addConnection(Connection::Ptr(new Connection(from, to)));
}

bool DeleteConnection::redo(Graph& graph)
{
    if(!refresh(graph)) {
        throw std::runtime_error("cannot redo DeleteConnection");
    }
    return execute(graph);
}

bool DeleteConnection::refresh(Graph& graph)
{
    Box* from_box = graph.findConnectorOwner(from_uuid);
    Box* to_box = graph.findConnectorOwner(to_uuid);

    from = NULL;
    to = NULL;

    if(!from_box || !to_box) {
        return false;
    }

    from = from_box->getOutput(from_uuid);
    to = to_box->getInput(to_uuid);

    assert(from);
    assert(to);

    return true;
}
