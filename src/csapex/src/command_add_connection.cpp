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

    graph = a->getBox()->getGraph();
}

AddConnection::AddConnection(Box* parent, const std::string &from_uuid, const std::string &to_uuid)
    : from(NULL), to(NULL), graph(graph), from_uuid(from_uuid), to_uuid(to_uuid)
{
    graph = parent->getGraph();
}

bool AddConnection::execute()
{
    if(from == NULL) {
        refresh();
    }

    return graph->addConnection(Connection::Ptr(new Connection(from, to)));
}

bool AddConnection::undo()
{
    refresh();

    graph->deleteConnection(Connection::Ptr(new Connection(from, to)));

    return true;
}

bool AddConnection::redo()
{
    refresh();
    return execute();
}

void AddConnection::refresh()
{
    assert(graph->findConnectorOwner(from_uuid));
    assert(graph->findConnectorOwner(to_uuid));

    from = dynamic_cast<ConnectorOut*> (graph->findConnector(from_uuid));
    to =  dynamic_cast<ConnectorIn*> (graph->findConnector(to_uuid));

    assert(from);
    assert(to);
}
