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

AddConnection::AddConnection(const std::string &from_uuid, const std::string &to_uuid)
    : from(NULL), to(NULL), from_uuid(from_uuid), to_uuid(to_uuid)
{
}

bool AddConnection::execute()
{
    if(from == NULL) {
        refresh();
    }

    return Graph::root()->addConnection(Connection::Ptr(new Connection(from, to)));
}

bool AddConnection::undo()
{
    refresh();

    Graph::root()->deleteConnection(Connection::Ptr(new Connection(from, to)));

    return true;
}

bool AddConnection::redo()
{
    refresh();
    return execute();
}

void AddConnection::refresh()
{
    from = dynamic_cast<ConnectorOut*> (Graph::root()->findConnector(from_uuid));
    to =  dynamic_cast<ConnectorIn*> (Graph::root()->findConnector(to_uuid));

    assert(from);
    assert(to);
}
