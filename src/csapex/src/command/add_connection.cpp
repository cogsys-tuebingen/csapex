/// HEADER
#include <csapex/command/add_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/box.h>
#include <csapex/model/graph.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;


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
    Connector* f = Graph::root()->findConnector(from_uuid);
    Connector* t = Graph::root()->findConnector(to_uuid);

    if(f->canOutput() && t->canInput()) {
        from = dynamic_cast<ConnectorOut*> (f);
        to =  dynamic_cast<ConnectorIn*> (t);
    } else {
        from_uuid.swap(to_uuid);

        from = dynamic_cast<ConnectorOut*> (t);
        to =  dynamic_cast<ConnectorIn*> (f);
    }

    assert(from);
    assert(to);
}
