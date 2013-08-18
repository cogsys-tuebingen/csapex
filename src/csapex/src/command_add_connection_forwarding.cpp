/// HEADER
#include <csapex/command_add_connection_forwarding.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/connector_forward.h>
#include <csapex/box.h>
#include <csapex/graph.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;

AddConnectionForwarding::AddConnectionForwarding(Box* parent, const std::string &from_uuid, const std::string &to_uuid)
    : from(NULL), to(NULL), graph(graph), from_uuid(from_uuid), to_uuid(to_uuid)
{
    graph = parent->getGraph();
}

bool AddConnectionForwarding::execute()
{
    if(from == NULL) {
        refresh();
    }

    if(in) {
        return graph->addConnection(Connection::Ptr(new Connection(from_in, to_in)));
    } else {
        return graph->addConnection(Connection::Ptr(new Connection(from_out, to_out)));
    }
    return true;
}

bool AddConnectionForwarding::undo()
{
    refresh();


    if(in) {
        graph->deleteConnection(Connection::Ptr(new Connection(from_in, to_in)));
    } else {
        graph->deleteConnection(Connection::Ptr(new Connection(from_out, to_out)));
    }

    return true;
}

bool AddConnectionForwarding::redo()
{
    refresh();
    return execute();
    return true;
}

void AddConnectionForwarding::refresh()
{
    assert(graph->findConnector(from_uuid));
    assert(graph->findConnector(to_uuid));

    Connector* f = graph->findConnector(from_uuid);
    Connector* t = graph->findConnector(to_uuid);

    assert(f->isInput() == t->isInput());
    assert(f->isOutput() == t->isOutput());

    in = f->isInput();

    if(in) {
        from_in = dynamic_cast<ConnectorForward*> (f);
        to_in  = dynamic_cast<ConnectorIn*> (t);
    } else {
        from_out = dynamic_cast<ConnectorOut*> (f);
        to_out = dynamic_cast<ConnectorForward*> (t);
    }

    assert(from);
    assert(to);
}
