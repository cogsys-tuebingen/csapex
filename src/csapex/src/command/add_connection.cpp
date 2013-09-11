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

    bool f_in = f->isInput();
    bool f_out = f->isOutput();
    bool f_for = f->isForwarding();

    bool t_in = t->isInput();
    bool t_out = t->isOutput();
    bool t_for = t->isForwarding();

    if((f_out && t_in) || (f_out && t_for && t_out) || (f_in && f_for && t_in)) {
        from = dynamic_cast<ConnectorOut*> (f);
        to =  dynamic_cast<ConnectorIn*> (t);

    } else if(f_in && t_out) {
        from = dynamic_cast<ConnectorOut*> (t);
        to =  dynamic_cast<ConnectorIn*> (f);

    } else {
        throw std::runtime_error(std::string("cannot connect ") +
                                 from_uuid + "(" + (f->isOutput() ? "o" : "") + (f->isInput() ? "i" : "") + "/" + (f->canOutput() ? "o" : "") + (f->canInput() ? "i" : "")  +
                                 ") to " +
                                 to_uuid + "(" + (t->isOutput() ? "o" : "") + (t->isInput() ? "i" : "") + "/" + (t->canOutput() ? "o" : "") + (t->canInput() ? "i" : "")  + ")");
    }

    assert(from);
    assert(to);
}
