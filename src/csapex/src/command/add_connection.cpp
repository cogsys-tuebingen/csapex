/// HEADER
#include <csapex/command/add_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>

#include <csapex/model/graph.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;


AddConnection::AddConnection(const UUID& from_uuid, const UUID& to_uuid)
    : from(NULL), to(NULL), from_uuid(from_uuid), to_uuid(to_uuid)
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

bool AddConnection::doExecute()
{
    if(from == NULL) {
        refresh();
    }

    return graph_->addConnection(Connection::Ptr(new Connection(from, to)));
}

bool AddConnection::doUndo()
{
    refresh();

    graph_->deleteConnection(Connection::Ptr(new Connection(from, to)));

    return true;
}

bool AddConnection::doRedo()
{
    refresh();
    return doExecute();
}

void AddConnection::refresh()
{
    Connectable* f = graph_->findConnector(from_uuid);
    Connectable* t = graph_->findConnector(to_uuid);

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
                                 from_uuid.getFullName() + "(" + (f->isOutput() ? "o" : "") + (f->isInput() ? "i" : "") + "/" + (f->canOutput() ? "o" : "") + (f->canInput() ? "i" : "")  +
                                 ") to " +
                                 to_uuid.getFullName() + "(" + (t->isOutput() ? "o" : "") + (t->isInput() ? "i" : "") + "/" + (t->canOutput() ? "o" : "") + (t->canInput() ? "i" : "")  + ")");
    }

    assert(from);
    assert(to);
}
