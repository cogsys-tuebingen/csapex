/// HEADER
#include <csapex/command/add_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>
#include <csapex/model/connection.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;


AddConnection::AddConnection(const UUID& from_uuid, const UUID& to_uuid)
    : from(nullptr), to(nullptr), from_uuid(from_uuid), to_uuid(to_uuid)
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
    if(from == nullptr) {
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

    if((f->isOutput() && t->isInput())) {
        from = f;
        to =  t;

    } else if(f->isInput() && t->isOutput()) {
        from = t;
        to =  f;

    } else {
        throw std::runtime_error(std::string("cannot connect ") +
                                 from_uuid.getFullName() + "(" + (f->isOutput() ? "o" : "") + (f->isInput() ? "i" : "") + "/" + (f->canOutput() ? "o" : "") + (f->canInput() ? "i" : "")  +
                                 ") to " +
                                 to_uuid.getFullName() + "(" + (t->isOutput() ? "o" : "") + (t->isInput() ? "i" : "") + "/" + (t->canOutput() ? "o" : "") + (t->canInput() ? "i" : "")  + ")");
    }

    apex_assert_hard(from);
    apex_assert_hard(to);
}
