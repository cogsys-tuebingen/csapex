/// HEADER
#include <csapex/command/add_msg_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_worker.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>

using namespace csapex;
using namespace csapex::command;

AddMessageConnection::AddMessageConnection(const UUID& from_uuid, const UUID& to_uuid)
    : AddConnection(from_uuid, to_uuid), from(nullptr), to(nullptr)
{
}

bool AddMessageConnection::doExecute()
{
    if(from == nullptr) {
        refresh();
    }

    return graph_->addConnection(BundledConnection::connect(from, to));
}

void AddMessageConnection::refresh()
{
    Connectable* f = graph_->findConnector(from_uuid);
    Connectable* t = graph_->findConnector(to_uuid);

    if((f->isOutput() && t->isInput())) {
        from = dynamic_cast<Output*>(f);
        to =  dynamic_cast<Input*>(t);

    } else if(f->isInput() && t->isOutput()) {
        from = dynamic_cast<Output*>(t);
        to =  dynamic_cast<Input*>(f);

    } else {
        throw std::runtime_error(std::string("cannot connect ") +
                                 from_uuid.getFullName() + "(" + (f->isOutput() ? "o" : "") + (f->isInput() ? "i" : "") + "/" + (f->canOutput() ? "o" : "") + (f->canInput() ? "i" : "")  +
                                 ") to " +
                                 to_uuid.getFullName() + "(" + (t->isOutput() ? "o" : "") + (t->isInput() ? "i" : "") + "/" + (t->canOutput() ? "o" : "") + (t->canInput() ? "i" : "")  + ")");
    }

    apex_assert_hard(from);
    apex_assert_hard(to);
}
