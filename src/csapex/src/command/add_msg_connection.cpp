/// HEADER
#include <csapex/command/add_msg_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/node_constructor.h>
#include <csapex/model/graph.h>
#include <csapex/utility/assert.h>
#include <csapex/msg/bundled_connection.h>
#include <csapex/msg/output.h>
#include <csapex/msg/input.h>
#include <csapex/model/node_handle.h>

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

    NodeHandle* from_nw = graph_->findNodeHandleForConnector(from_uuid);
    NodeHandle* to_nw = graph_->findNodeHandleForConnector(to_uuid);

    return graph_->addConnection(BundledConnection::connect(
                                     from, to,
                                     from_nw->getOutputTransition(),
                                     to_nw->getInputTransition()));
}

void AddMessageConnection::refresh()
{
    Connectable* f = graph_->findConnector(from_uuid);
    Connectable* t = graph_->findConnector(to_uuid);

    apex_assert_hard((f->isOutput() && t->isInput()));
    from = dynamic_cast<Output*>(f);
    to =  dynamic_cast<Input*>(t);

    apex_assert_hard(from);
    apex_assert_hard(to);
}