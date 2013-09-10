/// HEADER
#include <csapex/command/move_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/add_connection.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex::command;

MoveConnection::MoveConnection(Connector *a, Connector *b)
{
    Connector* from = dynamic_cast<ConnectorOut*>(a);
    Connector* to = NULL;

    if(from) {
        to = dynamic_cast<ConnectorOut*>(b);
        output = true;

    } else {
        from = dynamic_cast<ConnectorIn*>(a);
        to = dynamic_cast<ConnectorIn*>(b);
        output = false;
    }
    assert(from);
    assert(to);

    from_uuid = from->UUID();
    to_uuid = to->UUID();

    nested.clear();
    locked = false;

    if(output) {
        ConnectorOut* out = dynamic_cast<ConnectorOut*>(from);

        for(ConnectorOut::TargetIterator it = out->beginTargets(); it != out->endTargets(); ++it) {
            add(Command::Ptr(new DeleteConnection(from, *it)));
            add(Command::Ptr(new AddConnection(to_uuid, (*it)->UUID())));
        }

    } else {
        ConnectorIn* in = dynamic_cast<ConnectorIn*>(from);

        Connector* target = in->getConnected();
        add(Command::Ptr(new DeleteConnection(from, target)));
        add(Command::Ptr(new AddConnection(target->UUID(), to_uuid)));
    }
}
