/// HEADER
#include <csapex/command/move_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/add_connection.h>
#include <csapex/utility/assert.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex::command;

MoveConnection::MoveConnection(Connectable *from, Connectable *to)
    : Meta("MoveConnection"), from_uuid(from->getUUID()), to_uuid(to->getUUID())
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard((from->isOutput() && to->isOutput()) ||
           (from->isInput() && to->isInput()));

    output = from->isOutput();

    nested.clear();
    locked = false;

    if(output) {
        Output* out = dynamic_cast<Output*>(from);

        for(Output::TargetIterator it = out->beginTargets(); it != out->endTargets(); ++it) {
            add(Command::Ptr(new DeleteConnection(from, *it)));
            add(Command::Ptr(new AddConnection(to_uuid, (*it)->getUUID())));
        }

    } else {
        Input* in = dynamic_cast<Input*>(from);

        Connectable* target = in->getSource();
        add(Command::Ptr(new DeleteConnection(target, from)));
        add(Command::Ptr(new AddConnection(target->getUUID(), to_uuid)));
    }
}
