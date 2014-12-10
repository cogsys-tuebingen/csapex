/// HEADER
#include <csapex/command/move_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
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

    bool is_output = from->isOutput();

    nested.clear();
    locked = false;

    if(is_output) {
        Output* out = dynamic_cast<Output*>(from);
        if(out) {
            foreach(Input* input, out->getTargets()) {
                add(Command::Ptr(new DeleteConnection(from, input)));
                add(Command::Ptr(new AddConnection(to_uuid, input->getUUID())));
            }
        } else {
            Trigger* trigger = dynamic_cast<Trigger*>(from);
            if(trigger) {
                foreach(Slot* slot, trigger->getTargets()) {
                    add(Command::Ptr(new DeleteConnection(from, slot)));
                    add(Command::Ptr(new AddConnection(to_uuid, slot->getUUID())));
                }
            }
        }

    } else {
        Input* in = dynamic_cast<Input*>(from);

        if(in) {
            Connectable* target = in->getSource();
            add(Command::Ptr(new DeleteConnection(target, from)));
            add(Command::Ptr(new AddConnection(target->getUUID(), to_uuid)));
        } else {
            Slot* in = dynamic_cast<Slot*>(from);

            if(in) {
                Connectable* target = in->getSource();
                add(Command::Ptr(new DeleteConnection(target, from)));
                add(Command::Ptr(new AddConnection(target->getUUID(), to_uuid)));
            }
        }
    }
}
