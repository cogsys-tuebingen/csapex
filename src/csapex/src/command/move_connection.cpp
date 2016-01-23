/// HEADER
#include <csapex/command/move_connection.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_msg_connection.h>
#include <csapex/command/delete_signal_connection.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/command/add_signal_connection.h>
#include <csapex/utility/assert.h>

/// SYSTEM


using namespace csapex::command;

MoveConnection::MoveConnection(const UUID& parent_uuid, Connectable *from, Connectable *to)
    : Meta(parent_uuid, "MoveConnection"), from_uuid(from->getUUID()), to_uuid(to->getUUID())
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
            for(ConnectionPtr c : out->getConnections()) {
                if(!c) {
                    continue;
                }
                Input* input = dynamic_cast<Input*>(c->to());
                if(input) {
                    add(Command::Ptr(new DeleteMessageConnection(parent_uuid, out, input)));
                    add(Command::Ptr(new AddMessageConnection(parent_uuid, to_uuid, input->getUUID())));
                }
            }
        } else {
            Trigger* trigger = dynamic_cast<Trigger*>(from);
            if(trigger) {
                for(Slot* slot : trigger->getTargets()) {
                    add(Command::Ptr(new DeleteSignalConnection(parent_uuid, trigger, slot)));
                    add(Command::Ptr(new AddSignalConnection(parent_uuid, to_uuid, slot->getUUID())));
                }
            }
        }

    } else {
        Input* in = dynamic_cast<Input*>(from);

        if(in) {
            Output* target = dynamic_cast<Output*>(in->getSource());
            add(Command::Ptr(new DeleteMessageConnection(parent_uuid, target, in)));
            add(Command::Ptr(new AddMessageConnection(parent_uuid, target->getUUID(), to_uuid)));
        } else {
            Slot* in = dynamic_cast<Slot*>(from);

            if(in) {
                for(Trigger* target : in->getSources()) {
                    add(Command::Ptr(new DeleteSignalConnection(parent_uuid, target, in)));
                    add(Command::Ptr(new AddSignalConnection(parent_uuid, target->getUUID(), to_uuid)));
                }
            }
        }
    }
}
