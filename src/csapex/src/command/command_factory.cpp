/// HEADER
#include <csapex/command/command_factory.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/delete_signal_connection.h>
#include <csapex/command/delete_msg_connection.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/command/add_signal_connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/connection.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/assert.h>

using namespace csapex;

CommandFactory::CommandFactory(Graph *graph)
    : graph_(graph)
{

}

/// CONNECTORS
///
///

Command::Ptr CommandFactory::addConnection(const UUID &from, const UUID &to)
{
    auto from_c = graph_->findConnector(from);

    if(dynamic_cast<Output*>(from_c)) {
        return std::make_shared<command::AddMessageConnection>(from, to);
    } else if(dynamic_cast<Input*>(from_c)) {
        return std::make_shared<command::AddMessageConnection>(to, from);
    }
    if(dynamic_cast<Trigger*>(from_c)) {
        return std::make_shared<command::AddSignalConnection>(from, to);
    } else if(dynamic_cast<Slot*>(from_c)) {
        return std::make_shared<command::AddSignalConnection>(to, from);
    }
    return nullptr;
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(ConnectablePtr c)
{
    return removeAllConnectionsCmd(c.get());
}


Command::Ptr CommandFactory::removeAllConnectionsCmd(Connectable* c)
{
    if(Input* input = dynamic_cast<Input*>(c)) {
        return removeAllConnectionsCmd(input);
    }
    if(Output* output = dynamic_cast<Output*>(c)) {
        return removeAllConnectionsCmd(output);
    }
    if(Slot* slot = dynamic_cast<Slot*>(c)) {
        return removeAllConnectionsCmd(slot);
    }
    if(Trigger* trigger = dynamic_cast<Trigger*>(c)) {
        return removeAllConnectionsCmd(trigger);
    }
    return nullptr;
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Input* input)
{
    auto connections = input->getConnections();
    if(connections.empty()) {
        return nullptr;
    }
    apex_assert_hard(connections.size() == 1);
    Output* output = dynamic_cast<Output*>(input->getSource());
    Command::Ptr cmd(new command::DeleteMessageConnection(output, input));
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Output* output, Connection* connection) {
    Input* input = dynamic_cast<Input*>(connection->to());
    return Command::Ptr (new command::DeleteMessageConnection(output, input));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Output* output)
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    for(ConnectionPtr connection : output->getConnections()) {
        Input* input = dynamic_cast<Input*>(connection->to());
        Command::Ptr removeThis(new command::DeleteMessageConnection(output, input));
        removeAll->add(removeThis);
    }

    return removeAll;
}


Command::Ptr CommandFactory::removeAllConnectionsCmd(Slot* slot)
{
    command::Meta::Ptr cmd(new command::Meta("Delete sources"));
    for(Trigger* source : slot->getSources()) {
        cmd->add(Command::Ptr(new command::DeleteSignalConnection(source, slot)));
    }
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Trigger* trigger, Slot* other_side)
{
    return Command::Ptr (new command::DeleteSignalConnection(trigger, other_side));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Trigger* trigger)
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    for(Slot* target : trigger->getTargets()) {
        Command::Ptr removeThis(new command::DeleteSignalConnection(trigger, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}




/// graph_
///
///

Command::Ptr CommandFactory::clearCommand()
{
    command::Meta::Ptr clear(new command::Meta("Clear graph_"));

    for(auto node : graph_->getAllNodeHandles()) {
        clear->add(Command::Ptr (new command::DeleteNode(node->getUUID())));
    }

    return clear;
}

Command::Ptr CommandFactory::deleteConnectionByIdCommand(int id)
{
    for(const auto& connection : graph_->getConnections()) {
        if(connection->id() == id) {
            auto* f = connection->from();
            auto* t = connection->to();

            if(Output* output = dynamic_cast<Output*>(f)) {
                Input* input = dynamic_cast<Input*>(t);
                return Command::Ptr(new command::DeleteMessageConnection(output, input));

            } else if(Trigger* trigger = dynamic_cast<Trigger*>(f)) {
                Slot* slot = dynamic_cast<Slot*>(t);
                return Command::Ptr(new command::DeleteSignalConnection(trigger, slot));
            }

        }
    }

    return Command::Ptr();
}

Command::Ptr CommandFactory::deleteConnectionFulcrumCommand(int connection, int fulcrum)
{
    return Command::Ptr(new command::DeleteFulcrum(connection, fulcrum));
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(int connection)
{
    command::Meta::Ptr meta(new command::Meta("Delete All Connection Fulcrums"));

    if(connection >= 0) {
        int n = graph_->getConnectionWithId(connection)->getFulcrumCount();
        for(int i = n - 1; i >= 0; --i) {
            meta->add(deleteConnectionFulcrumCommand(connection, i));
        }
    }

    return meta;
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(ConnectionPtr connection)
{
    return deleteAllConnectionFulcrumsCommand(graph_->getConnectionId(connection));
}
