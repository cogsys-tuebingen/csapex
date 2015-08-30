/// HEADER
#include <csapex/command/command_factory.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/connection.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_worker.h>
#include <csapex/utility/assert.h>

using namespace csapex;

/// CONNECTORS
///
///

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
    Command::Ptr cmd(new command::DeleteConnection(input->getSource(), input));
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Output* output, Connection* connection) {
    return Command::Ptr (new command::DeleteConnection(output, connection->to()));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Output* output)
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    for(ConnectionPtr connection : output->getConnections()) {
        Command::Ptr removeThis(new command::DeleteConnection(output, connection->to()));
        removeAll->add(removeThis);
    }

    return removeAll;
}


Command::Ptr CommandFactory::removeAllConnectionsCmd(Slot* slot)
{
    command::Meta::Ptr cmd(new command::Meta("Delete sources"));
    for(Trigger* source : slot->getSources()) {
        cmd->add(Command::Ptr(new command::DeleteConnection(source, slot)));
    }
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Trigger* trigger, Slot* other_side)
{
    return Command::Ptr (new command::DeleteConnection(trigger, other_side));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Trigger* trigger)
{
    command::Meta::Ptr removeAll(new command::Meta("Remove All Connections"));

    for(Slot* target : trigger->getTargets()) {
        Command::Ptr removeThis(new command::DeleteConnection(trigger, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}




/// GRAPH
///
///

Command::Ptr CommandFactory::clearCommand(Graph* graph)
{
    command::Meta::Ptr clear(new command::Meta("Clear Graph"));

    for(auto node : graph->getAllNodeWorkers()) {
        clear->add(Command::Ptr (new command::DeleteNode(node->getUUID())));
    }

    return clear;
}

Command::Ptr CommandFactory::deleteConnectionByIdCommand(Graph* graph, int id)
{
    for(const auto& connection : graph->getConnections()) {
        if(connection->id() == id) {
            return Command::Ptr(new command::DeleteConnection(connection->from(), connection->to()));
        }
    }

    return Command::Ptr();
}

Command::Ptr CommandFactory::deleteConnectionFulcrumCommand(Graph* graph, int connection, int fulcrum)
{
    return Command::Ptr(new command::DeleteFulcrum(connection, fulcrum));
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(Graph* graph, int connection)
{
    command::Meta::Ptr meta(new command::Meta("Delete All Connection Fulcrums"));

    if(connection >= 0) {
        int n = graph->getConnectionWithId(connection)->getFulcrumCount();
        for(int i = n - 1; i >= 0; --i) {
            meta->add(deleteConnectionFulcrumCommand(graph, connection, i));
        }
    }

    return meta;
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(Graph* graph, ConnectionPtr connection)
{
    return deleteAllConnectionFulcrumsCommand(graph, graph->getConnectionId(connection));
}
