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
#include <csapex/model/graph_facade.h>

using namespace csapex;

CommandFactory::CommandFactory(GraphFacade *root, const AUUID &graph_id)
    : root_(root), graph_uuid(graph_id)
{

}
CommandFactory::CommandFactory(GraphFacade *root)
    : root_(root), graph_uuid(root->getAbsoluteUUID())
{

}

/// CONNECTORS
///
///

Command::Ptr CommandFactory::addConnection(const UUID &from, const UUID &to)
{
    GraphFacade* graph_facade = getGraphFacade();
    Graph* graph = graph_facade->getGraph();

    auto from_c = graph->findConnector(from);

    if(dynamic_cast<Output*>(from_c)) {
        return std::make_shared<command::AddMessageConnection>(graph_uuid, from, to);
    } else if(dynamic_cast<Input*>(from_c)) {
        return std::make_shared<command::AddMessageConnection>(graph_uuid, to, from);
    }
    if(dynamic_cast<Trigger*>(from_c)) {
        return std::make_shared<command::AddSignalConnection>(graph_uuid, from, to);
    } else if(dynamic_cast<Slot*>(from_c)) {
        return std::make_shared<command::AddSignalConnection>(graph_uuid, to, from);
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
    Command::Ptr cmd(new command::DeleteMessageConnection(graph_uuid, output, input));
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Output* output, Connection* connection) {
    Input* input = dynamic_cast<Input*>(connection->to());
    return Command::Ptr (new command::DeleteMessageConnection(graph_uuid, output, input));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Output* output)
{
    command::Meta::Ptr removeAll(new command::Meta(graph_uuid, "Remove All Connections"));

    for(ConnectionPtr connection : output->getConnections()) {
        Input* input = dynamic_cast<Input*>(connection->to());
        Command::Ptr removeThis(new command::DeleteMessageConnection(graph_uuid, output, input));
        removeAll->add(removeThis);
    }

    return removeAll;
}


Command::Ptr CommandFactory::removeAllConnectionsCmd(Slot* slot)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, "Delete sources"));
    for(Trigger* source : slot->getSources()) {
        cmd->add(Command::Ptr(new command::DeleteSignalConnection(graph_uuid, source, slot)));
    }
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Trigger* trigger, Slot* other_side)
{
    return Command::Ptr (new command::DeleteSignalConnection(graph_uuid, trigger, other_side));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Trigger* trigger)
{
    command::Meta::Ptr removeAll(new command::Meta(graph_uuid, "Remove All Connections"));

    for(Slot* target : trigger->getTargets()) {
        Command::Ptr removeThis(new command::DeleteSignalConnection(graph_uuid, trigger, target));
        removeAll->add(removeThis);
    }

    return removeAll;
}




/// graph_
///
///

Command::Ptr CommandFactory::clearCommand()
{
    command::Meta::Ptr clear(new command::Meta(graph_uuid, "Clear graph_"));

    for(auto node : root_->getGraph()->getAllNodeHandles()) {
        clear->add(Command::Ptr (new command::DeleteNode(graph_uuid, node->getUUID())));
    }

    return clear;
}

Command::Ptr CommandFactory::deleteConnectionByIdCommand(int id)
{
    GraphFacade* graph_facade = getGraphFacade();
    Graph* graph = graph_facade->getGraph();
    for(const auto& connection : graph->getConnections()) {
        if(connection->id() == id) {
            auto* f = connection->from();
            auto* t = connection->to();

            if(Output* output = dynamic_cast<Output*>(f)) {
                Input* input = dynamic_cast<Input*>(t);
                return Command::Ptr(new command::DeleteMessageConnection(graph_uuid, output, input));

            } else if(Trigger* trigger = dynamic_cast<Trigger*>(f)) {
                Slot* slot = dynamic_cast<Slot*>(t);
                return Command::Ptr(new command::DeleteSignalConnection(graph_uuid, trigger, slot));
            }

        }
    }

    return Command::Ptr();
}

Command::Ptr CommandFactory::deleteConnectionFulcrumCommand(int connection, int fulcrum)
{
    return Command::Ptr(new command::DeleteFulcrum(graph_uuid, connection, fulcrum));
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(int connection)
{
    command::Meta::Ptr meta(new command::Meta(graph_uuid, "Delete All Connection Fulcrums"));

    if(connection >= 0) {
        GraphFacade* graph_facade = getGraphFacade();
        Graph* graph = graph_facade->getGraph();

        int n = graph->getConnectionWithId(connection)->getFulcrumCount();
        for(int i = n - 1; i >= 0; --i) {
            meta->add(deleteConnectionFulcrumCommand(connection, i));
        }
    }

    return meta;
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(ConnectionPtr connection)
{
    return deleteAllConnectionFulcrumsCommand(root_->getGraph()->getConnectionId(connection));
}

GraphFacade* CommandFactory::getGraphFacade() const
{
    if(graph_uuid.empty()) {
        return root_;

    } else if(root_->getAbsoluteUUID() == graph_uuid) {
        return root_;

    } else {
        return root_->getSubGraph(graph_uuid);
    }
}
