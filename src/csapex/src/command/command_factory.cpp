/// HEADER
#include <csapex/command/command_factory.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/delete_msg_connection.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/modify_connection.h>
#include <csapex/command/add_msg_connection.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/connection.h>
#include <csapex/command/meta.h>
#include <csapex/command/delete_connection.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/utility/assert.h>
#include <csapex/model/graph_facade.h>

using namespace csapex;
using namespace csapex::command;

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

Command::Ptr CommandFactory::addConnection(const UUID &from, const UUID &to, bool active)
{
    GraphFacade* graph_facade = getGraphFacade();
    Graph* graph = graph_facade->getGraph();

    auto from_c = graph->findConnectorNoThrow(from);

    if(dynamic_cast<Output*>(from_c)) {
        return std::make_shared<AddMessageConnection>(graph_uuid, from, to, active);
    } else if(dynamic_cast<Input*>(from_c)) {
        return std::make_shared<AddMessageConnection>(graph_uuid, to, from, active);
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
    if(Event* trigger = dynamic_cast<Event*>(c)) {
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
    apex_assert_hard(!input->isVirtual() && !output->isVirtual());
    Command::Ptr cmd(new DeleteMessageConnection(graph_uuid, output, input));
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Output* output, Connection* connection) {
    Input* input = dynamic_cast<Input*>(connection->to());
    apex_assert_hard(!input->isVirtual() && !output->isVirtual());
    return Command::Ptr (new DeleteMessageConnection(graph_uuid, output, input));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Output* output)
{
    Meta::Ptr removeAll(new Meta(graph_uuid, "Remove All Connections"));

    for(ConnectionPtr connection : output->getConnections()) {
        Input* input = dynamic_cast<Input*>(connection->to());
        if(!input->isVirtual() && !output->isVirtual()) {
            Command::Ptr removeThis(new DeleteMessageConnection(graph_uuid, output, input));
            removeAll->add(removeThis);
        }
    }

    return removeAll;
}



/// graph_
///
///

Command::Ptr CommandFactory::setConnectionActive(int connection, bool active)
{
    return Command::Ptr(new ModifyConnection(graph_uuid, connection, active));
}

Command::Ptr CommandFactory::clearCommand()
{
    Meta::Ptr clear(new Meta(graph_uuid, "Clear graph_"));

    for(auto node : root_->getGraph()->getAllNodeHandles()) {
        clear->add(Command::Ptr (new DeleteNode(graph_uuid, node->getUUID())));
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
                return Command::Ptr(new DeleteMessageConnection(graph_uuid, output, input));
            }

        }
    }

    return Command::Ptr();
}

Command::Ptr CommandFactory::deleteConnectionFulcrumCommand(int connection, int fulcrum)
{
    return Command::Ptr(new DeleteFulcrum(graph_uuid, connection, fulcrum));
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(int connection)
{
    Meta::Ptr meta(new Meta(graph_uuid, "Delete All Connection Fulcrums"));

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

Command::Ptr CommandFactory::moveConnections(const UUID& from, const UUID& to)
{
    Graph* graph = getGraphFacade()->getGraph();
    Connectable *f = graph->findConnector(from);
    Connectable *t = graph->findConnector(to);
    return moveConnections(f, t);
}


Command::Ptr CommandFactory::moveConnections(Connectable *from, Connectable *to)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard((from->isOutput() && to->isOutput()) ||
                     (from->isInput() && to->isInput()));

    apex_assert_hard(!from->isVirtual());
    apex_assert_hard(!to->isVirtual());

    bool is_output = from->isOutput();

//    UUID from_uuid = from->getUUID();
    UUID to_uuid = to->getUUID();

    AUUID parent_uuid(graph_uuid);

    Meta::Ptr meta(new Meta(parent_uuid, "MoveConnection"));

    if(is_output) {
        Output* out = dynamic_cast<Output*>(from);
        if(out) {
            for(ConnectionPtr c : out->getConnections()) {
                if(!c) {
                    continue;
                }
                Input* input = dynamic_cast<Input*>(c->to());
                if(input && !input->isVirtual()) {
                    meta->add(Command::Ptr(new DeleteMessageConnection(parent_uuid, out, input)));
                    meta->add(Command::Ptr(new AddMessageConnection(parent_uuid, to_uuid, input->getUUID(), c->isActive())));
                }
            }
        }

    } else {
        Input* in = dynamic_cast<Input*>(from);

        if(in && !in->isVirtual()) {
            ConnectionPtr c = in->getConnections().front();
            Output* target = dynamic_cast<Output*>(c->from());
            meta->add(Command::Ptr(new DeleteMessageConnection(parent_uuid, target, in)));
            meta->add(Command::Ptr(new AddMessageConnection(parent_uuid, target->getUUID(), to_uuid, c->isActive())));
        }
    }

    return meta;
}


CommandPtr CommandFactory::createVariadicInput(const AUUID& node_uuid, TokenDataConstPtr connection_type, const std::string& label, bool optional)
{
    return createVariadicPort(node_uuid, ConnectorType::INPUT, connection_type, label, optional);
}

CommandPtr CommandFactory::createVariadicOutput(const AUUID& node_uuid, TokenDataConstPtr connection_type, const std::string& label)
{
    return createVariadicPort(node_uuid, ConnectorType::OUTPUT, connection_type, label, false);
}

CommandPtr CommandFactory::createVariadicEvent(const AUUID& node_uuid, const std::string& label)
{
    return createVariadicPort(node_uuid, ConnectorType::EVENT, connection_types::makeEmpty<connection_types::AnyMessage>(), label, false);
}

CommandPtr CommandFactory::createVariadicSlot(const AUUID& node_uuid, const std::string& label)
{
    return createVariadicPort(node_uuid, ConnectorType::SLOT_T, connection_types::makeEmpty<connection_types::AnyMessage>(), label, false);
}

CommandPtr CommandFactory::createVariadicPort(const AUUID& node_uuid, ConnectorType port_type, TokenDataConstPtr connection_type, const std::string& label)
{
    std::shared_ptr<AddVariadicConnector> res = std::make_shared<AddVariadicConnector>(graph_uuid, node_uuid, port_type, connection_type, label);
    return res;
}
CommandPtr CommandFactory::createVariadicPort(const AUUID& node_uuid, ConnectorType port_type, TokenDataConstPtr connection_type, const std::string& label, bool optional)
{
    std::shared_ptr<AddVariadicConnector> res = std::make_shared<AddVariadicConnector>(graph_uuid, node_uuid, port_type, connection_type, label);
    res->setOptional(optional);
    return res;
}
