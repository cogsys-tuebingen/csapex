/// HEADER
#include <csapex/command/command_factory.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/modify_connection.h>
#include <csapex/command/add_connection.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/switch_thread.h>
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
#include <csapex/model/node_runner.h>
#include <csapex/scheduling/scheduler.h>

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

CommandPtr CommandFactory::deleteAllNodes(const std::vector<UUID> &uuids)
{
    command::Meta::Ptr meta(new command::Meta(root_->getAbsoluteUUID(), "delete selected boxes", true));

    meta->add(deleteAllConnectionsFromNodes(uuids));

    for(const UUID& uuid : uuids) {
        meta->add(Command::Ptr(new command::DeleteNode(root_->getAbsoluteUUID(), uuid)));
    }
    return meta;
}


CommandPtr CommandFactory::deleteAllConnectionsFromNodes(const std::vector<UUID> &uuids)
{
    command::Meta::Ptr meta(new command::Meta(root_->getAbsoluteUUID(), "delete connections from boxes", true));


    std::set<ConnectionPtr> connections;

    for(const UUID& uuid : uuids) {
        NodeHandle* nh = root_->getGraph()->findNodeHandle(uuid);
        for(ConnectablePtr connectable : nh->getExternalConnectors()) {
            for(ConnectionPtr c : connectable->getConnections()) {
                connections.insert(c);
            }
        }
    }

    for(ConnectionPtr c : connections) {
        meta->add(std::make_shared<DeleteConnection>(graph_uuid, c->from().get(), c->to().get()));
    }
    return meta;
}

/// CONNECTORS
///
///

Command::Ptr CommandFactory::addConnection(const UUID &from, const UUID &to, bool active)
{
    GraphFacade* graph_facade = getGraphFacade();
    Graph* graph = graph_facade->getGraph();

    auto from_c = graph->findConnectorNoThrow(from);

    if(std::dynamic_pointer_cast<Output>(from_c)) {
        return std::make_shared<AddConnection>(graph_uuid, from, to, active);
    } else if(std::dynamic_pointer_cast<Input>(from_c)) {
        return std::make_shared<AddConnection>(graph_uuid, to, from, active);
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
    OutputPtr output = input->getSource();
    Command::Ptr cmd(new DeleteConnection(graph_uuid, output.get(), input));
    return cmd;
}

Command::Ptr CommandFactory::removeConnectionCmd(Output* output, Connection* connection) {
    InputPtr input = connection->to();
    return Command::Ptr (new DeleteConnection(graph_uuid, output, input.get()));
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(Output* output)
{
    Meta::Ptr removeAll(new Meta(graph_uuid, "Remove All Connections", true));

    for(ConnectionPtr connection : output->getConnections()) {
        InputPtr input = connection->to();
        Command::Ptr removeThis(new DeleteConnection(graph_uuid, output, input.get()));
        removeAll->add(removeThis);
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
    Meta::Ptr clear(new Meta(graph_uuid, "Clear graph_", true));

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
            OutputPtr out = connection->from();
            InputPtr in = connection->to();

            return Command::Ptr(new DeleteConnection(graph_uuid, out.get(), in.get()));

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
    ConnectablePtr f = graph->findConnector(from);
    ConnectablePtr t = graph->findConnector(to);
    return moveConnections(f.get(), t.get());
}


Command::Ptr CommandFactory::moveConnections(Connectable *from, Connectable *to)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard((from->isOutput() && to->isOutput()) ||
                     (from->isInput() && to->isInput()));

    bool is_output = from->isOutput();

    //    UUID from_uuid = from->getUUID();
    UUID to_uuid = to->getUUID();

    AUUID parent_uuid(graph_uuid);

    Meta::Ptr meta(new Meta(parent_uuid, "MoveConnection", true));

    if(is_output) {
        Output* out = dynamic_cast<Output*>(from);
        if(out) {
            for(ConnectionPtr c : out->getConnections()) {
                if(!c) {
                    continue;
                }
                InputPtr input = c->to();
                if(input) {
                    meta->add(Command::Ptr(new DeleteConnection(parent_uuid, out, input.get())));
                    meta->add(Command::Ptr(new AddConnection(parent_uuid, to_uuid, input->getUUID(), c->isActive())));
                }
            }
        }

    } else {
        Input* in = dynamic_cast<Input*>(from);

        if(in) {
            for(ConnectionPtr c : in->getConnections()) {
                if(!c) {
                    continue;
                }
                OutputPtr output = c->from();
                meta->add(Command::Ptr(new DeleteConnection(parent_uuid, output.get(), in)));
                meta->add(Command::Ptr(new AddConnection(parent_uuid, output->getUUID(), to_uuid, c->isActive())));
            }
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


void CommandFactory::switchThreadRecursively(const UUID &node_uuid, int old_thread_id, int id, std::shared_ptr<command::Meta>& out)
{
    NodeHandle* node = root_->getGraph()->findNodeHandle(node_uuid);
    if(NodeRunnerPtr runner = node->getNodeRunner()) {
        if(runner->getScheduler()->id() == old_thread_id)  {
            out->add(Command::Ptr(new command::SwitchThread(graph_uuid, node_uuid, id)));
        }
    }

    if(node && node->isGraph()) {
        //        if(GraphPtr graph = std::dynamic_pointer_cast<Graph>(node->getNode().lock())) {
        //            for(NodeHandle* child : graph->getAllNodeHandles()) {
        //                switchThreadRecursively(UUIDProvider::makeDerivedUUID_forced(node_uuid, child->getUUID().getFullName()), old_thread_id, id, out);
        //            }
        //        }
        if(GraphPtr graph = std::dynamic_pointer_cast<Graph>(node->getNode().lock())) {
            CommandFactory ccf(root_->getSubGraph(node_uuid));
            for(NodeHandle* child : graph->getAllNodeHandles()) {
                ccf.switchThreadRecursively(child->getUUID(), old_thread_id, id, out);
            }
        }
    }
}


CommandPtr CommandFactory::switchThreadRecursively(const std::vector<UUID> &node_uuids, int id)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, "change thread"));
    for(const UUID& uuid: node_uuids) {
        NodeHandle* node = root_->getGraph()->findNodeHandle(uuid);
        if(NodeRunnerPtr runner = node->getNodeRunner()) {
            switchThreadRecursively(uuid, runner->getScheduler()->id(), id, cmd);
        }
    }
    return cmd;
}
