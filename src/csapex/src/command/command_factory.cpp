/// HEADER
#include <csapex/command/command_factory.h>

/// COMPONENT
#include <csapex/command/add_connection.h>
#include <csapex/command/add_variadic_connector.h>
#include <csapex/command/command.h>
#include <csapex/command/clear_graph.h>
#include <csapex/command/delete_connection.h>
#include <csapex/command/delete_fulcrum.h>
#include <csapex/command/delete_node.h>
#include <csapex/command/delete_thread.h>
#include <csapex/command/meta.h>
#include <csapex/command/modify_connection.h>
#include <csapex/command/mute_node.h>
#include <csapex/command/set_logger_level.h>
#include <csapex/command/set_max_execution_frequency.h>
#include <csapex/command/switch_thread.h>
#include <csapex/command/switch_thread.h>
#include <csapex/model/connection.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_facade.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_runner.h>
#include <csapex/model/node_runner.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/any_message.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/scheduling/scheduler.h>
#include <csapex/scheduling/thread_group.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/utility/assert.h>

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

    std::set<std::pair<UUID, UUID>> connections;

    for(const UUID& uuid : uuids) {
        NodeFacadePtr nf = root_->findNodeFacade(uuid);
        for(const ConnectorDescription& connectable : nf->getExternalConnectors()) {
            for(const ConnectorDescription::Target& target : connectable.targets) {
                UUID out = connectable.isOutput() ? connectable.id : target.auuid.reshapeSoft(2);
                UUID in = connectable.isOutput() ? target.auuid.reshapeSoft(2) : connectable.id;
                connections.insert(std::make_pair(out, in));
            }
        }
    }

    for(const auto& pair : connections) {
        meta->add(std::make_shared<DeleteConnection>(graph_uuid, pair.first, pair.second));
    }

    return meta;
}

/// CONNECTORS
///
///

Command::Ptr CommandFactory::addConnection(const UUID &from, const UUID &to, bool active)
{
    auto from_c = getGraphFacade()->findConnectorNoThrow(from);

    if(from_c->isOutput()) {
        return std::make_shared<AddConnection>(graph_uuid, from, to, active);
    } else if(from_c->isInput()) {
        return std::make_shared<AddConnection>(graph_uuid, to, from, active);
    }
    return nullptr;
}

Command::Ptr CommandFactory::removeAllConnectionsCmd(ConnectorPtr c)
{
    return removeAllConnectionsCmd(c.get());
}


Command::Ptr CommandFactory::removeAllConnectionsCmd(Connector* c)
{
    Meta::Ptr removeAll(new Meta(graph_uuid, "Remove All Connections", true));

    UUID this_side = c->getUUID();
    for(const UUID& other_side : c->getConnectedPorts()) {
        if(c->isOutput()) {
            removeAll->add(std::make_shared<DeleteConnection>(graph_uuid, this_side, other_side));

        } else {
            removeAll->add(std::make_shared<DeleteConnection>(graph_uuid, other_side, this_side));
        }
    }

    return removeAll;
}

Command::Ptr CommandFactory::removeConnectionCmd(Connector* output, Connection* connection) {
    apex_assert_hard(!output->isVirtual());
    InputPtr input = connection->to();
    apex_assert_hard(!input->isVirtual());
    return Command::Ptr (new DeleteConnection(graph_uuid, output->getUUID(), input->getUUID()));
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
    return std::make_shared<ClearGraph>(graph_uuid);
}

Command::Ptr CommandFactory::deleteConnectionByIdCommand(int id)
{
    GraphFacade* graph_facade = getGraphFacade();

    for(const ConnectionInformation& connection : graph_facade->enumerateAllConnections()) {
        if(connection.id == id) {
            return Command::Ptr(new DeleteConnection(graph_uuid, connection.from, connection.to));

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

        ConnectionInformation ci = graph_facade->getConnectionWithId(connection);
        int n = ci.fulcrums.size();
        for(int i = n - 1; i >= 0; --i) {
            meta->add(std::make_shared<DeleteFulcrum>(graph_uuid, connection, i));
        }
    }

    return meta;
}

Command::Ptr CommandFactory::deleteAllConnectionFulcrumsCommand(ConnectionPtr connection)
{
    return deleteAllConnectionFulcrumsCommand(connection->id());
}

GraphFacade* CommandFactory::getGraphFacade() const
{
    if(graph_uuid.empty()) {
        return root_;

    } else if(root_->getAbsoluteUUID() == graph_uuid) {
        return root_;

    } else {
        return root_->getSubGraph(graph_uuid).get();
    }
}

Command::Ptr CommandFactory::moveConnections(const UUID& from, const UUID& to)
{
    GraphFacade* graph = getGraphFacade();
    ConnectorPtr f = graph->findConnector(from);
    ConnectorPtr t = graph->findConnector(to);
    return moveConnections(f.get(), t.get());
}


Command::Ptr CommandFactory::moveConnections(Connector *from, Connector *to)
{
    apex_assert_hard(from);
    apex_assert_hard(to);
    apex_assert_hard((from->isOutput() && to->isOutput()) ||
                     (from->isInput() && to->isInput()));

    //    UUID from_uuid = from->getUUID();
    UUID to_uuid = to->getUUID();

    AUUID parent_uuid(graph_uuid);

    Meta::Ptr meta(new Meta(parent_uuid, "MoveConnection", true));

    for(const UUID& other_id : from->getConnectedPorts()) {
        if(ConnectorPtr c = root_->findConnectorNoThrow(other_id)) {
            bool is_active = from->isActivelyConnectedTo(other_id);
            if(from->isOutput()) {
                meta->add(Command::Ptr(new DeleteConnection(parent_uuid, from->getUUID(), c->getUUID())));
                meta->add(Command::Ptr(new AddConnection(parent_uuid, to_uuid, c->getUUID(), is_active)));
            } else {
                meta->add(Command::Ptr(new DeleteConnection(parent_uuid, c->getUUID(), from->getUUID())));
                meta->add(Command::Ptr(new AddConnection(parent_uuid, c->getUUID(), to_uuid, is_active)));
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


namespace {
template <typename Lambda>
void foreachNode(GraphFacade* graph_facade, const UUID &node_uuid, Lambda fn)
{
    // evalute at the node
    NodeFacadePtr nf = graph_facade->findNodeFacade(node_uuid);
    apex_assert_hard(nf);
    fn(graph_facade, nf);

    // recursive call into child graphs
    if(nf->isGraph()) {
        GraphFacadePtr subgraph = graph_facade->getSubGraph(nf->getUUID());
        if(subgraph) {
            GraphFacadePtr child_facade = graph_facade->getSubGraph(node_uuid);
            apex_assert_hard(child_facade);

            for(const UUID& child : graph_facade->enumerateAllNodes()) {
                foreachNode(child_facade.get(), child, fn);
            }
        }
    }
}

template <typename Lambda>
void foreachNode(GraphFacade* graph_facade, const std::vector<UUID> &node_uuids, Lambda fn)
{
    for(const UUID& uuid: node_uuids) {
        foreachNode(graph_facade, uuid, fn);
    }
}
}


CommandPtr CommandFactory::switchThreadRecursively(const std::vector<UUID> &node_uuids, int id)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, "change thread"));
    for(const UUID& uuid: node_uuids) {
        NodeFacadePtr node = root_->findNodeFacade(uuid);
        int old_thread_id = node->getSchedulerId();

        foreachNode(root_, uuid, [&](GraphFacade* graph_facade, const NodeFacadePtr& nf) {
            if(node->getSchedulerId() == old_thread_id)  {
                cmd->add(Command::Ptr(new command::SwitchThread(graph_facade->getAbsoluteUUID(), nf->getUUID(), id)));
            }
        });
    }
    return cmd;
}

CommandPtr CommandFactory::muteRecursively(const std::vector<UUID> &node_uuids, bool muted)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, muted ? "mute nodes" : "unmute nodes"));
    foreachNode(root_, node_uuids, [&](GraphFacade* graph_facade, const NodeFacadePtr& nh) {
        if(nh->getNodeState()->isMuted() != muted) {
            cmd->add(Command::Ptr(new command::MuteNode(graph_facade->getAbsoluteUUID(), nh->getUUID(), muted)));
        }
    });
    return cmd;
}

CommandPtr CommandFactory::setMaximumFrequencyRecursively(const std::vector<UUID> &node_uuids, double frequency)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, frequency == 0.0 ? "set unbounded frequency" : "set maximum frequency"));
    foreachNode(root_, node_uuids, [&](GraphFacade* graph_facade, const NodeFacadePtr& nh) {
        cmd->add(Command::Ptr(new command::SetMaximumExecutionFrequency(graph_facade->getAbsoluteUUID(), nh->getUUID(), frequency)));
    });
    return cmd;
}

CommandPtr CommandFactory::setLoggerLevelRecursively(const std::vector<UUID> &node_uuids, int level)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, "set logger level"));
    foreachNode(root_, node_uuids, [&](GraphFacade* graph_facade, const NodeFacadePtr& nh) {
        cmd->add(Command::Ptr(new command::SetLoggerLevel(graph_facade->getAbsoluteUUID(), nh->getUUID(), level)));
    });
    return cmd;
}

CommandPtr CommandFactory::deleteThreadGroup(ThreadGroup* group)
{
    command::Meta::Ptr cmd(new command::Meta(graph_uuid, "delete thread group"));

    // first move all generators to the default thread
    for(const TaskGeneratorPtr& generator : *group) {
        cmd->add(std::make_shared<command::SwitchThread>(graph_uuid, generator->getUUID(), ThreadGroup::DEFAULT_GROUP_ID));
    }

    // then delete the group itself
    cmd->add(std::make_shared<command::DeleteThread>(group->id()));

    return cmd;
}
