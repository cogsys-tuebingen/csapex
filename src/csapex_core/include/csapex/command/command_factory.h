#ifndef COMMAND_FACTORY_H
#define COMMAND_FACTORY_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex_command_export.h>
#include <csapex/scheduling/scheduling_fwd.h>

namespace csapex
{

class CSAPEX_COMMAND_EXPORT CommandFactory
{
public:
    CommandFactory(GraphFacade *root, const AUUID& graph_id);
    CommandFactory(GraphFacade *root);

public:
    CommandPtr deleteAllNodes(const std::vector<UUID>& uuids);
    CommandPtr deleteAllConnectionsFromNodes(const std::vector<UUID>& uuids);

    CommandPtr addConnection(const UUID& from, const UUID& to, bool active);

    CommandPtr removeAllConnectionsCmd(ConnectorPtr input);
    CommandPtr removeAllConnectionsCmd(Connector* input);

    CommandPtr removeConnectionCmd(Connector* output, Connection* connection);


    CommandPtr moveConnections(const UUID& from, const UUID& to);
    CommandPtr moveConnections(Connector* from, Connector* to);


    CommandPtr setConnectionActive(int connection, bool active);

    CommandPtr deleteConnectionFulcrumCommand(int connection, int fulcrum);
    CommandPtr deleteAllConnectionFulcrumsCommand(int connection);
    CommandPtr deleteAllConnectionFulcrumsCommand(ConnectionPtr connection);
    CommandPtr deleteConnectionByIdCommand(int id);
    CommandPtr clearCommand();

    CommandPtr switchThreadRecursively(const std::vector<UUID> &node_uuids, int id);
    CommandPtr muteRecursively(const std::vector<UUID> &node_uuids, bool muted);
    CommandPtr setMaximumFrequencyRecursively(const std::vector<UUID> &node_uuids, double frequency);
    CommandPtr setLoggerLevelRecursively(const std::vector<UUID> &node_uuids, int level);


    CommandPtr createVariadicInput(const AUUID& node_uuid, TokenDataConstPtr connection_type, const std::string& label, bool optional);
    CommandPtr createVariadicOutput(const AUUID& node_uuid, TokenDataConstPtr connection_type, const std::string& label);
    CommandPtr createVariadicEvent(const AUUID& node_uuid, const std::string& label);
    CommandPtr createVariadicSlot(const AUUID& node_uuid, const std::string& label);

    CommandPtr createVariadicPort(const AUUID& node_uuid, ConnectorType port_type, TokenDataConstPtr connection_type, const std::string &label);
    CommandPtr createVariadicPort(const AUUID& node_uuid, ConnectorType port_type, TokenDataConstPtr connection_type, const std::string& label, bool optional);

    CommandPtr deleteThreadGroup(csapex::ThreadGroup * group);

private:
    GraphFacade* getGraphFacade() const;


private:
    GraphFacade* root_;
    AUUID graph_uuid;
};

}

#endif // COMMAND_FACTORY_H

