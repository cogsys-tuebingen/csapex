#ifndef COMMAND_FACTORY_H
#define COMMAND_FACTORY_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>

namespace csapex
{

class CommandFactory
{
public:
    CommandFactory(GraphFacade *root, const AUUID& graph_id);
    CommandFactory(GraphFacade *root);

public:
    CommandPtr addConnection(const UUID& from, const UUID& to);

    CommandPtr removeAllConnectionsCmd(ConnectablePtr input);
    CommandPtr removeAllConnectionsCmd(Connectable* input);

    CommandPtr removeAllConnectionsCmd(Input* input);
    CommandPtr removeAllConnectionsCmd(Output* output);
    CommandPtr removeAllConnectionsCmd(Slot* slot);
    CommandPtr removeAllConnectionsCmd(Event* trigger);

    CommandPtr removeConnectionCmd(Output* output, Connection* connection);
    CommandPtr removeConnectionCmd(Event* trigger, Slot* other_side);


    CommandPtr moveConnections(const UUID& from, const UUID& to);
    CommandPtr moveConnections(Connectable* from, Connectable* to);


    CommandPtr deleteConnectionFulcrumCommand(int connection, int fulcrum);
    CommandPtr deleteAllConnectionFulcrumsCommand(int connection);
    CommandPtr deleteAllConnectionFulcrumsCommand(ConnectionPtr connection);
    CommandPtr deleteConnectionByIdCommand(int id);
    CommandPtr clearCommand();


    CommandPtr createVariadicInput(const AUUID& node_uuid, ConnectionTypeConstPtr connection_type, const std::string& label, bool optional);
    CommandPtr createVariadicOutput(const AUUID& node_uuid, ConnectionTypeConstPtr connection_type, const std::string& label);
    CommandPtr createVariadicEvent(const AUUID& node_uuid, const std::string& label);
    CommandPtr createVariadicSlot(const AUUID& node_uuid, const std::string& label);

    CommandPtr createVariadicPort(const AUUID& node_uuid, ConnectorType port_type, ConnectionTypeConstPtr connection_type);
    CommandPtr createVariadicPort(const AUUID& node_uuid, ConnectorType port_type, ConnectionTypeConstPtr connection_type, const std::string& label, bool optional);

private:
    GraphFacade* getGraphFacade() const;

private:
    GraphFacade* root_;
    AUUID graph_uuid;
};

}

#endif // COMMAND_FACTORY_H

