#ifndef COMMAND_FACTORY_H
#define COMMAND_FACTORY_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/command/command_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

class CommandFactory
{
public:
    CommandFactory(GraphFacade *root, const UUID& graph_id);
    CommandFactory(GraphFacade *root);

public:
    CommandPtr addConnection(const UUID& from, const UUID& to);

    CommandPtr removeAllConnectionsCmd(ConnectablePtr input);
    CommandPtr removeAllConnectionsCmd(Connectable* input);

    CommandPtr removeAllConnectionsCmd(Input* input);
    CommandPtr removeAllConnectionsCmd(Output* output);
    CommandPtr removeAllConnectionsCmd(Slot* slot);
    CommandPtr removeAllConnectionsCmd(Trigger* trigger);

    CommandPtr removeConnectionCmd(Output* output, Connection* connection);
    CommandPtr removeConnectionCmd(Trigger* trigger, Slot* other_side);



    CommandPtr deleteConnectionFulcrumCommand(int connection, int fulcrum);
    CommandPtr deleteAllConnectionFulcrumsCommand(int connection);
    CommandPtr deleteAllConnectionFulcrumsCommand(ConnectionPtr connection);
    CommandPtr deleteConnectionByIdCommand(int id);
    CommandPtr clearCommand();

private:
    GraphFacade* getGraphFacade() const;

private:
    GraphFacade* root_;
    UUID graph_uuid;
};

}

#endif // COMMAND_FACTORY_H

