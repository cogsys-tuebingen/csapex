#ifndef COMMAND_FACTORY_H
#define COMMAND_FACTORY_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/command/command_fwd.h>

namespace csapex
{

class CommandFactory
{
public:
    static CommandPtr removeAllConnectionsCmd(ConnectablePtr input);
    static CommandPtr removeAllConnectionsCmd(Connectable* input);

    static CommandPtr removeAllConnectionsCmd(Input* input);
    static CommandPtr removeAllConnectionsCmd(Output* output);
    static CommandPtr removeAllConnectionsCmd(Slot* slot);
    static CommandPtr removeAllConnectionsCmd(Trigger* trigger);

    static CommandPtr removeConnectionCmd(Output* output, Connection* connection);
    static CommandPtr removeConnectionCmd(Trigger* trigger, Slot* other_side);



    static CommandPtr deleteConnectionFulcrumCommand(Graph* graph, int connection, int fulcrum);
    static CommandPtr deleteAllConnectionFulcrumsCommand(Graph* graph, int connection);
    static CommandPtr deleteAllConnectionFulcrumsCommand(Graph* graph, ConnectionPtr connection);
    static CommandPtr deleteConnectionByIdCommand(Graph* graph, int id);
    static CommandPtr clearCommand(Graph* graph);
};

}

#endif // COMMAND_FACTORY_H

