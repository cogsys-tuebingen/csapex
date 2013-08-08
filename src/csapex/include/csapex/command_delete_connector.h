#ifndef COMMAND_DELETE_CONNECTOR_H
#define COMMAND_DELETE_CONNECTOR_H

/// COMPONENT
#include "command.h"

namespace csapex
{
class Connector;
class ConnectorIn;
class ConnectorOut;

struct DeleteConnector : public Command
{
    DeleteConnector(Connector *_c);

protected:
    bool execute(Graph& graph);
    bool undo(Graph& graph);
    bool redo(Graph& graph);

    bool refresh(Graph &graph);

private:
    bool       in;
    Connector* c;

    Command::Ptr    delete_connections;

    std::string c_uuid;

};
}
#endif // COMMAND_DELETE_CONNECTOR_H
