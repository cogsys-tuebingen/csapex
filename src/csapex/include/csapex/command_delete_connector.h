#ifndef COMMAND_DELETE_CONNECTOR_H
#define COMMAND_DELETE_CONNECTOR_H

/// COMPONENT
#include "command.h"

namespace csapex
{

namespace command
{

struct DeleteConnector : public Command
{
    DeleteConnector(Connector *_c);

protected:
    bool execute();
    bool undo();
    bool redo();

    bool refresh();

private:
    bool       in;
    Connector* c;

    Command::Ptr    delete_connections;

    std::string c_uuid;

};
}
}
#endif // COMMAND_DELETE_CONNECTOR_H
