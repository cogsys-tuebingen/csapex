#ifndef COMMAND_ADD_CONNECTION_HPP
#define COMMAND_ADD_CONNECTION_HPP

/// COMPONENT
#include "command.h"

namespace csapex
{

class Connector;
class ConnectorIn;
class ConnectorOut;

namespace command
{

struct AddConnection : public Command
{
    AddConnection(Connector* a, Connector* b);

protected:
    bool execute();
    bool undo();
    bool redo();

    void refresh();

private:
    ConnectorOut* from;
    ConnectorIn* to;

    std::string from_uuid;
    std::string to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
