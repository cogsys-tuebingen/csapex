#ifndef COMMAND_ADD_CONNECTION_FORWARDING_HPP
#define COMMAND_ADD_CONNECTION_FORWARDING_HPP

/// COMPONENT
#include "command.h"

namespace csapex
{

class Connector;
class ConnectorIn;
class ConnectorOut;
class ConnectorForward;
class Box;

namespace command
{

struct AddConnectionForwarding : public Command
{
    AddConnectionForwarding(Box* parent, const std::string& from_uuid, const std::string& to_uuid);

protected:
    bool execute();
    bool undo();
    bool redo();

    void refresh();

private:
    bool in;

    union {
        Connector* from;
        ConnectorForward* from_in;
        ConnectorOut* from_out;
    };

    union {
        Connector* to;
        ConnectorIn* to_in;
        ConnectorForward* to_out;
    };

    Graph* graph;

    std::string from_uuid;
    std::string to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_FORWARDING_HPP
