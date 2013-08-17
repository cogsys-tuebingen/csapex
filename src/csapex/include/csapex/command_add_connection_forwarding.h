#ifndef COMMAND_ADD_CONNECTION_FORWARDING_HPP
#define COMMAND_ADD_CONNECTION_FORWARDING_HPP

/// COMPONENT
#include "command.h"

namespace csapex
{

class Connector;
class ConnectorIn;
class ConnectorInForward;
class ConnectorOut;
class ConnectorOutForward;
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
        ConnectorInForward* from_in;
        ConnectorOut* from_out;
    };

    union {
        Connector* to;
        ConnectorIn* to_in;
        ConnectorOutForward* to_out;
    };

    Graph* graph;

    std::string from_uuid;
    std::string to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_FORWARDING_HPP
