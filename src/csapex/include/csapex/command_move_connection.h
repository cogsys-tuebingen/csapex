#ifndef COMMAND_MOVE_CONNECTION_H
#define COMMAND_MOVE_CONNECTION_H

/// COMPONENT
#include "command.h"
#include "command_meta.h"

namespace csapex
{

class Connector;
class ConnectorIn;
class ConnectorOut;

namespace command
{

struct MoveConnection : public Meta
{
    MoveConnection(Connector* a, Connector* b);

protected:
    void makeCommand();

private:
    bool output;

    Connector* from;
    Connector* to;

    std::string from_uuid;
    std::string to_uuid;
};
}
}

#endif // COMMAND_MOVE_CONNECTION_H
