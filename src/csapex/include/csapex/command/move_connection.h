#ifndef COMMAND_MOVE_CONNECTION_H
#define COMMAND_MOVE_CONNECTION_H

/// COMPONENT
#include <csapex/command/meta.h>

namespace csapex
{

namespace command
{

struct MoveConnection : public Meta
{
    MoveConnection(Connector* a, Connector* b);

    virtual std::string getType() const;

private:
    bool output;

    std::string from_uuid;
    std::string to_uuid;
};
}
}

#endif // COMMAND_MOVE_CONNECTION_H
