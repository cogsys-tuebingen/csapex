#ifndef COMMAND_MOVE_CONNECTION_H
#define COMMAND_MOVE_CONNECTION_H

/// COMPONENT
#include <csapex/command/meta.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

class MoveConnection : public Meta
{
public:
    MoveConnection(const AUUID& graph_uuid, Connectable* a, Connectable* b);

private:
    UUID from_uuid;
    UUID to_uuid;
};
}
}

#endif // COMMAND_MOVE_CONNECTION_H
