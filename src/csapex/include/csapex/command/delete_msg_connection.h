#ifndef DELETE_MSG_CONNECTION_H
#define DELETE_MSG_CONNECTION_H

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/utility/uuid.h>
#include <csapex/msg/msg_fwd.h>

namespace csapex
{

namespace command
{

class CSAPEX_COMMAND_EXPORT DeleteMessageConnection : public DeleteConnection
{
public:
    DeleteMessageConnection(const AUUID& graph_uuid, Output* a, Input* b);

    virtual bool doUndo() override;
};
}
}

#endif // DELETE_MSG_CONNECTION_H

