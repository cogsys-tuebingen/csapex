#ifndef DELETE_SIGNAL_CONNECTION_H
#define DELETE_SIGNAL_CONNECTION_H

/// COMPONENT
#include <csapex/command/delete_connection.h>
#include <csapex/utility/uuid.h>
#include <csapex/signal/signal_fwd.h>

namespace csapex
{

namespace command
{

class DeleteSignalConnection : public DeleteConnection
{
public:
    DeleteSignalConnection(const AUUID &graph_uuid, Event* a, Slot* b);

    virtual bool doUndo() override;
};
}
}
#endif // DELETE_SIGNAL_CONNECTION_H

