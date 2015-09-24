#ifndef ADD_SIGNAL_CONNECTION_H
#define ADD_SIGNAL_CONNECTION_H

/// COMPONENT
#include "command.h"
#include <csapex/model/model_fwd.h>
#include <csapex/signal/signal_fwd.h>
#include <csapex/command/add_connection.h>
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

class AddSignalConnection : public AddConnection
{
public:
    AddSignalConnection(const UUID &from_uuid, const UUID &to_uuid);

protected:
    bool doExecute();

    void refresh();

private:
    Trigger* from;
    Slot* to;
};
}
}

#endif // ADD_SIGNAL_CONNECTION_H

