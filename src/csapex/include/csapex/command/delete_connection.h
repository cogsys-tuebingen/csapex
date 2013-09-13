#ifndef COMMAND_DELETE_CONNECTION_H
#define COMMAND_DELETE_CONNECTION_H

/// COMPONENT
#include "command.h"

namespace csapex
{

namespace command
{

struct DeleteConnection : public Command {
    DeleteConnection(Connector* a, Connector* b);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    bool refresh();

private:
    Command::Ptr remove_fulcrums;

    int connection_id;

    ConnectorOut* from;
    ConnectorIn* to;

    std::string from_uuid;
    std::string to_uuid;
};
}
}

#endif // COMMAND_DELETE_CONNECTION_H
