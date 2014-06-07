#ifndef COMMAND_DELETE_CONNECTION_H
#define COMMAND_DELETE_CONNECTION_H

/// COMPONENT
#include "meta.h"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

struct DeleteConnection : public Meta {
    DeleteConnection(Connectable* a, Connectable* b);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    bool refresh();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

private:
    int connection_id;

    ConnectorOut* from;
    ConnectorIn* to;

    UUID from_uuid;
    UUID to_uuid;
};
}
}

#endif // COMMAND_DELETE_CONNECTION_H
