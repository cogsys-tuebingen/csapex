#ifndef COMMAND_DELETE_CONNECTION_H
#define COMMAND_DELETE_CONNECTION_H

/// COMPONENT
#include "meta.h"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

class DeleteConnection : public Meta
{
protected:
    DeleteConnection(Connectable* a, Connectable* b);

protected:
    bool doExecute();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    int connection_id;

    UUID from_uuid;
    UUID to_uuid;
};
}
}

#endif // COMMAND_DELETE_CONNECTION_H
