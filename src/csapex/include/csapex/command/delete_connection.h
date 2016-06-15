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
    DeleteConnection(const AUUID& graph_uuid, Connectable* a, Connectable* b);

protected:
    bool doExecute() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    int connection_id;
    bool active_;

    UUID from_uuid;
    UUID to_uuid;
};
}
}

#endif // COMMAND_DELETE_CONNECTION_H
