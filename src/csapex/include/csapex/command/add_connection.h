#ifndef COMMAND_ADD_CONNECTION_HPP
#define COMMAND_ADD_CONNECTION_HPP

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{


namespace command
{

struct AddConnection : public Command
{
    AddConnection(const UUID &from_uuid, const UUID &to_uuid);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    void refresh();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

private:
    ConnectorOut* from;
    ConnectorIn* to;

    UUID from_uuid;
    UUID to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
