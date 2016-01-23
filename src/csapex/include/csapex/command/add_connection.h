#ifndef COMMAND_ADD_CONNECTION_HPP
#define COMMAND_ADD_CONNECTION_HPP

/// COMPONENT
#include "command.h"
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{


namespace command
{

class AddConnection : public Command
{
public:
    AddConnection(const UUID& parent_uuid, const UUID &from_uuid, const UUID &to_uuid);

protected:
    bool doUndo() override;
    bool doRedo() override;

    virtual void refresh() = 0;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    UUID from_uuid;
    UUID to_uuid;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
