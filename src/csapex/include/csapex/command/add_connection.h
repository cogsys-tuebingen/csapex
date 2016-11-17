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

class CSAPEX_COMMAND_EXPORT AddConnection : public Command
{
public:
    AddConnection(const AUUID& graph_uuid, const UUID &from_uuid, const UUID &to_uuid, bool active);

protected:
    bool doUndo() override;
    bool doRedo() override;

    bool doExecute() override;
    void refresh();

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    UUID from_uuid;
    UUID to_uuid;

private:
    OutputPtr from;
    InputPtr to;

    bool active;
};
}
}
#endif // COMMAND_ADD_CONNECTION_HPP
