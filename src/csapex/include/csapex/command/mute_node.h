#ifndef MUTE_NODE_H
#define MUTE_NODE_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT MuteNode : public Command
{
    MuteNode(const AUUID &graph_uuid, const UUID& node, bool muted);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    bool muted;
    bool executed;
};

}

}
#endif // MUTE_NODE_H

