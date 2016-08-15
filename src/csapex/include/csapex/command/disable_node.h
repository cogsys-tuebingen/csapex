#ifndef COMMAND_DISABLE_NODE_H
#define COMMAND_DISABLE_NODE_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{
class CSAPEX_COMMAND_EXPORT DisableNode : public Command
{
public:
    DisableNode(const AUUID &graph_uuid, const UUID &uuid, bool disable = true);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    UUID uuid;
    bool disable_;
};

}
}
#endif // COMMAND_DISABLE_NODE_H
