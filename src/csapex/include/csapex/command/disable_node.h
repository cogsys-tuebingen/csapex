#ifndef COMMAND_DISABLE_NODE_H
#define COMMAND_DISABLE_NODE_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{
class DisableNode : public Command
{
public:
    DisableNode(const UUID &uuid, bool disable = true);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    UUID uuid;
    bool disable_;
};

}
}
#endif // COMMAND_DISABLE_NODE_H
