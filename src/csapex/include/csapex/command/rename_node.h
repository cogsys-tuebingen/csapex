#ifndef RENAME_NODE_H
#define RENAME_NODE_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT RenameNode : public Command
{
    RenameNode(const AUUID &graph_uuid, const UUID& node, const std::string &new_name);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    std::string new_name_;
    std::string old_name_;
};

}

}
#endif // RENAME_NODE_H
