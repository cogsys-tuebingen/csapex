#ifndef RENAME_CONNECTOR_H
#define RENAME_CONNECTOR_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT RenameConnector : public Command
{
    RenameConnector(const AUUID &graph_uuid, const UUID& connector, const std::string &new_name);

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
#endif // RENAME_CONNECTOR_H
