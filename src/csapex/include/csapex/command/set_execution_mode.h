#ifndef SET_EXECUTION_MODE_H
#define SET_EXECUTION_MODE_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/model/execution_mode.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT SetExecutionMode : public Command
{
    SetExecutionMode(const AUUID &graph_uuid, const UUID& node, ExecutionMode mode);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    ExecutionMode was_mode;
    ExecutionMode mode;
};

}

}
#endif // SET_EXECUTION_MODE_H

