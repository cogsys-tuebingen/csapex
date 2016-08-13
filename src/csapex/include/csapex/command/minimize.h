#ifndef MINIMIZE_H
#define MINIMIZE_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT Minimize : public Command
{
    Minimize(const AUUID &graph_uuid, const UUID& node, bool mini);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    bool mini;
    bool executed;
};

}

}
#endif // MINIMIZE_H

