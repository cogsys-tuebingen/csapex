#ifndef SET_LOGGER_LEVEL_H
#define SET_LOGGER_LEVEL_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/model/execution_mode.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT SetLoggerLevel : public Command
{
    SetLoggerLevel(const AUUID &graph_uuid, const UUID& node, int level);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    int was_level;
    int level;
};

}

}
#endif // SET_LOGGER_LEVEL_H

