#ifndef SET_MAX_EXECUTION_FREQUENCY_H
#define SET_MAX_EXECUTION_FREQUENCY_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/model/execution_mode.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT SetMaximumExecutionFrequency : public Command
{
    SetMaximumExecutionFrequency(const AUUID &graph_uuid, const UUID& node, double frequency);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
    double was_frequency;
    double frequency;
};

}

}

#endif // SET_MAX_EXECUTION_FREQUENCY_H
