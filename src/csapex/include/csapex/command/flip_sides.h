#ifndef FLIP_SIDES_H
#define FLIP_SIDES_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct CSAPEX_COMMAND_EXPORT FlipSides : public Command
{
    FlipSides(const AUUID &graph_uuid, const UUID& node);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    UUID uuid;
};

}

}
#endif // FLIP_SIDES_H

