#ifndef FLIP_SIDES_H
#define FLIP_SIDES_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct FlipSides : public Command
{
    FlipSides(const UUID &parent_uuid, const UUID& node);

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

