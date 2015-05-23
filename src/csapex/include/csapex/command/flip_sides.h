#ifndef FLIP_SIDES_H
#define FLIP_SIDES_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct FlipSides : public Command
{
    FlipSides(const UUID& node);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    UUID uuid;
};

}

}
#endif // FLIP_SIDES_H

