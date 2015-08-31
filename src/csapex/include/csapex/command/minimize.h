#ifndef MINIMIZE_H
#define MINIMIZE_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct Minimize : public Command
{
    Minimize(const UUID& node, bool mini);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    UUID uuid;
    bool mini;
    bool executed;
};

}

}
#endif // MINIMIZE_H

