#ifndef MINIMIZE_H
#define MINIMIZE_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{
namespace command
{

struct Minimize : public Command
{
    Minimize(const UUID& node);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    UUID uuid;
    bool mini;
};

}

}
#endif // MINIMIZE_H

