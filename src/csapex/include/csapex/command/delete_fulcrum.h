#ifndef COMMAND_DELETE_FULCRUM_H
#define COMMAND_DELETE_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{
namespace command
{

struct DeleteFulcrum : public Command
{
    DeleteFulcrum(int connection_id, int fulcrum_id);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    int connection_id;
    int fulcrum_id;
    QPoint pos;
};

}

}

#endif // COMMAND_DELETE_FULCRUM_H
