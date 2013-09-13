#ifndef COMMAND_MOVE_FULCRUM_H
#define COMMAND_MOVE_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{
namespace command
{

struct MoveFulcrum : public Command
{
    MoveFulcrum(int connection_id, int fulcrum_id, const QPoint& from, const QPoint& to);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    int connection_id;
    int fulcrum_id;
    QPoint from;
    QPoint to;
};

}

}

#endif // COMMAND_MOVE_FULCRUM_H
