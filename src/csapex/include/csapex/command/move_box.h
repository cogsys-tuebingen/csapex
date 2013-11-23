#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{

namespace command
{
class MoveBox : public Command
{
public:
    MoveBox(Box* box, QPoint to);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    QPoint from;
    QPoint to;

    std::string uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
