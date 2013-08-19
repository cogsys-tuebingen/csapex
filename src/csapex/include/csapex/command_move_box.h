#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command.h"

/// SYSTEM
#include <QPoint>

namespace csapex
{

class Box;

namespace command
{
class MoveBox : public Command
{
public:
    MoveBox(Box* box, QPoint from, QPoint to);

protected:
    bool execute();
    bool undo();
    bool redo();

protected:
    Box* box;

    QPoint from;
    QPoint to;

    std::string uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
