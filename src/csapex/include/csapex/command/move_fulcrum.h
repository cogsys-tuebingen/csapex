#ifndef COMMAND_MOVE_FULCRUM_H
#define COMMAND_MOVE_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{

class MoveFulcrum : public Command
{
public:
    MoveFulcrum(int connection_id, int fulcrum_id, const Point& from, const Point& to);

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

private:
    int connection_id;
    int fulcrum_id;
    Point from;
    Point to;
};

}

}

#endif // COMMAND_MOVE_FULCRUM_H
