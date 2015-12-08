#ifndef COMMAND_DELETE_FULCRUM_H
#define COMMAND_DELETE_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{

class DeleteFulcrum : public Command
{
public:
    DeleteFulcrum(int connection_id, int fulcrum_id);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

private:
    int connection_id;
    int fulcrum_id;
    Point pos;
    Point in;
    Point out;
    int type;
};

}

}

#endif // COMMAND_DELETE_FULCRUM_H
