#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

namespace csapex
{

namespace command
{
class CSAPEX_COMMAND_EXPORT MoveBox : public Command
{
public:
    MoveBox(const AUUID &graph_uuid, const UUID& node_uuid, Point from, Point to);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    Point from;
    Point to;

    UUID box_uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
