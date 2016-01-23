#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>
#include <csapex/view/designer/designer.h>

namespace csapex
{

namespace command
{
class MoveBox : public Command
{
public:
    MoveBox(const UUID &graph_uuid, const UUID& node_uuid, Point from, Point to, Designer *designer);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    Designer* view_;

    Point from;
    Point to;

    UUID box_uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
