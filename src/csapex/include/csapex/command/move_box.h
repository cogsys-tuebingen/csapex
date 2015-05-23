#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/data/point.h>

namespace csapex
{

namespace command
{
class MoveBox : public Command
{
public:
    MoveBox(const UUID& node_uuid, Point from, Point to, WidgetController &widget_controller);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    WidgetController& widget_controller_;

    Point from;
    Point to;

    UUID uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
