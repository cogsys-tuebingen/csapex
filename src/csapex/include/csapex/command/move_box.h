#ifndef COMMAND_MOVE_BOX_H
#define COMMAND_MOVE_BOX_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QPointF>

namespace csapex
{

namespace command
{
class MoveBox : public Command
{
public:
    MoveBox(const UUID& node_uuid, QPointF from, QPointF to, WidgetController &widget_controller);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

protected:
    WidgetController& widget_controller_;

    QPointF from;
    QPointF to;

    UUID uuid;
};

}
}

#endif // COMMAND_MOVE_BOX_H
