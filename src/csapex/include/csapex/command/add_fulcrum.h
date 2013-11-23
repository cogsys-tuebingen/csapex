#ifndef COMMAND_ADD_FULCRUM_H
#define COMMAND_ADD_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QPoint>

namespace csapex
{
namespace command
{

struct AddFulcrum : public Command
{
    AddFulcrum(int connection_id, int sub_section_to_split, const QPoint& pos, int type);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

private:
    int connection_id;
    int sub_section_to_split;
    const QPoint pos;
    int type;
};

}

}

#endif // COMMAND_ADD_FULCRUM_H
