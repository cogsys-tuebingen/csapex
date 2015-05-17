#ifndef COMMAND_ADD_FULCRUM_H
#define COMMAND_ADD_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/csapex_fwd.h>
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{

class AddFulcrum : public Command
{
public:
    AddFulcrum(int connection_id, int sub_section_to_split, const Point& pos, int type);

protected:
    bool doExecute();
    bool doUndo();
    bool doRedo();

    virtual std::string getType() const;
    virtual std::string getDescription() const;

private:
    int connection_id;
    int sub_section_to_split;
    Point pos;
    int type;
};

}

}

#endif // COMMAND_ADD_FULCRUM_H
