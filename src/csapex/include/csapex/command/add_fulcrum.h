#ifndef COMMAND_ADD_FULCRUM_H
#define COMMAND_ADD_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/data/point.h>
#include <string>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT AddFulcrum : public Command
{
public:
    AddFulcrum(const AUUID& graph_uuid, int connection_id, int sub_section_to_split, const Point& pos, int type);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

private:
    int connection_id;
    int sub_section_to_split;
    Point pos;
    int type;
};

}

}

#endif // COMMAND_ADD_FULCRUM_H
