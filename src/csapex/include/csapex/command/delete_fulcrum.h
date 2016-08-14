#ifndef COMMAND_DELETE_FULCRUM_H
#define COMMAND_DELETE_FULCRUM_H

/// COMPONENT
#include "command.h"
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT DeleteFulcrum : public Command
{
public:
    DeleteFulcrum(const AUUID &graph_uuid, int connection_id, int fulcrum_id);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

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
