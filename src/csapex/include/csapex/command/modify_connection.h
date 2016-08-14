#ifndef MODIFY_CONNECTION_H
#define MODIFY_CONNECTION_H

/// COMPONENT
#include "command.h"
#include <csapex/data/point.h>

namespace csapex
{
namespace command
{

class CSAPEX_COMMAND_EXPORT ModifyConnection : public Command
{
public:
    ModifyConnection(const AUUID &graph_uuid, int connection_id, bool active);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    int connection_id;

    bool was_active;
    bool active;
};

}
}

#endif // MODIFY_CONNECTION_H
