#ifndef GROUP_NODES_H
#define GROUP_NODES_H

/// COMPONENT
#include "meta.h"
#include <csapex/utility/uuid.h>

namespace csapex
{
namespace command
{

struct GroupNodes : public Meta
{
    GroupNodes(const std::vector<UUID>& nodes);

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

private:
    std::vector<UUID> uuids;
};

}

}
#endif // GROUP_NODES_H
