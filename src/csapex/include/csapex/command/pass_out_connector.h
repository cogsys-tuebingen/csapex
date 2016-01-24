#ifndef PASS_OUT_CONNECTOR_H
#define PASS_OUT_CONNECTOR_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

class PassOutConnector : public Command
{
public:
    PassOutConnector(const AUUID &graph_id, const std::string &connector_type, const ConnectionTypeConstPtr& type);
    std::pair<UUID, UUID> getMap() const;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;


private:
    std::string connector_type;
    ConnectionTypeConstPtr token_type;
    std::pair<UUID, UUID> map;
};
}
}

#endif // PASS_OUT_CONNECTOR_H
