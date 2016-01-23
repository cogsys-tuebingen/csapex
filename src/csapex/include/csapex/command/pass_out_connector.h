#ifndef PASS_OUT_CONNECTOR_H
#define PASS_OUT_CONNECTOR_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>

namespace csapex
{

namespace command
{

struct PassOutConnector : public Command
{
    PassOutConnector(const UUID &graph_id, const UUID& connector_id);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;

private:
    UUID c_uuid;
};
}
}

#endif // PASS_OUT_CONNECTOR_H
