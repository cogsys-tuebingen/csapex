#ifndef add_variadic_connector_H
#define add_variadic_connector_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/graph.h>

namespace csapex
{

namespace command
{

class AddVariadicConnector : public Command
{
public:
    AddVariadicConnector(const AUUID &graph_id, const ConnectorType &connector_type, const ConnectionTypeConstPtr& type);
    RelayMapping getMap() const;

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;


private:
    ConnectorType connector_type;
    ConnectionTypeConstPtr token_type;
    RelayMapping map;
};
}
}

#endif // add_variadic_connector_H
