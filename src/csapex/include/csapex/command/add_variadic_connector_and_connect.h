#ifndef ADD_VARIADIC_CONNECTOR_AND_CONNECT_H
#define ADD_VARIADIC_CONNECTOR_AND_CONNECT_H

/// COMPONENT
#include "add_variadic_connector.h"
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/subgraph_node.h>

namespace csapex
{

namespace command
{

class CSAPEX_COMMAND_EXPORT AddVariadicConnectorAndConnect : public AddVariadicConnector
{
    COMMAND_HEADER(AddVariadicConnectorAndConnect);

public:
    AddVariadicConnectorAndConnect(const AUUID &graph_id, const AUUID &node, const ConnectorType &connector_type, const TokenDataConstPtr& type, const std::string& label,
                                   const UUID &target, bool move, bool external);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getDescription() const override;

private:
    UUID target_;
    bool move_;
    bool external_;

    CommandPtr additional_work_;
};
}
}

#endif // ADD_VARIADIC_CONNECTOR_AND_CONNECT_H
