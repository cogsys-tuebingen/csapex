#ifndef add_variadic_connector_H
#define add_variadic_connector_H

/// COMPONENT
#include "command.h"
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/subgraph_node.h>

namespace csapex
{

namespace command
{

class CSAPEX_COMMAND_EXPORT AddVariadicConnector : public Command
{
public:
    AddVariadicConnector(const AUUID &graph_id, const AUUID &node, const ConnectorType &connector_type, const TokenDataConstPtr& type, const std::string& label);
    RelayMapping getMap() const;

    void setLabel(const std::string& label);
    void setOptional(bool optional);

protected:
    bool doExecute() override;
    bool doUndo() override;
    bool doRedo() override;

    virtual std::string getType() const override;
    virtual std::string getDescription() const override;


private:
    ConnectorType connector_type;
    TokenDataConstPtr token_type;
    RelayMapping map;

    std::string label_;
    bool optional_;

    AUUID node_id;
    UUID connector_id;
};
}
}

#endif // add_variadic_connector_H
