#ifndef CONNECTOR_DESCRIPTION_H
#define CONNECTOR_DESCRIPTION_H

/// HEADER
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/any_message.h>

namespace csapex
{

struct ConnectorDescription
{
    AUUID owner;
    ConnectorType connector_type;
    std::string label;
    bool optional;

    TokenDataConstPtr token_type;

    std::vector<AUUID> targets;

    ConnectorDescription(const AUUID& owner, ConnectorType connector_type, const TokenDataConstPtr& token_type, const std::string& label, bool optional = true)
        : owner(owner), connector_type(connector_type), label(label), optional(optional),
          token_type(token_type)
    {
    }
    ConnectorDescription(const AUUID& owner, ConnectorType connector_type, const std::string& label, bool optional = true)
        : owner(owner), connector_type(connector_type), label(label), optional(optional)
    {
        token_type = connection_types::makeEmpty<connection_types::AnyMessage>();
    }
};

}

#endif // CONNECTOR_DESCRIPTION_H
