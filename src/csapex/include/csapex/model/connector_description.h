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
    AUUID target;
    ConnectorType connector_type;
    std::string label;
    bool optional;

    TokenDataConstPtr type;

    ConnectorDescription(const AUUID& target, ConnectorType connector_type, const TokenDataConstPtr& type, const std::string& label, bool optional = true)
        : target(target), connector_type(connector_type), label(label), optional(optional),
          type(type)
    {
    }
    ConnectorDescription(const AUUID& target, ConnectorType connector_type, const std::string& label, bool optional = true)
        : target(target), connector_type(connector_type), label(label), optional(optional)
    {
        type = connection_types::makeEmpty<connection_types::AnyMessage>();
    }
};

}

#endif // CONNECTOR_DESCRIPTION_H
