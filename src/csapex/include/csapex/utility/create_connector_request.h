#ifndef CREATE_CONNECTOR_REQUEST_H
#define CREATE_CONNECTOR_REQUEST_H

/// HEADER
#include <csapex/utility/uuid.h>
#include <csapex/model/connector_type.h>
#include <csapex/model/model_fwd.h>
#include <csapex/msg/any_message.h>

namespace csapex
{

struct CreateConnectorRequest
{
    AUUID target;
    ConnectorType connector_type;
    std::string label;
    bool optional;

    TokenDataConstPtr type;

    CreateConnectorRequest(const AUUID& target, ConnectorType connector_type, const TokenDataConstPtr& type, const std::string& label, bool optional = true)
        : target(target), connector_type(connector_type), label(label), optional(optional),
          type(type)
    {
    }
    CreateConnectorRequest(const AUUID& target, ConnectorType connector_type, const std::string& label, bool optional = true)
        : target(target), connector_type(connector_type), label(label), optional(optional)
    {
        type = connection_types::makeEmpty<connection_types::AnyMessage>();
    }
};

}

#endif // CREATE_CONNECTOR_REQUEST_H
