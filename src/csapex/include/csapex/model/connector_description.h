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
    bool is_parameter;

    TokenDataConstPtr token_type;

    UUID id;
    std::vector<AUUID> targets;

    ConnectorDescription(const AUUID& owner,
                         ConnectorType connector_type,
                         const TokenDataConstPtr& token_type,
                         const std::string& label,
                         bool optional = true,
                         bool is_parameter = false)
        : owner(owner),
          connector_type(connector_type),
          label(label),
          optional(optional),
          is_parameter(is_parameter),
          token_type(token_type)
    {
        id = UUID::NONE;
    }
    ConnectorDescription(const AUUID& owner,
                         ConnectorType connector_type,
                         const std::string& label,
                         bool optional = true,
                         bool is_parameter = false)
        : owner(owner),
          connector_type(connector_type),
          label(label),
          optional(optional),
          is_parameter(is_parameter)
    {
        token_type = connection_types::makeEmpty<connection_types::AnyMessage>();
        id = UUID::NONE;
    }

    ConnectorDescription(const AUUID& owner,
                         const UUID& uuid,
                         ConnectorType connector_type,
                         const TokenDataConstPtr& token_type,
                         const std::string& label,
                         bool optional = true,
                         bool is_parameter = false)
        : owner(owner),
          connector_type(connector_type),
          label(label),
          optional(optional),
          is_parameter(is_parameter),
          token_type(token_type),
          id(uuid)
    {
    }
};

}

#endif // CONNECTOR_DESCRIPTION_H
