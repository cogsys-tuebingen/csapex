/// HEADER
#include <csapex/model/connector_description.h>

/// PROJECT
#include <csapex/msg/any_message.h>
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;


ConnectorDescription::ConnectorDescription()
    : connector_type(ConnectorType::NONE),
      optional(false),
      is_parameter(false),
      valid(false)
{}

ConnectorDescription::ConnectorDescription(const AUUID& owner,
                     ConnectorType connector_type,
                     const TokenDataConstPtr& token_type,
                     const std::string& label,
                     bool optional,
                     bool is_parameter)
    : owner(owner),
      connector_type(connector_type),
      label(label),
      optional(optional),
      is_parameter(is_parameter),
      token_type(token_type),
      id(UUID::NONE),
      valid(true)
{
}

ConnectorDescription::ConnectorDescription(const AUUID& owner,
                     ConnectorType connector_type,
                     const std::string& label,
                     bool optional,
                     bool is_parameter)
    : owner(owner),
      connector_type(connector_type),
      label(label),
      optional(optional),
      is_parameter(is_parameter),
      token_type(connection_types::makeEmpty<connection_types::AnyMessage>()),
      id(UUID::NONE),
      valid(true)
{
}

ConnectorDescription::ConnectorDescription(const AUUID& owner,
                     const UUID& uuid,
                     ConnectorType connector_type,
                     const TokenDataConstPtr& token_type,
                     const std::string& label,
                     bool optional,
                     bool is_parameter)
    : owner(owner),
      connector_type(connector_type),
      label(label),
      optional(optional),
      is_parameter(is_parameter),
      token_type(token_type),
      id(uuid),
      valid(true)
{
}

std::shared_ptr<Clonable> ConnectorDescription::makeEmptyClone() const
{
    return std::make_shared<ConnectorDescription>();
}

void ConnectorDescription::serialize(SerializationBuffer &data) const
{
    data << owner;
    data << connector_type;
    data << label;
    data << optional;
    data << is_parameter;

    data << token_type;

    data << id;
}
void ConnectorDescription::deserialize(const SerializationBuffer& data)
{
    data >> owner;
    data >> connector_type;
    data >> label;
    data >> optional;
    data >> is_parameter;

    data >> token_type;

    data >> id;
}
