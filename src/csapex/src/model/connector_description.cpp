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
      is_variadic(false),
      valid(false)
{}

ConnectorDescription::ConnectorDescription(const AUUID& owner,
                                           ConnectorType connector_type,
                                           const TokenDataConstPtr& token_type,
                                           const std::string& label)
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
                                           const std::string& label)
    : owner(owner),
      connector_type(connector_type),
      label(label),
      optional(false),
      is_parameter(false),
      is_variadic(false),
      token_type(connection_types::makeEmpty<connection_types::AnyMessage>()),
      id(UUID::NONE),
      valid(true)
{
}

ConnectorDescription::ConnectorDescription(const AUUID& owner,
                                           const UUID& uuid,
                                           ConnectorType connector_type,
                                           const TokenDataConstPtr& token_type,
                                           const std::string& label)
    : owner(owner),
      connector_type(connector_type),
      label(label),
      optional(optional),
      is_parameter(false),
      is_variadic(false),
      token_type(false),
      id(uuid),
      valid(true)
{
}


ConnectorDescription& ConnectorDescription::setOptional(bool optional)
{
    this->optional = optional;
    return *this;
}
ConnectorDescription& ConnectorDescription::setParameter(bool parameter)
{
    is_parameter = parameter;
    return *this;
}
ConnectorDescription& ConnectorDescription::setVariadic(bool variadic)
{
    is_variadic = variadic;
    return *this;
}

bool ConnectorDescription::isOutput() const
{
    return connector_type == ConnectorType::OUTPUT || connector_type == ConnectorType::EVENT;
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
    data << is_variadic;

    data << token_type;

    data << id;

    data << targets;
    data << valid;
}
void ConnectorDescription::deserialize(const SerializationBuffer& data)
{
    data >> owner;
    data >> connector_type;
    data >> label;
    data >> optional;
    data >> is_parameter;
    data >> is_variadic;

    data >> token_type;

    data >> id;

    data >> targets;
    data >> valid;
}



void ConnectorDescription::Target::serialize(SerializationBuffer &data) const
{
    data << auuid;
    data << active;
}

void ConnectorDescription::Target::deserialize(const SerializationBuffer& data)
{
    data >> auuid;
    data >> active;
}

std::shared_ptr<Clonable> ConnectorDescription::Target::makeEmptyClone() const
{
    return std::make_shared<Target>();
}

AUUID ConnectorDescription::getAUUID() const
{
    return AUUID(UUIDProvider::makeDerivedUUID_forced(owner, id.id()));
}
