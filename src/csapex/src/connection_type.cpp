/// HEADER
#include <csapex/connection_type.h>

/// COMPONENT
#include <csapex/message.h>

using namespace csapex;

ConnectionType::Ptr ConnectionType::default_;

ConnectionType::ConnectionType(const std::string& name)
    : name_(name)
{
}

ConnectionType::~ConnectionType()
{
}

ConnectionType::Ptr ConnectionType::getDefaultConnectionType()
{
    return default_->clone();
}

bool ConnectionType::canConnectTo(ConnectionType::ConstPtr other_side) const
{
    return other_side->acceptsConnectionFrom(this);
}

bool ConnectionType::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    return name_ == other_side->name();
}

std::string ConnectionType::name() const
{
    return name_;
}

ConnectionType::Ptr ConnectionType::makeDefault()
{
    if(!default_) {
        ConnectionType::default_ = connection_types::AnyMessage::make();
    }

    ConnectionType::Ptr res = default_->clone();
    assert(res);
    return res;
}

void ConnectionType::write(std::ostream &out)
{
    YAML::Emitter yaml;
    yaml << YAML::Flow << YAML::BeginMap;
    yaml << YAML::Key << "type" << YAML::Value << name();

    writeYaml(yaml);

    yaml << YAML::EndMap;

    out << yaml.c_str();
}
