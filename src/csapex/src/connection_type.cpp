/// HEADER
#include <csapex/connection_type.h>

ConnectionType::Ptr ConnectionType::default_;

ConnectionType::ConnectionType(const std::string& name)
    : name_(name)
{
}

ConnectionType::~ConnectionType()
{
}

bool ConnectionType::canConnectTo(ConnectionType::Ptr other_side)
{
    return other_side->acceptsConnectionFrom(this);
}

bool ConnectionType::acceptsConnectionFrom(ConnectionType *other_side)
{
    return name_ == other_side->name();
}

std::string ConnectionType::name()
{
    return name_;
}

ConnectionType::Ptr ConnectionType::makeDefault()
{
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
