/// HEADER
#include <csapex/model/multi_connection_type.h>

using namespace csapex;

namespace {
std::string toString(const std::vector<ConnectionType::Ptr>& types) {
    std::stringstream ss;
    int i = 0;
    for(std::vector<ConnectionType::Ptr>::const_iterator it = types.begin(); it != types.end(); ++it) {
        if(i++ > 0) ss << ", ";
        ss << (*it)->name();
    }
    return ss.str();
}
}

MultiConnectionType::MultiConnectionType(const std::vector<ConnectionType::Ptr>& types)
    : ConnectionType("one of {" + toString(types) + "}"), types_(types)
{

}

bool MultiConnectionType::canConnectTo(const ConnectionType *other_side) const
{
    for(std::vector<ConnectionType::Ptr>::const_iterator it = types_.begin(); it != types_.end(); ++it) {
        if((*it)->canConnectTo(other_side)) {
            return true;
        }
    }

    return false;
}

bool MultiConnectionType::acceptsConnectionFrom(const ConnectionType *other_side) const
{
    for(std::vector<ConnectionType::Ptr>::const_iterator it = types_.begin(); it != types_.end(); ++it) {
        if((*it)->acceptsConnectionFrom(other_side)) {
            return true;
        }
    }

    return false;
}

ConnectionType::Ptr MultiConnectionType::clone()
{
    Ptr new_msg(new MultiConnectionType(types_));
    return new_msg;
}

ConnectionType::Ptr MultiConnectionType::toType()
{
    Ptr new_msg(new MultiConnectionType(types_));
    return new_msg;
}

void MultiConnectionType::writeYaml(YAML::Emitter &/*yaml*/) const
{

}

void MultiConnectionType::readYaml(const YAML::Node &/*node*/)
{

}
