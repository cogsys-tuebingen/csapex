/// HEADER
#include "parameter_map.h"

using namespace vision;

ParameterMap::ParameterMap()
{
}

Parameter& ParameterMap::operator [] (const std::string &name)
{
    try {
        return map_.at(name);
    } catch(const std::out_of_range& e) {
        map_[name] = Parameter(name);
        return operator [](name);
    }

    throw std::logic_error("programming error, this line should not be reached");
}

Parameter& ParameterMap::at(const std::string &name)
{
    return map_.at(name);
}

const Parameter& ParameterMap::at(const std::string &name) const
{
    return map_.at(name);
}
