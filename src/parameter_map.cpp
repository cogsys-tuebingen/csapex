/// HEADER
#include <utils_param/parameter_map.h>

using namespace param;

ParameterMap::ParameterMap()
{
}

Parameter& ParameterMap::operator [] (const std::string &name)
{
    std::map<std::string, Parameter>::iterator it = map_.find(name);
    if(it != map_.end()) {
        return it->second;
    } else {
        map_.insert(std::make_pair(name, Parameter(name)));
        return operator [](name);
    }

    throw std::logic_error("programming error, this line should not be reached");
}

Parameter& ParameterMap::at(const std::string &name)
{
    std::map<std::string, Parameter>::iterator it = map_.find(name);
    if(it != map_.end()) {
        return it->second;
    }

    throw std::out_of_range(std::string("parameter ") + name + " not found");
}

const Parameter& ParameterMap::at(const std::string &name) const
{
    std::map<std::string, Parameter>::const_iterator it = map_.find(name);
    if(it != map_.end()) {
        return it->second;
    }

    throw std::out_of_range(std::string("parameter ") + name + " not found");
}
