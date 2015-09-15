/// HEADER
#include <csapex/param/parameter_map.h>

/// COMPONENT
#include <csapex/param/range_parameter.h>

using namespace csapex;
using namespace param;

ParameterMap::ParameterMap()
{
}

Parameter& ParameterMap::operator [] (const std::string &name)
{
    std::map<std::string, Parameter::Ptr>::iterator it = map_.find(name);
    if(it != map_.end()) {
        assert(it->second->name() == name);
        return *it->second;
    } else {
        map_.insert(std::make_pair(name, Parameter::Ptr(new RangeParameter(name, ParameterDescription()))));
        return operator [](name);
    }

    throw std::logic_error("programming error, this line should not be reached");
}

Parameter::Ptr ParameterMap::at(const std::string &name)
{
    std::map<std::string, Parameter::Ptr>::iterator it = map_.find(name);
    if(it != map_.end()) {
        return it->second;
    }

    throw std::out_of_range(std::string("parameter ") + name + " not found");
}

const Parameter::Ptr ParameterMap::at(const std::string &name) const
{
    std::map<std::string, Parameter::Ptr>::const_iterator it = map_.find(name);
    if(it != map_.end()) {
        return it->second;
    }

    throw std::out_of_range(std::string("parameter ") + name + " not found");
}
