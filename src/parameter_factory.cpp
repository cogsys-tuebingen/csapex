/// HEADER
#include <utils_param/parameter_factory.h>

/// COMPONENT
#include <utils_param/range_parameter.h>

using namespace param;

Parameter::Ptr ParameterFactory::makeEmpty(const std::string &type)
{
    if(type == "range") {
        return Parameter::Ptr(new RangeParameter);
    } else if(type == "value") {
        return Parameter::Ptr(new ValueParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + type);
    }
}
