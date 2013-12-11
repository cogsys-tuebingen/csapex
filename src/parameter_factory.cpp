/// HEADER
#include <utils_param/parameter_factory.h>

using namespace param;

Parameter::Ptr ParameterFactory::makeEmpty(const std::string &type)
{
    if(type == "range") {
        return Parameter::Ptr(new RangeParameter);
    } else if(type == "value") {
        return Parameter::Ptr(new ValueParameter);
    } else if(type == "set") {
        return Parameter::Ptr(new SetParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + type);
    }
}
