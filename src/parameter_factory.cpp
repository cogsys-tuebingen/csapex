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
    } else if(type == "path") {
        return Parameter::Ptr(new PathParameter);
    } else if(type == "trigger") {
        return Parameter::Ptr(new TriggerParameter);
    } else {
        throw std::runtime_error(std::string("illegal parameter type: ") + type);
    }
}
