/// HEADER
#include <utils_param/parameter_description.h>

using namespace param;

ParameterDescription::ParameterDescription(const std::string &description)
    : description_(description)
{

}
ParameterDescription::ParameterDescription()
    : description_("")
{

}

std::string ParameterDescription::toString() const
{
    return description_;
}
