/// HEADER
#include <csapex/param/parameter_description.h>

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
    if(description_.empty()) {
        return "<i>no description</i>";
    } else {
        return description_;
    }
}
