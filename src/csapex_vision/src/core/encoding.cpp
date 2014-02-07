/// HEADER
#include <csapex_vision/encoding.h>

/// SYSTEM
#include <sstream>

using namespace csapex;

Encoding::Encoding()
{}

std::string Encoding::toString() const
{
    std::stringstream s;

    std::string separator = "";
    s << "[";
    for(std::vector<Channel>::const_iterator it = begin(); it != end(); ++it) {
        const Channel& c = *it;
        s << separator << c.name;
        separator = ", ";
    }
    s << "]";

    return s.str();
}
