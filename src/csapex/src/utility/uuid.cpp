/// HEADER
#include <csapex/utility/uuid.h>

using namespace csapex;

std::string UUID::stripNamespace(const std::string &name)
{
    size_t from = name.rfind("::");
    return name.substr(from != name.npos ? from + 2 : 0);
}

UUID::UUID(const std::string &representation)
{
    representation_ = representation;
}

UUID::operator std::string() const
{
    return representation_;
}

const char* UUID::c_str() const
{
    return representation_.c_str();
}

std::string UUID::getFullName() const
{
    return representation_;
}

std::string UUID::getShortName() const
{
    return stripNamespace(representation_);
}
