/// HEADER
#include <csapex/model/unique.h>

using namespace csapex;

Unique::Unique(const std::string &uuid)
    : uuid_(uuid)
{

}

Unique::~Unique()
{

}

void Unique::setUUID(const std::string &uuid)
{
    uuid_ = UUID(uuid);
}

UUID Unique::getUUID() const
{
    return uuid_;
}
