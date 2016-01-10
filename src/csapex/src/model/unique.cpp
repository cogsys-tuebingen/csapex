/// HEADER
#include <csapex/model/unique.h>

using namespace csapex;

Unique::Unique(const UUID &uuid)
    : uuid_(uuid)
{

}

Unique::~Unique()
{
    uuid_.free();
}

void Unique::setUUID(const UUID &uuid)
{
    uuid_ = uuid;
}

UUID Unique::getUUID() const
{
    return uuid_;
}
