/// HEADER
#include <csapex/model/connection_information.h>

/// PROJECT
#include <csapex/model/connectable.h>

using namespace csapex;

ConnectionInformation::ConnectionInformation(Connectable *from, Connectable *to, const TokenDataConstPtr& type, bool active)
    : from(from->getUUID()), to(to->getUUID()), from_label(from->getLabel()), to_label(to->getLabel()), type(type), active(active)
{
}
ConnectionInformation::ConnectionInformation(const UUID& from, const UUID& to, const TokenDataConstPtr& type, bool active)
    : from(from), to(to), from_label(""), to_label(""), type(type), active(active)
{
}
