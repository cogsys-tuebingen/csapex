/// HEADER
#include <csapex/model/connection_information.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/serialization/serialization_buffer.h>

using namespace csapex;

ConnectionInformation::ConnectionInformation(const UUID& from, const UUID& to,
                                             const TokenDataConstPtr& type,
                                             int id, bool active,
                                             const std::vector<Fulcrum> &fulcrums)
    : from(from), to(to), from_label(""), to_label(""), type(type),
      id(id), active(active),
      fulcrums(fulcrums)
{
}

ConnectionInformation::ConnectionInformation()
{
}

std::shared_ptr<Clonable> ConnectionInformation::makeEmptyClone() const
{
    return std::shared_ptr<Clonable>( new ConnectionInformation );
}

void ConnectionInformation::serialize(SerializationBuffer &data) const
{
    data << from;
    data << to;
    data << from_label;
    data << to_label;
    data << type;
    data << id;
    data << active;
    data << fulcrums;
}
void ConnectionInformation::deserialize(const SerializationBuffer& data)
{
    data >> from;
    data >> to;
    data >> from_label;
    data >> to_label;
    data >> type;
    data >> id;
    data >> active;
    data >> fulcrums;
}
