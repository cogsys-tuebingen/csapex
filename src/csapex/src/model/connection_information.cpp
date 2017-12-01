/// HEADER
#include <csapex/model/connection_information.h>

/// PROJECT
#include <csapex/model/connectable.h>
#include <csapex/serialization/serialization_buffer.h>
#include <csapex/model/fulcrum.h>

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

ConnectionInformation::ConnectionInformation(const ConnectionInformation& other)
    : from(other.from), to(other.to), from_label(other.from_label), to_label(other.to_label), type(other.type),
      id(other.id), active(other.active),
      fulcrums(other.fulcrums)
{
}

ConnectionInformation::ConnectionInformation()
{
}


ConnectionInformation& ConnectionInformation::operator = (const ConnectionInformation& other)
{
    from = other.from;
    to = other.to;
    from_label = other.from_label;
    to_label = other.to_label;
    type = other.type;
    id = other.id;
    active = other.active;
    fulcrums = other.fulcrums;

    return *this;
}

bool ConnectionInformation:: operator == (const ConnectionInformation& other) const
{
    return from == other.from && to_label == other.to_label;
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
