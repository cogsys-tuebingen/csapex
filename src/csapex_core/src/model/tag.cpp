/// HEADER
#include <csapex/model/tag.h>

/// COMPONENT
#include <csapex/utility/assert.h>
#include <csapex/serialization/packet_serializer.h>
#include <csapex/serialization/io/std_io.h>

using namespace csapex;

CREATE_DEFAULT_SERIALIZER(Tag);

Tag::Manager::Manager()
{

}

Tag::Tag()
{

}

Tag::Tag(const std::string &name)
    : name_(name)
{

}

Tag::~Tag()
{

}

const Tag::Ptr Tag::get(const std::string &name)
{
    return Manager::instance().get(name);
}

bool Tag::exists(const std::string &name)
{
    return Manager::instance().exists(name);
}

std::string Tag::getName() const
{
    return name_;
}

const Tag::Ptr Tag::Manager::get(const std::string &name)
{
    std::map<std::string, Tag::Ptr>::const_iterator it = tags_.find(name);

    if(it == tags_.end()) {
        create(name);
        return get(name);

    } else {
        return it->second;
    }
}

bool Tag::Manager::exists(const std::string &name) const
{
    return tags_.find(name) != tags_.end();
}

void Tag::create(const std::string &name)
{
    Manager::instance().create(name);
}

void Tag::createIfNotExists(const std::string &name)
{
    if(!Manager::instance().exists(name)) {
        Manager::instance().create(name);
    }
}

void Tag::Manager::create(const std::string &name)
{
    apex_assert_hard(!exists(name));
    tags_.insert(std::make_pair(name, Tag::Ptr(new Tag(name))));
}

int Tag::compare(const Tag &tag) const
{
    return name_.compare(tag.getName());
}

bool Tag::operator < (const Tag& tag) const
{
    return name_ < tag.getName();
}


uint8_t Tag::getPacketType() const
{
    return PACKET_TYPE_ID;
}


void Tag::serialize(SerializationBuffer &data) const
{
    data << name_;
}
void Tag::deserialize(const SerializationBuffer& data)
{
    data >> name_;
}

std::shared_ptr<Clonable> Tag::makeEmptyClone() const
{
    return makeEmpty();
}

Tag::Ptr Tag::makeEmpty()
{
    return Ptr{new Tag};
}
