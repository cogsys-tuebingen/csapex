/// HEADER
#include <csapex/model/tag.h>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;

Tag::Manager::Manager()
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
