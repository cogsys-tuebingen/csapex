/// HEADER
#include <csapex/model/tag.h>

/// SYSTEM
#include <assert.h>
#include <iostream>

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

const Tag Tag::get(const std::string &name)
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

const Tag Tag::Manager::get(const std::string &name) const
{
    try {
        return tags_.at(name);
    } catch(const std::exception& e) {
        std::cerr << "tag doesn't exist: " << name << std::endl;
        throw e;
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
    assert(!exists(name));
    tags_.insert(std::make_pair(name, Tag(name)));
}

int Tag::compare(const Tag &tag) const
{
    return name_.compare(tag.getName());
}

bool Tag::operator < (const Tag& tag) const
{
    return name_ < tag.getName();
}
