/// HEADER
#include <csapex/model/boxed_object_constructor.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace csapex;

const BoxedObjectConstructor::Ptr BoxedObjectConstructor::NullPtr
    (new BoxedObjectConstructor("void", "", boost::bind(&BoxedObjectConstructor::makeNull)));

BoxedObject::Ptr BoxedObjectConstructor::makeNull()
{
    return BoxedObject::Ptr (new NullBoxedObject);
}

BoxedObjectConstructor::BoxedObjectConstructor(const std::string &type, const std::string &description, Make c)
    : type_(type), descr_(description), is_loaded(false), c(c)
{
}

BoxedObjectConstructor::BoxedObjectConstructor(const std::string &type, const std::string &description)
    : type_(type), descr_(description)
{
}

BoxedObjectConstructor::~BoxedObjectConstructor()
{
}


std::string BoxedObjectConstructor::getType() const
{
    return type_;
}

void BoxedObjectConstructor::load() const
{
    BoxedObject::Ptr prototype = c();

    icon = prototype->getIcon();
    cat = prototype->getTags();

    is_loaded = true;
}

std::vector<Tag> BoxedObjectConstructor::getTags() const
{
    if(!is_loaded) {
        load();
    }
    return cat;
}

QIcon BoxedObjectConstructor::getIcon() const
{
    return icon;
}

std::string BoxedObjectConstructor::getDescription() const
{
    return descr_;
}

BoxedObject::Ptr BoxedObjectConstructor::makePrototypeContent() const
{
    return c();
}

BoxedObject::Ptr BoxedObjectConstructor::makeContent() const
{
    return c();
}