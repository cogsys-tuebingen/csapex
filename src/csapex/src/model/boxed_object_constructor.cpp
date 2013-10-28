/// HEADER
#include <csapex/model/boxed_object_constructor.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>

/// SYSTEM
#include <boost/bind.hpp>

using namespace csapex;

BoxedObject::Ptr BoxedObjectConstructor::makeNull()
{
    return BoxedObject::Ptr (new NullBoxedObject("void"));
}

BoxedObjectConstructor::BoxedObjectConstructor(const std::string &type, const std::string &description, Make c)
    : type_(type), descr_(description), is_loaded(false), c(c)
{
    assert(!c.empty());
}

BoxedObjectConstructor::BoxedObjectConstructor(const std::string &type, const std::string &description)
    : type_(type), descr_(description), is_loaded(false)
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
    try {
        BoxedObject::Ptr prototype = c();

        icon = prototype->getIcon();
        cat = prototype->getTags();

        is_loaded = true;
    } catch(const std::exception& e) {
        std::cerr << "cannot load object of type '" << type_ << "': " << e.what() << std::endl;
        is_loaded = false;
    }
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
    BoxedObject::Ptr res = c();
    res->setType(type_);
    return res;
}

BoxedObject::Ptr BoxedObjectConstructor::makeContent(const std::string& uuid) const
{
    BoxedObject::Ptr res = c();
    res->setType(type_);
    res->setUUID(uuid);
    return res;
}
