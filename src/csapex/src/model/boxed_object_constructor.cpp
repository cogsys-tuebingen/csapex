/// HEADER
#include <csapex/model/boxed_object_constructor.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>

using namespace csapex;

const BoxedObjectConstructor::Ptr BoxedObjectConstructor::NullPtr;

BoxedObjectConstructor::BoxedObjectConstructor(const std::string &type, const std::string &description, Make c)
    : type_(type), descr_(description), c(c)
{
    BoxedObject::Ptr prototype = c();

    icon = prototype->getIcon();
    cat = prototype->getTags();
}

BoxedObjectConstructor::~BoxedObjectConstructor()
{
}


std::string BoxedObjectConstructor::getType() const
{
    return type_;
}

std::vector<Tag> BoxedObjectConstructor::getTags() const
{
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

BoxedObject::Ptr BoxedObjectConstructor::makeContent() const
{
    return c();
}
