/// HEADER
#include <csapex/model/template_constructor.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>

using namespace csapex;

TemplateConstructor::TemplateConstructor(const std::string &type, const std::string &description)
    : BoxedObjectConstructor(type, description)
{
    icon = QIcon(":/group.png");
}

BoxedObjectPtr TemplateConstructor::makeContent() const
{
    return BoxedObjectPtr(new NullBoxedObject);
}

BoxedObjectPtr TemplateConstructor::makePrototypeContent() const
{
    return BoxedObjectPtr(new NullBoxedObject);
}

void TemplateConstructor::load() const
{

}
