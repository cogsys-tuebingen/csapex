/// HEADER
#include <csapex/model/template_constructor.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>

using namespace csapex;

TemplateConstructor::TemplateConstructor(bool temporary, const std::string &type, const std::string &description)
    : BoxedObjectConstructor(type, description)
{
    icon = QIcon(":/group.png");
    if(temporary) {
        cat.push_back(Tag::get("Temporary"));
    } else {
        cat.push_back(Tag::get("General"));
        cat.push_back(Tag::get("Template"));
    }
}

BoxedObjectPtr TemplateConstructor::makeContent() const
{
    return BoxedObjectPtr(new NullBoxedObject("void"));
}

BoxedObjectPtr TemplateConstructor::makePrototypeContent() const
{
    return BoxedObjectPtr(new NullBoxedObject("void"));
}

void TemplateConstructor::load() const
{

}
