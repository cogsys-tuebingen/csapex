/// HEADER
#include <csapex/model/template_constructor.h>

/// COMPONENT
#include <csapex/model/boxed_object.h>
#include <csapex/manager/template_manager.h>

using namespace csapex;

TemplateConstructor::TemplateConstructor(bool temporary, const std::string &type, const std::string &description)
    : BoxedObjectConstructor(type, description)
{
    std::string name = type.substr(std::string("::template::").size());

    Template::Ptr t = TemplateManager::instance().get(name);

    icon = t->icon;

    if(temporary) {
        cat.push_back(Tag::get("Temporary"));
    } else {
        cat.push_back(Tag::get("General"));
        cat.push_back(Tag::get("Template"));
    }

    cat.insert(cat.end(), t->tags.begin(), t->tags.end());
}

BoxedObjectPtr TemplateConstructor::makeContent(const std::string&) const
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
