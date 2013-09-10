/// HEADER
#include <csapex/template_manager.h>

using namespace csapex;

TemplateManager::TemplateManager()
    : next_id(0)
{

}

SubGraphTemplate::Ptr TemplateManager::get(const std::string &name)
{
    foreach(const SubGraphTemplate::Ptr& templ, temporary_templates) {
        if(templ->name_ == name) {
            return templ;
        }
    }

    throw std::out_of_range(std::string("no such template: " + name));
}

SubGraphTemplate::Ptr TemplateManager::createNewTemporaryTemplate()
{

    std::stringstream name;
    name << "unnamed template #" << next_id;

    SubGraphTemplate::Ptr res(new SubGraphTemplate(name.str()));
    ++next_id;

    temporary_templates.push_back(res);

    return res;
}
