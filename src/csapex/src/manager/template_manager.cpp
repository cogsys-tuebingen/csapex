/// HEADER
#include <csapex/manager/template_manager.h>

using namespace csapex;

TemplateManager::TemplateManager()
    : next_id(0)
{

}

Template::Ptr TemplateManager::get(const std::string &name)
{
    foreach(const Template::Ptr& templ, temporary_templates) {
        if(templ->name_ == name) {
            return templ;
        }
    }

    throw std::out_of_range(std::string("no such template: " + name));
}

Template::Ptr TemplateManager::createNewTemporaryTemplate()
{

    std::stringstream name;
    name << "unnamed template #" << next_id;

    Template::Ptr res(new Template(name.str()));
    ++next_id;

    temporary_templates.push_back(res);

    return res;
}
