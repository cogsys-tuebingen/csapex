/// HEADER
#include <csapex/manager/template_manager.h>

/// COMPONENT
#include <csapex/core/graphio.h>
#include <csapex/manager/box_manager.h>
#include <csapex/model/template_constructor.h>

/// SYSTEM
#include <boost/filesystem.hpp>
#include <fstream>

using namespace csapex;

namespace bfs = boost::filesystem3;

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

    try {
        return named_templates.at(name);

    } catch(const std::exception&) {
        throw std::out_of_range(std::string("no such template: \"" + name + "\""));
    }
}

Template::Ptr TemplateManager::createNewTemporaryTemplate()
{

    std::stringstream name;
    name << "unnamed_template_#" << next_id;

    Template::Ptr res(new Template(name.str()));
    ++next_id;

    temporary_templates.push_back(res);

    TemplateConstructor::Ptr constructor(new TemplateConstructor(std::string("::template::") + res->getName(), "no description"));
    BoxManager::instance().register_box_type(constructor);

    return res;
}

void TemplateManager::load(const std::string &path)
{
    bfs::directory_iterator end; // default construction yields past-the-end
    for(bfs::directory_iterator it(path); it != end; ++it) {
        if(it->path().extension() == GraphIO::template_extension) {
            const std::string& file(it->path().string());
            std::cerr << "loading template " << file << std::endl;

            std::ifstream ifs(file.c_str());
            YAML::Parser parser(ifs);

            YAML::Node doc;
            if(!parser.GetNextDocument(doc)) {
                std::cerr << "cannot read the template" << file << std::endl;
                continue;
            }

            Template::Ptr tmp(new Template(it->path().string()));
            tmp->read(doc);

            named_templates[tmp->getName()] = tmp;

            // TODO: allow description
            TemplateConstructor::Ptr constructor(new TemplateConstructor(std::string("::template::") + tmp->getName(), "no description"));
            BoxManager::instance().register_box_type(constructor);
        }
    }
}
