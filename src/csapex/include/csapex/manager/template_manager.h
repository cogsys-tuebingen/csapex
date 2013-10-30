#ifndef TEMPLATE_MANAGER_H
#define TEMPLATE_MANAGER_H

/// COMPONENT
#include <csapex/model/template.h>

/// PROJECT
#include <utils_plugin/singleton.hpp>

namespace csapex
{

class TemplateManager : public Singleton<TemplateManager>
{
    friend class Singleton<TemplateManager>;
    friend class GraphIO;

private:
    TemplateManager();

public:
    static std::string defaultTemplatePath();

    Template::Ptr createNewTemporaryTemplate();
    Template::Ptr createNewNamedTemplate(const std::string& name);
    Template::Ptr get(const std::string& name);

    void reset();

    void load(const std::string& path);
    bool save(const std::string& path, Template::Ptr templ);
    bool templateExists(const std::string& name);

private:
    int next_id;
    std::vector<Template::Ptr> temporary_templates;
    std::map<std::string, Template::Ptr> named_templates;
};

}

#endif // TEMPLATE_MANAGER_H
