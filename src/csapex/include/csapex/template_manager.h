#ifndef TEMPLATE_MANAGER_H
#define TEMPLATE_MANAGER_H

/// COMPONENT
#include <csapex/sub_graph_template.h>

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
    SubGraphTemplate::Ptr createNewTemporaryTemplate();
    SubGraphTemplate::Ptr get(const std::string& name);

private:
    int next_id;
    std::vector<SubGraphTemplate::Ptr> temporary_templates;
};

}

#endif // TEMPLATE_MANAGER_H
