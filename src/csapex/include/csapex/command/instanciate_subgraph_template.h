#ifndef COMMAND_INSTANCIATE_SUBGRAPH_TEMPLATE_H
#define COMMAND_INSTANCIATE_SUBGRAPH_TEMPLATE_H

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/memento.h>
#include <csapex/command/meta.h>
#include <csapex/model/boxed_object_constructor.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

namespace command
{

struct InstanciateTemplate : public Meta
{
    InstanciateTemplate(const std::string& templ, const std::string& parent_uuid);

    bool execute();

private:
    std::string templ;
    std::string parent;
};
}
}

#endif // COMMAND_INSTANCIATE_SUBGRAPH_TEMPLATE_H
