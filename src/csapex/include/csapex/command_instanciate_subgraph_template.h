#ifndef COMMAND_INSTANCIATE_SUBGRAPH_TEMPLATE_H
#define COMMAND_INSTANCIATE_SUBGRAPH_TEMPLATE_H

/// COMPONENT
#include <csapex/command.h>
#include <csapex/memento.h>
#include <csapex/command_meta.h>
#include <csapex/selector_proxy.h>
#include <csapex/csapex_fwd.h>

/// SYSTEM
#include <QWidget>

namespace csapex
{

namespace command
{

struct InstanciateSubGraphTemplate : public Meta
{
    InstanciateSubGraphTemplate(const std::string& templ, const std::string& parent_uuid, const QPoint& pos);

    bool execute();

private:
    std::string templ;
    std::string parent;
    QPoint pos;
};
}
}

#endif // COMMAND_INSTANCIATE_SUBGRAPH_TEMPLATE_H
