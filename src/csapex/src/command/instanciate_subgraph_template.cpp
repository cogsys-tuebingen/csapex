/// HEADER
#include <csapex/command/instanciate_subgraph_template.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/command/add_box.h>
#include <csapex/model/box.h>
#include <csapex/model/box_group.h>
#include <csapex/model/graph.h>
#include <csapex/manager/template_manager.h>

using namespace csapex::command;

InstanciateTemplate::InstanciateTemplate(const std::string &templ, const std::string &parent_uuid, const QPoint &pos)
    : templ(templ), parent(parent_uuid), pos(pos)
{
    add(command::AddBox::Ptr(new command::AddBox("::meta", pos, "", parent)));
    TemplateManager::instance().get(templ)->createCommands(this, parent);
}

bool InstanciateTemplate::execute()
{
    bool r = Meta::execute();

    Box::Ptr box = Graph::root()->findBox(parent);
    assert(box);

    BoxGroup::Ptr group = boost::dynamic_pointer_cast<BoxGroup> (box);
    assert(group);

    group->setTemplateName(templ);

    return r;
}
