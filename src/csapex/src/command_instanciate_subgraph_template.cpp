/// HEADER
#include <csapex/command_instanciate_subgraph_template.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/command_add_box.h>
#include <csapex/box.h>
#include <csapex/box_group.h>
#include <csapex/graph.h>
#include <csapex/template_manager.h>

using namespace csapex::command;

InstanciateSubGraphTemplate::InstanciateSubGraphTemplate(const std::string &templ, const std::string &parent_uuid, const QPoint &pos)
    : templ(templ), parent(parent_uuid), pos(pos)
{
    add(command::AddBox::Ptr(new command::AddBox("::meta", pos, "", parent)));
    TemplateManager::instance().get(templ)->createCommands(this, parent);
}

bool InstanciateSubGraphTemplate::execute()
{
    bool r = Meta::execute();

    Box::Ptr box = Graph::root()->findBox(parent);
    assert(box);

    BoxGroup::Ptr group = boost::dynamic_pointer_cast<BoxGroup> (box);
    assert(group);

    group->setTemplateName(templ);

    return r;
}
