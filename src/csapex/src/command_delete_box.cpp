/// HEADER
#include <csapex/command_delete_box.h>

/// COMPONENT
#include <csapex/command_delete_connection.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>
#include <csapex/graph.h>
#include <csapex/template_manager.h>
#include <csapex/command_instanciate_subgraph_template.h>

using namespace csapex::command;

DeleteBox::DeleteBox(const std::string &uuid)
    : uuid(uuid)
{
    Box::Ptr box = Graph::root()->findBox(uuid);

    parent = box->parentWidget();
    type = box->getType();
}

bool DeleteBox::execute()
{
    Box::Ptr box = Graph::root()->findBox(uuid);

    pos = box->pos();
    remove_connections = box->removeAllConnectionsCmd();

    if(doExecute(remove_connections)) {
        saved_state = box->getState();

        Graph::root()->deleteBox(box->UUID());
        return true;
    }

    return false;
}

bool DeleteBox::undo()
{
    Box::Ptr box = BoxManager::instance().makeBox(pos, type, uuid);
    box->setState(saved_state);

    if(!box->state->template_.empty()) {
        SubGraphTemplate::Ptr templ = TemplateManager::instance().get(box->state->template_);
//        command::Meta::Ptr meta(new command::Meta);
//        templ->createCommands(meta.get(), uuid);
//        Command::doExecute(meta);

        doExecute(Command::Ptr(new command::InstanciateSubGraphTemplate(templ, uuid, pos)));

    } else {
        Graph::root()->addBox(box);
    }


    return doUndo(remove_connections);
}

bool DeleteBox::redo()
{
    if(doRedo(remove_connections)) {
        Box::Ptr box = Graph::root()->findBox(uuid);
        saved_state = box->getState();

        Graph::root()->deleteBox(box->UUID());
        return true;
    }

    return false;
}
