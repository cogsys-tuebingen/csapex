/// HEADER
#include <csapex/command_delete_box.h>

/// COMPONENT
#include <csapex/command_delete_connection.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>

using namespace csapex::command;

DeleteBox::DeleteBox(Box *box)
    : box(box), pos(box->pos())
{
    parent = box->parentWidget();
    type = box->getType();
    uuid = box->UUID();
    saved_state = box->getState();

    remove_connections = box->removeAllConnectionsCmd();
}

void DeleteBox::execute()
{
    doExecute(remove_connections);

    box->stop();
    box->deleteLater();
}

bool DeleteBox::undo()
{
    box = BoxManager::instance().makeBox(parent, pos, type, uuid);
    box->setState(saved_state);

    return doUndo(remove_connections);
}

void DeleteBox::redo()
{
    refresh();

    doRedo(remove_connections);

    box->stop();
    box->deleteLater();
}

void DeleteBox::refresh()
{
    box = BoxManager::instance().findBox(uuid);
}
