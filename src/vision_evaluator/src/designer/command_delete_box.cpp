/// HEADER
#include "command_delete_box.h"

/// COMPONENT
#include "command_delete_connection.h"
#include "selector_proxy.h"
#include "box.h"
#include "box_manager.h"

using namespace vision_evaluator::command;

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
    doRedo(remove_connections);

    box->stop();
    box->deleteLater();
}
