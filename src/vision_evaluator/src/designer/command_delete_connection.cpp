/// HEADER
#include "command_delete_connection.h"

/// COMPONENT
#include <boost/foreach.hpp>
#include "command.h"
#include "selector_proxy.h"
#include "box.h"
#include "box_manager.h"

using namespace vision_evaluator;
using namespace vision_evaluator::command;

DeleteConnection::DeleteConnection(Connector* a, Connector* b)
{
    from = dynamic_cast<ConnectorOut*>(a);
    if(from) {
        to = dynamic_cast<ConnectorIn*>(b);
    } else {
        from = dynamic_cast<ConnectorOut*>(b);
        to = dynamic_cast<ConnectorIn*>(a);
    }
    assert(from);
    assert(to);

    from_uuid = from->UUID();
    to_uuid = to->UUID();
}

void DeleteConnection::execute()
{
    from->removeConnection(to);
    BoxManager::instance().setDirty(true);
}

bool DeleteConnection::undo()
{
    if(!refresh()) {
        return false;
    }

    if(from->tryConnect(to)) {
        BoxManager::instance().setDirty(true);
        return true;
    }

    return false;
}
void DeleteConnection::redo()
{
    refresh();
    execute();
}

bool DeleteConnection::refresh()
{
    Box* from_box = BoxManager::instance().findConnectorOwner(from_uuid);
    Box* to_box = BoxManager::instance().findConnectorOwner(to_uuid);

    if(!from_box || !to_box) {
        return false;
    }

    from = from_box->getOutput(from_uuid);
    to = to_box->getInput(to_uuid);

    assert(from);
    assert(to);

    return true;
}
