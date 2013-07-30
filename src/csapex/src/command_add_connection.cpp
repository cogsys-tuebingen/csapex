/// HEADER
#include <csapex/command_add_connection.h>

/// COMPONENT
#include <csapex/command.h>
#include <csapex/selector_proxy.h>
#include <csapex/box.h>
#include <csapex/box_manager.h>

/// SYSTEM
#include <boost/foreach.hpp>

using namespace csapex;
using namespace csapex::command;

AddConnection::AddConnection(Connector* a, Connector* b)
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

void AddConnection::execute()
{
    if(from->tryConnect(to)) {
        BoxManager::instance().setDirty(true);
    }
}

bool AddConnection::undo()
{
    refresh();
    from->removeConnection(to);
    BoxManager::instance().setDirty(true);

    return true;
}
void AddConnection::redo()
{
    refresh();
    execute();
}

void AddConnection::refresh()
{
    from = BoxManager::instance().findConnectorOwner(from_uuid)->getOutput(from_uuid);
    to = BoxManager::instance().findConnectorOwner(to_uuid)->getInput(to_uuid);

    assert(from);
    assert(to);
}
