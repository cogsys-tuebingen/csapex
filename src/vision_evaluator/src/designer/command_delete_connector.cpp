/// HEADER
#include "command_delete_connector.h"

/// COMPONENT
#include "selector_proxy.h"
#include "box.h"
#include "box_manager.h"

using namespace vision_evaluator;

DeleteConnector::DeleteConnector(Connector *_c) :
    in(_c->isInput()),
    c(_c)
{
    assert(c);
    c_uuid = c->UUID();
}

void DeleteConnector::execute()
{
    Box* box_c = BoxManager::instance().findConnectorOwner(c_uuid);

    if(c->isConnected()) {
        if(in) {
            delete_connections = ((ConnectorIn*) c)->removeAllConnectionsCmd();
        } else {
            delete_connections = ((ConnectorOut*) c)->removeAllConnectionsCmd();
        }
        BoxManager::instance().execute(delete_connections);
    }

    if(in) {
        box_c->removeInput((ConnectorIn*) c);
    } else {
        box_c->removeOutput((ConnectorOut*) c);
    }

    BoxManager::instance().setDirty(true);
}

bool DeleteConnector::undo()
{
    if(!refresh()) {
        return false;
    }

    return false;
}

void DeleteConnector::redo()
{

}

bool DeleteConnector::refresh()
{
    Box* box_c = BoxManager::instance().findConnectorOwner(c_uuid);

    if(!box_c) {
        return false;
    }

    if(in) {
        c = box_c->getInput(c_uuid);
    } else {
        c = box_c->getOutput(c_uuid);
    }

    assert(c);

    return true;
}


