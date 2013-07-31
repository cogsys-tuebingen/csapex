/// HEADER
#include <csapex/command_delete_connector.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/box_manager.h>

using namespace csapex;

DeleteConnector::DeleteConnector(Connector *_c) :
    in(_c->isInput()),
    c(_c)
{
    assert(c);
    c_uuid = c->UUID();
}

bool DeleteConnector::execute()
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

    return true;
}

bool DeleteConnector::undo()
{
    if(!refresh()) {
        return false;
    }

    return false;
}

bool DeleteConnector::redo()
{
    return false;
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


