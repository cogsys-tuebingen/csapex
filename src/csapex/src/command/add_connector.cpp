/// HEADER
#include <csapex/command/add_connector.h>

/// COMPONENT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex/model/connector_forward.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/manager/connection_type_manager.h>

using namespace csapex;
using namespace command;

AddConnector::AddConnector(const std::string &box_uuid, const std::string& label, const std::string& type, bool input, const std::string &uuid, bool forward)
    : type(type), label(label), input(input), c(NULL), b_uuid(box_uuid), c_uuid(uuid), forward(forward)
{

}

bool AddConnector::doExecute()
{
    Box::Ptr box = graph_->findBox(b_uuid);
    assert(box);

    if(input) {
        std::string uuid = c_uuid.empty() ? Connector::makeUUID(box->UUID(), forward ? Connector::TYPE_MISC : Connector::TYPE_IN, box->nextInputId()) : c_uuid;
        ConnectorIn* in;
        if(forward) {
            in = new ConnectorForward(box.get(), true, uuid);
        } else {
            in = new ConnectorIn(box.get(), uuid);
        }
        c = in;
        box->getNode()->registerInput(in);
    } else {
        std::string uuid = c_uuid.empty() ? Connector::makeUUID(box->UUID(), forward ? Connector::TYPE_MISC : Connector::TYPE_OUT, box->nextOutputId()) : c_uuid;
        ConnectorOut* out;
        if(forward) {
            out = new ConnectorForward(box.get(), false, uuid);
        } else {
            out = new ConnectorOut(box.get(), uuid);
        }
        c = out;
        box->getNode()->registerOutput(out);
    }

    c->setType(ConnectionTypeManager::createMessage(type));
    c->setLabel(label);
    c_uuid = c->UUID();

    return true;
}

bool AddConnector::doUndo()
{
    Box::Ptr box = graph_->findBox(b_uuid);
    assert(box);

    if(input) {
        box->getNode()->removeInput(box->getNode()->getInput(c_uuid));
    } else {
        box->getNode()->removeOutput(box->getNode()->getOutput(c_uuid));
    }
    return false;
}

bool AddConnector::doRedo()
{
    return doExecute();
}
