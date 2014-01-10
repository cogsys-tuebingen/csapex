/// HEADER
#include <csapex/command/add_connector.h>

/// COMPONENT

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

std::string AddConnector::getType() const
{
    return "AddConnector";
}

std::string AddConnector::getDescription() const
{
    return std::string("added a connector with UUID ") + c_uuid + " to " + b_uuid;
}


bool AddConnector::doExecute()
{
    Node* node = graph_->findNode(b_uuid);
    assert(node);

    if(input) {
        std::string uuid = c_uuid.empty() ? Connectable::makeUUID(node->getUUID(), forward ? Connectable::TYPE_MISC : Connectable::TYPE_IN, node->nextInputId()) : c_uuid;
        ConnectorIn* in;
        if(forward) {
            in = new ConnectorForward(true, uuid);
        } else {
            in = new ConnectorIn(uuid);
        }
        c = in;
        node->registerInput(in);
    } else {
        std::string uuid = c_uuid.empty() ? Connectable::makeUUID(node->getUUID(), forward ? Connectable::TYPE_MISC : Connectable::TYPE_OUT, node->nextOutputId()) : c_uuid;
        ConnectorOut* out;
        if(forward) {
            out = new ConnectorForward(false, uuid);
        } else {
            out = new ConnectorOut(uuid);
        }
        c = out;
        node->registerOutput(out);
    }

    c->setType(ConnectionTypeManager::createMessage(type));
    c->setLabel(label);
    c_uuid = c->getUUID();

    return true;
}

bool AddConnector::doUndo()
{
    Node* node = graph_->findNode(b_uuid);
    assert(node);

    if(input) {
        node->removeInput(node->getInput(c_uuid));
    } else {
        node->removeOutput(node->getOutput(c_uuid));
    }
    return false;
}

bool AddConnector::doRedo()
{
    return doExecute();
}
