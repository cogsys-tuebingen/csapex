/// HEADER
#include <csapex/command_add_connector.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/connector_forward.h>
#include <csapex/connection_type.h>
#include <csapex/graph.h>
#include <csapex/command_dispatcher.h>

using namespace csapex;
using namespace command;

AddConnector::AddConnector(const std::string &box_uuid, const std::string& label, ConnectionType::ConstPtr type, bool input, const std::string &uuid, bool forward)
    : type(type), label(label), input(input), c(NULL), b_uuid(box_uuid), c_uuid(uuid), forward(forward)
{

}

bool AddConnector::execute()
{
    Box::Ptr box = Graph::root()->findBox(b_uuid);
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
        box->addInput(in);
    } else {
        std::string uuid = c_uuid.empty() ? Connector::makeUUID(box->UUID(), forward ? Connector::TYPE_MISC : Connector::TYPE_OUT, box->nextOutputId()) : c_uuid;
        ConnectorOut* out;
        if(forward) {
            out = new ConnectorForward(box.get(), false, uuid);
        } else {
            out = new ConnectorOut(box.get(), uuid);
        }
        c = out;
        box->addOutput(out);
    }

    c->setType(type);
    c->setLabel(label);
    c_uuid = c->UUID();

    return true;
}

bool AddConnector::undo()
{
    Box::Ptr box = Graph::root()->findBox(b_uuid);
    assert(box);

    if(input) {
        box->removeInput(box->getInput(c_uuid));
    } else {
        box->removeOutput(box->getOutput(c_uuid));
    }
    return false;
}

bool AddConnector::redo()
{
    return execute();
}
