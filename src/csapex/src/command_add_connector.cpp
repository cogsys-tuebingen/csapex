/// HEADER
#include <csapex/command_add_connector.h>

/// COMPONENT
#include <csapex/box.h>
#include <csapex/connector_in.h>
#include <csapex/connector_out.h>
#include <csapex/connector_forward.h>
//#include <csapex/connector_in_forward.h>
//#include <csapex/connector_out_forward.h>
#include <csapex/graph.h>
#include <csapex/command_dispatcher.h>

using namespace csapex;
using namespace command;

AddConnector::AddConnector(Box* box, bool input, const std::string &uuid, bool forward)
    : box(box), input(input), c(NULL), c_uuid(uuid), forward(forward)
{
    graph = box->getGraph();
    b_uuid = box->UUID();
}

bool AddConnector::execute()
{
    if(input) {
        std::string uuid = c_uuid.empty() ? Connector::makeUUID(box->UUID(), forward ? Connector::TYPE_MISC : Connector::TYPE_IN, box->nextInputId()) : c_uuid;
        ConnectorIn* in;
        if(forward) {
            in = new ConnectorForward(box, true, uuid);
        } else {
            in = new ConnectorIn(box, uuid);
        }
        c = in;
        box->addInput(in);
    } else {
        std::string uuid = c_uuid.empty() ? Connector::makeUUID(box->UUID(), forward ? Connector::TYPE_MISC : Connector::TYPE_OUT, box->nextOutputId()) : c_uuid;
        ConnectorOut* out;
        if(forward) {
            out = new ConnectorForward(box, false, uuid);
        } else {
            out = new ConnectorOut(box, uuid);
        }
        c = out;
        box->addOutput(out);
    }

    c_uuid = c->UUID();

    return true;
}

bool AddConnector::undo()
{
    box = graph->findBox(b_uuid);
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
    box = graph->findBox(b_uuid);

    assert(box);

    execute();
    return false;
}
