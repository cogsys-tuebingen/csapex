/// HEADER
#include <csapex/command/add_connector.h>

/// COMPONENT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/connection_type.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/msg/message_factory.h>
#include <csapex/utility/assert.h>
#include <csapex/model/node_worker.h>

using namespace csapex;
using namespace command;

AddConnector::AddConnector(const UUID &box_uuid, const std::string& label, const std::string& type, bool input, const UUID &uuid)
    : type(type), label(label), input(input), c(NULL), b_uuid(box_uuid), c_uuid(uuid)
{

}

std::string AddConnector::getType() const
{
    return "AddConnector";
}

std::string AddConnector::getDescription() const
{
    return std::string("added a connector with UUID ") + c_uuid.getFullName() + " to " + b_uuid.getFullName();
}


bool AddConnector::doExecute()
{
    NodeWorker* node_worker = graph_->findNodeWorker(b_uuid);
    apex_assert_hard(node_worker);

    if(input) {
        UUID uuid = c_uuid.empty() ? Connectable::makeUUID(node_worker->getNodeUUID(), Connectable::TYPE_IN, node_worker->getMessageInputs().size()) : c_uuid;
        Input* in = new Input(*settings_, uuid);
        c = in;
        node_worker->registerInput(in);
    } else {
        UUID uuid = c_uuid.empty() ? Connectable::makeUUID(node_worker->getNodeUUID(), Connectable::TYPE_OUT, node_worker->getMessageOutputs().size()) : c_uuid;
        Output* out = new Output(*settings_, uuid);
        c = out;
        node_worker->registerOutput(out);
    }

    c->setType(MessageFactory::createMessage(type));
    c->setLabel(label);
    c_uuid = c->getUUID();

    return true;
}

bool AddConnector::doUndo()
{
    NodeWorker* node_worker = graph_->findNodeWorker(b_uuid);
    apex_assert_hard(node_worker);

    if(input) {
        node_worker->removeInput(c_uuid);
    } else {
        node_worker->removeOutput(c_uuid);
    }
    return false;
}

bool AddConnector::doRedo()
{
    return doExecute();
}
