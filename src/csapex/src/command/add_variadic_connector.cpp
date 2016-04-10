/// HEADER
#include <csapex/command/add_variadic_connector.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/trigger.h>
#include <csapex/signal/slot.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>

using namespace csapex;
using namespace command;

AddVariadicConnector::AddVariadicConnector(const AUUID& graph_id, const ConnectorType& connector_type,
                                           const ConnectionTypeConstPtr& type)
    : Command(graph_id), connector_type(connector_type), token_type(type), label_("forwarding"), optional_(false)
{
}

void AddVariadicConnector::setLabel(const std::string &label)
{
    label_ = label;
}

void AddVariadicConnector::setOptional(bool optional)
{
    optional_ = optional;
}

std::string AddVariadicConnector::getType() const
{
    return "AddVariadicConnector";
}

std::string AddVariadicConnector::getDescription() const
{
    return std::string("create forwarding connector with type ") + port_type::name(connector_type);
}


bool AddVariadicConnector::doExecute()
{
    Graph* graph = getGraph();

    switch(connector_type) {
    case ConnectorType::INPUT:
    {
        Input* input = graph->createVariadicInput(token_type, label_, optional_);
        connector_id = map.external = input->getUUID();
        auto relay = graph->getRelayForInput(connector_id);
        map.internal = relay->getUUID();
    }
        break;
    case ConnectorType::OUTPUT:
    {
        Output* output = graph->createVariadicOutput(token_type, label_);
        connector_id = map.external = output->getUUID();
        auto relay = graph->getRelayForOutput(connector_id);
        map.internal = relay->getUUID();
    }
        break;
    case ConnectorType::SLOT_T:
    {
        Slot* slot = graph->createVariadicSlot(label_, [](){});
        connector_id = map.external = slot->getUUID();
        auto relay = graph->getRelayForSlot(connector_id);
        map.internal = relay->getUUID();
    }
        break;
    case ConnectorType::TRIGGER:
    {
        Trigger* trigger = graph->createVariadicTrigger(label_);
        connector_id = map.external = trigger->getUUID();
        auto relay = graph->getRelayForTrigger(connector_id);
        map.internal = relay->getUUID();
    }
        break;
    default:
        throw std::logic_error(std::string("unknown connector type: ") + port_type::name(connector_type));
    }

    return true;
}

bool AddVariadicConnector::doUndo()
{
    Graph* graph = getGraph();

    switch(connector_type) {
    case ConnectorType::INPUT:
        graph->removeVariadicInputById(connector_id);
        break;
    case ConnectorType::OUTPUT:
        graph->removeVariadicOutputById(connector_id);
        break;
    case ConnectorType::SLOT_T:
        graph->removeVariadicSlotById(connector_id);
        break;
    case ConnectorType::TRIGGER:
        graph->removeVariadicTriggerById(connector_id);
        break;
    default:
        throw std::logic_error(std::string("unknown connector type: ") + port_type::name(connector_type));
    }

    return true;
}

bool AddVariadicConnector::doRedo()
{
    return doExecute();
}

RelayMapping AddVariadicConnector::getMap() const
{
    return map;
}
