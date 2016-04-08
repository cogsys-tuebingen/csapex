/// HEADER
#include <csapex/command/pass_out_connector.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>

using namespace csapex;
using namespace command;

PassOutConnector::PassOutConnector(const AUUID& graph_id, const ConnectorType& connector_type,
                                   const ConnectionTypeConstPtr& type)
    : Command(graph_id), connector_type(connector_type), token_type(type)
{
}

std::string PassOutConnector::getType() const
{
    return "PassOutConnector";
}

std::string PassOutConnector::getDescription() const
{
    return std::string("create forwarding connector with type ") + port_type::name(connector_type);
}


bool PassOutConnector::doExecute()
{
    Graph* graph = getGraph();

    switch(connector_type) {
    case ConnectorType::INPUT:
        map = graph->addForwardingInput(token_type, "forwarding", false);
        break;
    case ConnectorType::OUTPUT:
        map = graph->addForwardingOutput(token_type, "forwarding");
        break;
    case ConnectorType::SLOT_T:
        map = graph->addForwardingSlot("forwarding");
        break;
    case ConnectorType::TRIGGER:
        map = graph->addForwardingTrigger("forwarding");
        break;
    default:
        throw std::logic_error(std::string("unknown connector type: ") + port_type::name(connector_type));
    }

    return true;
}

bool PassOutConnector::doUndo()
{
    return false;
}

bool PassOutConnector::doRedo()
{
    return doExecute();
}

std::pair<UUID, UUID> PassOutConnector::getMap() const
{
    return map;
}
