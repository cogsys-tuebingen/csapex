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

PassOutConnector::PassOutConnector(const AUUID& graph_id, const std::string& connector_type,
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
    return std::string("create forwarding connector with type ") + connector_type;
}


bool PassOutConnector::doExecute()
{
    Graph* graph = getGraph();

    if(connector_type == "in") {
        map = graph->addForwardingInput(token_type, "forwarding", false);
    } else {
        map = graph->addForwardingOutput(token_type, "forwarding");
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
