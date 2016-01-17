/// HEADER
#include <csapex/command/pass_out_connector.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>

using namespace csapex;
using namespace command;

PassOutConnector::PassOutConnector(const UUID& graph_id, const UUID& connector_id)
    : g_uuid(graph_id), c_uuid(connector_id)
{
}

std::string PassOutConnector::getType() const
{
    return "PassOutConnector";
}

std::string PassOutConnector::getDescription() const
{
    return std::string("deleted connector with UUID ") + c_uuid.getFullName();
}


bool PassOutConnector::doExecute()
{
    NodeHandle* node_handle = getGraph()->findNodeHandle(g_uuid);
    apex_assert_hard(node_handle);

    NodePtr sub_graph_node = node_handle->getNode().lock();
    apex_assert_hard(sub_graph_node);

    GraphPtr sub_graph = std::dynamic_pointer_cast<Graph>(sub_graph_node);
    apex_assert_hard(sub_graph);

    if(c_uuid.type() == "in") {
        UUID relay_input = sub_graph->passOutInput(c_uuid);
    } else {
        UUID relay_output = sub_graph->passOutOutput(c_uuid);
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


