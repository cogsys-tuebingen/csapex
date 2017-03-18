/// HEADER
#include <csapex/command/add_variadic_connector_and_connect.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph_facade.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>

using namespace csapex;
using namespace command;

AddVariadicConnectorAndConnect::AddVariadicConnectorAndConnect(const AUUID& graph_id, const AUUID& node,
                                                               const ConnectorType& connector_type,
                                                               const TokenDataConstPtr& type, const std::string &label,
                                                               const UUID& target, bool move,
                                                               bool external)
    : AddVariadicConnector(graph_id, node, connector_type, type, label),
      target_(target), move_(move), external_(external)
{
}

std::string AddVariadicConnectorAndConnect::getType() const
{
    return "AddVariadicConnectorAndConnect";
}

std::string AddVariadicConnectorAndConnect::getDescription() const
{
    return AddVariadicConnector::getDescription() + " and connect it";
}


bool AddVariadicConnectorAndConnect::doExecute()
{
    if(AddVariadicConnector::doExecute()) {
        RelayMapping ports = getMap();
        CommandFactory factory(getGraphFacade());

        if(move_) {
            additional_work_ = factory.moveConnections(target_, external_ ? ports.external : ports.internal);
        } else {
            additional_work_ = factory.addConnection(external_ ? ports.external : ports.internal, target_, false);
        }
        return executeCommand(additional_work_);
    }

    return false;
}

bool AddVariadicConnectorAndConnect::doUndo()
{
    if(undoCommand(additional_work_)) {
        return AddVariadicConnector::doUndo();
    }

    return false;
}

bool AddVariadicConnectorAndConnect::doRedo()
{
    return doExecute();
}
