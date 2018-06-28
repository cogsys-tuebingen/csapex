/// HEADER
#include <csapex/command/add_variadic_connector_and_connect.h>

/// COMPONENT
#include <csapex/model/node_handle.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/signal/event.h>
#include <csapex/signal/slot.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node.h>
#include <csapex/command/dispatcher.h>
#include <csapex/command/command_factory.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

using namespace csapex;
using namespace command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(AddVariadicConnectorAndConnect)

AddVariadicConnectorAndConnect::AddVariadicConnectorAndConnect(const AUUID& graph_id, const AUUID& node,
                                                               const ConnectorType& connector_type,
                                                               const TokenDataConstPtr& type, const std::string &label,
                                                               const UUID& target, bool move,
                                                               bool external)
    : AddVariadicConnector(graph_id, node, connector_type, type, label),
      target_(target), move_(move), external_(external)
{
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



void AddVariadicConnectorAndConnect::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    AddVariadicConnector::serialize(data, version);

    data << target_;
    data << move_;
    data << external_;

    data << additional_work_;
}

void AddVariadicConnectorAndConnect::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    AddVariadicConnector::deserialize(data, version);

    data >> target_;
    data >> move_;
    data >> external_;

    data >> additional_work_;
}
