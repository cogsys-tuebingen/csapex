/// HEADER
#include <csapex/command/set_isolated_execution.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_state.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(SetIsolatedExecution)

SetIsolatedExecution::SetIsolatedExecution(const AUUID& parent_uuid, const UUID &node, ExecutionType type)
    : CommandImplementation(parent_uuid), uuid(node), type_(type)
{
}

std::string SetIsolatedExecution::getDescription() const
{
    std::stringstream ss;
    ss << "switch processing of " << uuid << " to ";
    if(type_ == ExecutionType::DIRECT) {
        ss << "not ";
    }
    ss << "isolated";

    return ss.str();
}

bool SetIsolatedExecution::doExecute()
{
    NodeFacadeImplementationPtr nf = std::dynamic_pointer_cast<NodeFacadeImplementation>(getGraph()->findNodeFacade(uuid));
    apex_assert_hard(nf);

    NodeStatePtr state = nf->getNodeState();
    was_type_ = state->getExecutionType();

    state->setExecutionType(type_);

    return true;
}

bool SetIsolatedExecution::doUndo()
{
    NodeFacadeImplementationPtr nf = std::dynamic_pointer_cast<NodeFacadeImplementation>(getGraph()->findNodeFacade(uuid));
    apex_assert_hard(nf);

    NodeStatePtr state = nf->getNodeState();
    state->setExecutionType(was_type_);

    return true;
}

bool SetIsolatedExecution::doRedo()
{
    return doExecute();
}



void SetIsolatedExecution::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << uuid;
    data << type_;
    data << was_type_;
}

void SetIsolatedExecution::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> uuid;
    data >> type_;
    data >> was_type_;
}
