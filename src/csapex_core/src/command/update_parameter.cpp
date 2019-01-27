/// HEADER
#include <csapex/command/update_parameter.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/core/settings.h>
#include <csapex/core/csapex_core.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>
#include <csapex/serialization/parameter_serializer.h>

/// SYSTEM
#include <sstream>
#include <iostream>
#include <typeindex>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(UpdateParameter)

UpdateParameter::UpdateParameter(const UUID& parameter_uuid, const param::Parameter& param)
  : CommandImplementation(parameter_uuid.getAbsoluteUUID()), uuid(parameter_uuid.getAbsoluteUUID()), parameter_(param.cloneAs<param::Parameter>())
{
    apex_assert_hard(!uuid.empty());
    if (parameter_uuid.global()) {
        apex_assert_hard(!parameter_uuid.globalName().empty());
    } else {
        apex_assert(!parameter_uuid.empty());
    }
}

bool UpdateParameter::isUndoable() const
{
    return false;
}

std::string UpdateParameter::getDescription() const
{
    std::stringstream ss;
    ss << "set parameter " << uuid << " to " << parameter_->toString();
    return ss.str();
}

bool UpdateParameter::doExecute()
{
    apex_assert_hard(!uuid.empty());

    if (uuid.global()) {
        // setting
        apex_assert_hard(!uuid.globalName().empty());
        core_->getSettings().get(uuid.globalName())->cloneDataFrom(*parameter_);

    } else {
        UUID node_uuid = uuid.parentUUID();

        NodeHandle* node_handle = getRoot()->getLocalGraph()->findNodeHandle(node_uuid);
        apex_assert_hard(node_handle);

        NodePtr node = node_handle->getNode().lock();
        apex_assert_hard(node);

        node->setParameterLater(uuid.name(), parameter_);
    }
    return true;
}

bool UpdateParameter::doUndo()
{
    return true;
}

bool UpdateParameter::doRedo()
{
    return doExecute();
}

void UpdateParameter::serialize(SerializationBuffer& data, SemanticVersion& version) const
{
    apex_assert_hard(!uuid.empty());

    Command::serialize(data, version);

    data << uuid;
    data << parameter_;
}

void UpdateParameter::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> uuid;
    data >> parameter_;
    
    apex_assert_hard(!uuid.empty());
}
