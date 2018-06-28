/// HEADER
#include <csapex/command/rename_connector.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/connectable.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/io/std_io.h>
#include <csapex/serialization/io/csapex_io.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(RenameConnector)

RenameConnector::RenameConnector(const AUUID& parent_uuid, const UUID &connector, const std::string& new_name)
    : CommandImplementation(parent_uuid), uuid(connector), new_name_(new_name)
{
}

std::string RenameConnector::getDescription() const
{
    std::stringstream ss;
    ss << "rename connector " << uuid << " from " << old_name_ << " to " << new_name_;
    return ss.str();
}

bool RenameConnector::doExecute()
{
    ConnectablePtr connector = getGraph()->findConnectable(uuid);
    apex_assert_hard(connector);

    old_name_ = connector->getLabel();
    connector->setLabel(new_name_);

    return true;
}

bool RenameConnector::doUndo()
{
    ConnectablePtr connector = getGraph()->findConnectable(uuid);
    apex_assert_hard(connector);

    connector->setLabel(old_name_);

    return true;
}

bool RenameConnector::doRedo()
{
    return doExecute();
}



void RenameConnector::serialize(SerializationBuffer &data, SemanticVersion& version) const
{
    Command::serialize(data, version);

    data << uuid;
    data << new_name_;
    data << old_name_;
}

void RenameConnector::deserialize(const SerializationBuffer& data, const SemanticVersion& version)
{
    Command::deserialize(data, version);

    data >> uuid;
    data >> new_name_;
    data >> old_name_;
}
