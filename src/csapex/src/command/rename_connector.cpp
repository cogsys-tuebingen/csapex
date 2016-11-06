/// HEADER
#include <csapex/command/rename_connector.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/connectable.h>

/// SYSTEM
#include <sstream>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex;
using namespace csapex::command;

RenameConnector::RenameConnector(const AUUID& parent_uuid, const UUID &connector, const std::string& new_name)
    : Command(parent_uuid), uuid(connector), new_name_(new_name)
{
}

std::string RenameConnector::getType() const
{
    return "RenameConnector";
}

std::string RenameConnector::getDescription() const
{
    std::stringstream ss;
    ss << "rename connector " << uuid << " from " << old_name_ << " to " << new_name_;
    return ss.str();
}

bool RenameConnector::doExecute()
{
    ConnectablePtr connector = getGraph()->findConnector(uuid);
    apex_assert_hard(connector);

    old_name_ = connector->getLabel();
    connector->setLabel(new_name_);

    return true;
}

bool RenameConnector::doUndo()
{
    ConnectablePtr connector = getGraph()->findConnector(uuid);
    apex_assert_hard(connector);

    connector->setLabel(old_name_);

    return true;
}

bool RenameConnector::doRedo()
{
    return doExecute();
}

