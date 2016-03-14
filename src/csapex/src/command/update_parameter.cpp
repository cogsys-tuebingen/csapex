/// HEADER
#include <csapex/command/update_parameter.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade.h>

/// SYSTEM
#include <sstream>
#include <iostream>
#include <typeindex>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;


bool UpdateParameter::isUndoable() const
{
    return false;
}

std::string UpdateParameter::getType() const
{
    return "UpdateParameter";
}

std::string UpdateParameter::getDescription() const
{
    std::stringstream ss;
    ss << "set parameter " << uuid << " to ";

    if(value.type() == typeid(int)) {
        ss << boost::any_cast<int> (value);

    } else if(value.type() == typeid(double)) {
        ss << boost::any_cast<double> (value);

    } else if(value.type() == typeid(bool)) {
        ss << boost::any_cast<bool> (value);

    } else if(value.type() == typeid(std::string)) {
        ss << boost::any_cast<std::string> (value);

    } else {
        throw std::runtime_error(std::string("unsupported type: ") + value.type().name());
    }

    return ss.str();
}

bool UpdateParameter::doExecute()
{
    UUID node_uuid = uuid.parentUUID();

    NodeHandle* node_handle = getRoot()->getGraph()->findNodeHandle(node_uuid);
    apex_assert_hard(node_handle);


    NodePtr node = node_handle->getNode().lock();
    apex_assert_hard(node);


    if(value.type() == typeid(int)) {
        node->setParameterLater(uuid.name(), boost::any_cast<int> (value), true);

    } else if(value.type() == typeid(double)) {
        node->setParameterLater(uuid.name(), boost::any_cast<double> (value), true);

    } else if(value.type() == typeid(bool)) {
        node->setParameterLater(uuid.name(), boost::any_cast<bool> (value), true);

    } else if(value.type() == typeid(std::string)) {
        node->setParameterLater(uuid.name(), boost::any_cast<std::string> (value), true);

    } else {
        throw std::runtime_error(std::string("unsupported type: ") + value.type().name());
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

