/// HEADER
#include <csapex/command/update_parameter.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
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

    } else if(value.type() == typeid(std::vector<int>)) {
        auto v =  boost::any_cast<std::vector<int>> (value);
        for(const auto& e : v) {
            ss << e << " ";
        }

    } else if(value.type() == typeid(std::string)) {
        ss << boost::any_cast<std::string> (value);

    } else if(value.type() == typeid(std::pair<int, int>)) {
        auto p = boost::any_cast<std::pair<int, int>> (value);
        ss << "[" << p.first << ", " << p.second << "]";

    } else if(value.type() == typeid(std::pair<double, double>)) {
        auto p = boost::any_cast<std::pair<double, double>> (value);
        ss << "[" << p.first << ", " << p.second << "]";

    } else if(value.type() == typeid(std::pair<std::string, bool>)) {
        auto p = boost::any_cast<std::pair<std::string, bool>> (value);
        ss << "[" << p.first << ": " << p.second << "]";

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
        node->setParameterLater(uuid.name(), boost::any_cast<int> (value));

    } else if(value.type() == typeid(double)) {
        node->setParameterLater(uuid.name(), boost::any_cast<double> (value));

    } else if(value.type() == typeid(bool)) {
        node->setParameterLater(uuid.name(), boost::any_cast<bool> (value));

    } else if(value.type() == typeid(std::string)) {
        node->setParameterLater(uuid.name(), boost::any_cast<std::string> (value));

    } else if(value.type() == typeid(std::vector<int>)) {
        node->setParameterLater(uuid.name(), boost::any_cast<std::vector<int>> (value));

    } else if(value.type() == typeid(std::pair<int, int>)) {
        node->setParameterLater(uuid.name(), boost::any_cast<std::pair<int, int>> (value));

    } else if(value.type() == typeid(std::pair<double, double>)) {
        node->setParameterLater(uuid.name(), boost::any_cast<std::pair<double, double>> (value));

    } else if(value.type() == typeid(std::pair<std::string, bool>)) {
        node->setParameterLater(uuid.name(), boost::any_cast<std::pair<std::string, bool>> (value));

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

