/// HEADER
#include <csapex/command/update_parameter.h>

/// COMPONENT
#include <csapex/command/command.h>
#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_state.h>
#include <csapex/model/graph_facade_local.h>
#include <csapex/core/settings.h>
#include <csapex/core/csapex_core.h>
#include <csapex/command/command_serializer.h>
#include <csapex/serialization/serialization_buffer.h>

/// SYSTEM
#include <sstream>
#include <iostream>
#include <typeindex>

/// COMPONENT
#include <csapex/utility/assert.h>

using namespace csapex::command;

CSAPEX_REGISTER_COMMAND_SERIALIZER(UpdateParameter)

bool UpdateParameter::isUndoable() const
{
    return false;
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

    } else if(value.type() == typeid(const char*)) {
        ss << boost::any_cast<const char*> (value);

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
    if(value.type() == typeid(int)) {
        setParameter(boost::any_cast<int> (value));

    } else if(value.type() == typeid(double)) {
        setParameter(boost::any_cast<double> (value));

    } else if(value.type() == typeid(bool)) {
        setParameter(boost::any_cast<bool> (value));

    } else if(value.type() == typeid(std::string)) {
        setParameter(boost::any_cast<std::string> (value));

    } else if(value.type() == typeid(const char*)) {
        setParameter(std::string(boost::any_cast<const char*> (value)));

    } else if(value.type() == typeid(std::vector<int>)) {
        setParameter(boost::any_cast<std::vector<int>> (value));

    } else if(value.type() == typeid(std::pair<int, int>)) {
        setParameter(boost::any_cast<std::pair<int, int>> (value));

    } else if(value.type() == typeid(std::pair<double, double>)) {
        setParameter(boost::any_cast<std::pair<double, double>> (value));

    } else if(value.type() == typeid(std::pair<std::string, bool>)) {
        setParameter(boost::any_cast<std::pair<std::string, bool>> (value));

    } else if(!value.empty()) {
        throw std::runtime_error(std::string("unsupported type: ") + value.type().name());
    }

    return true;
}


template <typename T>
void UpdateParameter::setParameter(const T& value)
{
    if(uuid.global()) {
        // setting
        apex_assert_hard(!uuid.globalName().empty());
        core_->getSettings().set(uuid.globalName(), value);

    } else {
        UUID node_uuid = uuid.parentUUID();

        NodeHandle* node_handle = getRoot()->getGraph()->findNodeHandle(node_uuid);
        apex_assert_hard(node_handle);


        NodePtr node = node_handle->getNode().lock();
        apex_assert_hard(node);

        node->setParameterLater(uuid.name(), value);
    }
}

bool UpdateParameter::doUndo()
{
    return true;
}

bool UpdateParameter::doRedo()
{
    return doExecute();
}


void UpdateParameter::serialize(SerializationBuffer &data) const
{
    Command::serialize(data);

    data << uuid;
    data << value;
}

void UpdateParameter::deserialize(const SerializationBuffer& data)
{
    Command::deserialize(data);

    data >> uuid;
    data >> value;
}

