/// HEADER
#include <csapex/model/node_facade.h>

/// PROJECT
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/model/node_state.h>
#include <csapex/msg/input_transition.h>
#include <csapex/msg/output_transition.h>
#include <csapex/signal/event.h>
#include <csapex/model/graph/vertex.h>
#include <csapex/model/node.h>

/// SYSTEM
#include <iostream>
#include <sstream>

using namespace csapex;

NodeFacade::NodeFacade()
{
}

NodeFacade::~NodeFacade()
{
}

std::vector<ConnectorDescription> NodeFacade::getExternalConnectors() const
{
    std::vector<ConnectorDescription> result;
    if(getAUUID() != AUUID::NONE) {
        auto insert = [&result](const std::vector<ConnectorDescription>& vec)
        {
            result.insert(result.end(), vec.begin(), vec.end());
        };

        insert(getExternalInputs());
        insert(getExternalOutputs());
        insert(getExternalSlots());
        insert(getExternalEvents());
    }

    return result;
}

std::vector<ConnectorDescription> NodeFacade::getInternalConnectors() const
{
    std::vector<ConnectorDescription> result;
    if(getAUUID() != AUUID::NONE) {
        auto insert = [&result](const std::vector<ConnectorDescription>& vec)
        {
            result.insert(result.end(), vec.begin(), vec.end());
        };

        insert(getInternalInputs());
        insert(getInternalOutputs());
        insert(getInternalSlots());
        insert(getInternalEvents());
    }

    return result;
}

template <typename T>
T NodeFacade::readParameter(const std::string& name) const
{
    param::ParameterConstPtr p = getParameter(name);
    return p->as<T>();
}

template <typename T>
void NodeFacade::setParameter(const std::string& name, const T& value)
{    
    param::ParameterPtr p = getParameter(name);
    return p->set<T>(value);
}



template CSAPEX_EXPORT bool NodeFacade::readParameter<bool>(const std::string& name) const;
template CSAPEX_EXPORT double NodeFacade::readParameter<double>(const std::string& name) const;
template CSAPEX_EXPORT int NodeFacade::readParameter<int>(const std::string& name) const;
template CSAPEX_EXPORT std::string NodeFacade::readParameter<std::string>(const std::string& name) const;
template CSAPEX_EXPORT std::pair<int,int> NodeFacade::readParameter<std::pair<int,int> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<double,double> NodeFacade::readParameter<std::pair<double,double> >(const std::string& name) const;
template CSAPEX_EXPORT std::pair<std::string, bool> NodeFacade::readParameter<std::pair<std::string, bool> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<double> NodeFacade::readParameter<std::vector<double> >(const std::string& name) const;
template CSAPEX_EXPORT std::vector<int> NodeFacade::readParameter<std::vector<int> >(const std::string& name) const;


template CSAPEX_EXPORT void NodeFacade::setParameter<bool>(const std::string& name, const bool& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<double>(const std::string& name, const double& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<int>(const std::string& name, const int& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::string>(const std::string& name, const std::string& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::pair<int,int> > (const std::string& name, const std::pair<int,int>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::pair<double,double> >(const std::string& name, const std::pair<double,double>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::pair<std::string, bool> >(const std::string& name, const std::pair<std::string, bool>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::vector<int> >(const std::string& name, const std::vector<int>& value);
template CSAPEX_EXPORT void NodeFacade::setParameter<std::vector<double> >(const std::string& name, const std::vector<double>& value);

std::vector<ConnectorDescription> NodeFacade::getInputs() const
{
    return getExternalInputs();
}

std::vector<ConnectorDescription> NodeFacade::getOutputs() const
{
    return getExternalOutputs();
}

std::vector<ConnectorDescription> NodeFacade::getEvents() const
{
    return getExternalEvents();
}

std::vector<ConnectorDescription> NodeFacade::getSlots() const
{
    return getExternalSlots();
}
