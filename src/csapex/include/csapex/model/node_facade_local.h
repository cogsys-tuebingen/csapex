#ifndef NODE_FACADE_LOCAL_H
#define NODE_FACADE_LOCAL_H

/// PROJECT
#include <csapex/model/node_facade.h>

namespace csapex
{

class CSAPEX_EXPORT NodeFacadeLocal : public NodeFacade
{
public:
    NodeFacadeLocal(NodeHandlePtr nh, NodeWorkerPtr nw, NodeRunnerPtr nr);
    NodeFacadeLocal(NodeHandlePtr nh);
    ~NodeFacadeLocal();

    std::string getType() const override;
    UUID getUUID() const override;
    AUUID getAUUID() const override;

    bool isActive() const override;
    void setActive(bool active);

    bool isProcessingEnabled() const override;

    bool isGraph() const override;
    AUUID getSubgraphAUUID() const override;

    bool isSource() const override;
    bool isSink() const override;    
    bool isProcessingNothingMessages() const override;

    bool isParameterInput(const UUID& id) override;
    bool isParameterOutput(const UUID& id) override;

    bool isVariadic() const override;
    bool hasVariadicInputs() const override;
    bool hasVariadicOutputs() const override;
    bool hasVariadicEvents() const override;
    bool hasVariadicSlots() const override;

    std::vector<ConnectorDescription> getInternalInputs() const override;
    std::vector<ConnectorDescription> getInternalOutputs() const override;
    std::vector<ConnectorDescription> getInternalEvents() const override;
    std::vector<ConnectorDescription> getInternalSlots() const override;

    std::vector<ConnectorDescription> getExternalInputs() const override;
    std::vector<ConnectorDescription> getExternalOutputs() const override;
    std::vector<ConnectorDescription> getExternalEvents() const override;
    std::vector<ConnectorDescription> getExternalSlots() const override;

    ConnectorPtr getConnector(const UUID& id) const override;
    ConnectorPtr getConnectorNoThrow(const UUID& id) const noexcept override;

    ConnectorPtr getParameterInput(const std::string& name) const override;
    ConnectorPtr getParameterOutput(const std::string& name) const override;


    NodeCharacteristics getNodeCharacteristics() const override;

    bool canStartStepping() const override;

    bool isProfiling() const override;
    void setProfiling(bool profiling) override;

    ExecutionState getExecutionState() const override;

    std::string getLabel() const override;

    double getExecutionFrequency() const override;
    double getMaximumFrequency() const override;

    // Parameterizable    
    virtual std::vector<param::ParameterPtr> getParameters() const override;
    param::ParameterPtr getParameter(const std::string& name) const override;
    bool hasParameter(const std::string& name) const override;

//    template <typename T>
//    T readParameter(const std::string& name) const;
//    template <typename T>
//    void setParameter(const std::string& name, const T& value);

    // Debug Access
    std::string getDebugDescription() const override;
    std::string getLoggerOutput(ErrorState::ErrorLevel level) const override;

    // TODO: proxies
    ProfilerPtr getProfiler() override;

    NodeStatePtr getNodeState() const override;
    NodeStatePtr getNodeStateCopy() const override;
    void setNodeState(NodeStatePtr memento);

    GenericStateConstPtr getParameterState() const override;

    NodeHandlePtr getNodeHandle() const;
    NodeWorkerPtr getNodeWorker() const;
    NodeRunnerPtr getNodeRunner() const;
    NodePtr getNode() const;

private:
    void connectNodeHandle();
    void connectNodeWorker();

    void triggerExternalConnectorsChanged(const ConnectableConstPtr& connector);
    void triggerInternalConnectorsChanged(const ConnectableConstPtr& connector);

private:
    NodeHandlePtr nh_;
    NodeWorkerPtr nw_;
    NodeRunnerPtr nr_;
};

}

#endif // NODE_FACADE_LOCAL_H
