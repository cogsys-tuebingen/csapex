#ifndef NODE_FACADE_H
#define NODE_FACADE_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/utility/notifier.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/profiling/profiling_fwd.h>
#include <csapex/model/activity_type.h>
#include <csapex/model/execution_state.h>
#include <csapex/model/connector_description.h>
#include <csapex/param/param_fwd.h>

namespace csapex
{

class CSAPEX_EXPORT NodeFacade : public Observer, public Notifier
{
protected:
    NodeFacade();

public:
    ~NodeFacade();

    virtual std::string getType() const = 0;
    virtual UUID getUUID() const = 0;
    virtual AUUID getAUUID() const = 0;

    virtual bool isActive() const = 0;
    virtual void setActive(bool active) = 0;

    virtual bool isProcessingEnabled() const = 0;

    virtual bool isGraph() const = 0;
    virtual AUUID getSubgraphAUUID() const = 0;

    virtual bool isSource() const = 0;
    virtual bool isSink() const = 0;
    virtual bool isProcessingNothingMessages() const = 0;

    virtual bool isParameterInput(const UUID& id) = 0;
    virtual bool isParameterOutput(const UUID& id) = 0;

    virtual bool isVariadic() const = 0;
    virtual bool hasVariadicInputs() const = 0;
    virtual bool hasVariadicOutputs() const = 0;
    virtual bool hasVariadicEvents() const = 0;
    virtual bool hasVariadicSlots() const = 0;

    virtual std::vector<ConnectorDescription> getInputs() const = 0;
    virtual std::vector<ConnectorDescription> getOutputs() const = 0;
    virtual std::vector<ConnectorDescription> getEvents() const = 0;
    virtual std::vector<ConnectorDescription> getSlots() const = 0;

    virtual std::vector<ConnectorDescription> getInternalInputs() const = 0;
    virtual std::vector<ConnectorDescription> getInternalOutputs() const = 0;
    virtual std::vector<ConnectorDescription> getInternalEvents() const = 0;
    virtual std::vector<ConnectorDescription> getInternalSlots() const = 0;

    virtual std::vector<ConnectorDescription> getExternalInputs() const = 0;
    virtual std::vector<ConnectorDescription> getExternalOutputs() const = 0;
    virtual std::vector<ConnectorDescription> getExternalEvents() const = 0;
    virtual std::vector<ConnectorDescription> getExternalSlots() const = 0;

    virtual NodeCharacteristics getNodeCharacteristics() const = 0;

    virtual bool canStartStepping() const = 0;

    virtual bool isProfiling() const = 0;
    virtual void setProfiling(bool profiling) = 0;

    virtual bool isError() const = 0;
    virtual ErrorState::ErrorLevel errorLevel() const = 0;
    virtual std::string errorMessage() const = 0;

    virtual ExecutionState getExecutionState() const = 0;

    virtual std::string getLabel() const = 0;

    virtual double getExecutionFrequency() const = 0;
    virtual double getMaximumFrequency() const = 0;

    // Parameterizable
    virtual std::vector<param::ParameterPtr> getParameters() const = 0;
    virtual param::ParameterPtr getParameter(const std::string& name) const = 0;
    virtual bool hasParameter(const std::string& name) const = 0;

    // Debug Access
    virtual std::string getDebugDescription() const = 0;
    virtual std::string getLoggerOutput(ErrorState::ErrorLevel level) const = 0;

    // TODO: proxies
    virtual ProfilerPtr getProfiler() = 0;

    virtual NodeStatePtr getNodeState() const = 0;
    virtual NodeStatePtr getNodeStateCopy() const = 0;
    virtual void setNodeState(NodeStatePtr memento) = 0;

    virtual GenericStateConstPtr getParameterState() const = 0;



    template <typename T>
    T readParameter(const std::string& name) const;
    template <typename T>
    void setParameter(const std::string& name, const T& value);

    // TODO: remove or add proxies for all of them
    virtual NodeHandlePtr getNodeHandle() const = 0;
    virtual NodeWorkerPtr getNodeWorker() const = 0;
    virtual NodeRunnerPtr getNodeRunner() const = 0;

public:
    slim_signal::Signal<void(NodeFacade* facade)> start_profiling;
    slim_signal::Signal<void(NodeFacade* facade)> stop_profiling;


    slim_signal::Signal<void (ConnectorPtr)> connector_created;
    slim_signal::Signal<void (ConnectorPtr)> connector_removed;

    slim_signal::Signal<void (ConnectorPtr, ConnectorPtr)> connection_in_prograss;
    slim_signal::Signal<void (ConnectorPtr)> connection_done;
    slim_signal::Signal<void (ConnectorPtr)> connection_start;

    slim_signal::Signal<void()> messages_processed;

    slim_signal::Signal<void()> node_state_changed;
    slim_signal::Signal<void()> parameters_changed;

    slim_signal::Signal<void()> destroyed;

    slim_signal::Signal<void(NodeFacade* facade, ActivityType type, std::shared_ptr<const Interval> stamp)> interval_start;
    slim_signal::Signal<void(NodeFacade* facade, std::shared_ptr<const Interval> stamp)> interval_end;
};

}

#endif // NODE_FACADE_H
