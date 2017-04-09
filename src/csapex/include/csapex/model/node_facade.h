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

namespace csapex
{

class NodeFacade : public Observer, public Notifier
{
public:
    NodeFacade(NodeHandlePtr nh);
    NodeFacade(NodeHandlePtr nh, NodeWorkerPtr nw);

    ~NodeFacade();

    std::string getType() const;
    UUID getUUID() const;

    bool isActive() const;
    bool isProcessingEnabled() const;

    bool isProfiling() const;
    void setProfiling(bool profiling);

    bool isError() const;
    ErrorState::ErrorLevel errorLevel() const;
    std::string errorMessage() const;

    ExecutionState getExecutionState() const;

    NodeHandlePtr getNodeHandle();

    ProfilerPtr getProfiler();

    std::string getLabel() const;

public:
    slim_signal::Signal<void(NodeFacade* facade)> start_profiling;
    slim_signal::Signal<void(NodeFacade* facade)> stop_profiling;

    slim_signal::Signal<void()> messages_processed;

    slim_signal::Signal<void()> destroyed;

    slim_signal::Signal<void(NodeFacade* facade, ActivityType type, std::shared_ptr<const Interval> stamp)> interval_start;
    slim_signal::Signal<void(NodeFacade* facade, std::shared_ptr<const Interval> stamp)> interval_end;

private:
    NodeHandlePtr nh_;
    NodeWorkerPtr nw_;
};

}

#endif // NODE_FACADE_H
