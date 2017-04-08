#ifndef NODE_FACADE_H
#define NODE_FACADE_H

/// COMPONENT
#include <csapex/model/model_fwd.h>
#include <csapex/model/observer.h>
#include <csapex/utility/uuid.h>
#include <csapex/utility/slim_signal.hpp>

namespace csapex
{

class NodeFacade : public Observer
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

    NodeWorkerPtr getNodeWorker();
    NodeHandlePtr getNodeHandle();

public:
    slim_signal::Signal<void(NodeFacade* facade)> start_profiling;
    slim_signal::Signal<void(NodeFacade* facade)> stop_profiling;

private:
    NodeHandlePtr nh_;
    NodeWorkerPtr nw_;
};

}

#endif // NODE_FACADE_H
