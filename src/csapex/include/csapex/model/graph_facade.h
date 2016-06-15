#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <csapex/utility/slim_signal.hpp>
#include <unordered_map>

namespace csapex
{

class GraphFacade
{
public:
    typedef std::shared_ptr<GraphFacade> Ptr;

public:
    GraphFacade(ThreadPool& executor, Graph* graph, NodeHandle *nh = nullptr, GraphFacade* parent = nullptr);
    ~GraphFacade();

    AUUID getAbsoluteUUID() const;
    Graph* getGraph();
    NodeHandle* getNodeHandle();
    GraphFacade* getParent() const;
    GraphFacade* getSubGraph(const UUID& uuid);
    ThreadPool* getThreadPool();

    NodeWorkerPtr getNodeWorker(const NodeHandle* handle);

    void addNode(NodeHandlePtr node);

    ConnectionPtr connect(OutputPtr output, InputPtr input);
    ConnectionPtr connect(Output* output, Input* input);

    ConnectionPtr connect(NodeHandlePtr output, const std::string& output_name,
                          NodeHandlePtr input, const std::string& inputput_name);
    ConnectionPtr connect(NodeHandle* output, const std::string& output_name,
                          NodeHandle* input, const std::string& input_name);
    ConnectionPtr connect(const UUID& output_id,
                          NodeHandle* input, const std::string& input_name);
    ConnectionPtr connect(NodeHandle* output, const std::string& output_name,
                          const UUID& input_id);
    ConnectionPtr connect(const UUID& output_id,
                          NodeHandlePtr input, const std::string& input_name);
    ConnectionPtr connect(NodeHandlePtr output, const std::string& output_name,
                          const UUID& input_id);

    ConnectionPtr connect(const UUID& output_id, const UUID& input_id);

    ConnectionPtr connect(const UUID& output_id,
                          NodeHandlePtr input, int input_id);
    ConnectionPtr connect(NodeHandlePtr output, int output_id,
                          const UUID& input_id);
    ConnectionPtr connect(NodeHandlePtr output, int output_id,
                          NodeHandlePtr input, int input_id);
    ConnectionPtr connect(NodeHandle* output, int output_id,
                          NodeHandle* input, int input_id);

    TaskGenerator* getTaskGenerator(const UUID& uuid);

    void stop();
    void clearBlock();
    void resetActivity();

    bool isPaused() const;
    void pauseRequest(bool pause);

    void clear();

public:
    csapex::slim_signal::Signal<void (bool)> paused;
    csapex::slim_signal::Signal<void ()> stopped;

    csapex::slim_signal::Signal<void(GraphFacadePtr)> childAdded;
    csapex::slim_signal::Signal<void(GraphFacadePtr)> childRemoved;

    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeAdded;
    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeRemoved;

    csapex::slim_signal::Signal<void(NodeWorkerPtr)> nodeWorkerAdded;
    csapex::slim_signal::Signal<void(NodeWorkerPtr)> nodeWorkerRemoved;

    csapex::slim_signal::Signal<void(TaskGeneratorPtr)> generatorAdded;
    csapex::slim_signal::Signal<void(TaskGeneratorPtr)> generatorRemoved;

    csapex::slim_signal::Signal<void()> panic;

private:
    void nodeAddedHandler(NodeHandlePtr node);
    void nodeRemovedHandler(NodeHandlePtr node);

private:
    GraphFacade* parent_;

    AUUID absolute_uuid_;
    Graph* graph_;
    NodeHandle* graph_handle_;
    ThreadPool& executor_;

    std::unordered_map<UUID, GraphFacadePtr, UUID::Hasher> children_;

    std::vector<csapex::slim_signal::Connection> connections_;
    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;

    std::map<const NodeHandle*, NodeWorkerPtr> node_workers_;
};

}

#endif // GRAPH_WORKER_H
