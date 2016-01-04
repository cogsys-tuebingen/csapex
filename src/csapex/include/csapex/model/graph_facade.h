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
    GraphFacade(Executor& executor, Graph* graph);
    ~GraphFacade();

    Graph* getGraph();


    void addNode(NodeHandlePtr node);

    ConnectionPtr connect(OutputPtr output, InputPtr input,
                          OutputTransition* ot, InputTransition* it);
    ConnectionPtr connect(Output* output, Input* input,
                          OutputTransition* ot, InputTransition* it);

    ConnectionPtr connect(NodeHandlePtr output, const std::string& output_name,
                          NodeHandlePtr input, const std::string& inputput_name);
    ConnectionPtr connect(NodeHandle* output, const std::string& output_name,
                          NodeHandle* input, const std::string& input_name);

    ConnectionPtr connect(const UUID& output_id, const UUID& input_id);

    ConnectionPtr connect(NodeHandlePtr output, int output_id,
                          NodeHandlePtr input, int input_id);
    ConnectionPtr connect(NodeHandle* output, int output_id,
                          NodeHandle* input, int input_id);

    TaskGenerator* getTaskGenerator(const UUID& uuid);

    void stop();
    void clearBlock();

    bool isPaused() const;
    void pauseRequest(bool pause);

    void reset();

public:
    csapex::slim_signal::Signal<void (bool)> paused;
    csapex::slim_signal::Signal<void ()> stopped;

    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeAdded;
    csapex::slim_signal::Signal<void(NodeHandlePtr)> nodeRemoved;

    csapex::slim_signal::Signal<void(NodeWorkerPtr)> nodeWorkerAdded;
    csapex::slim_signal::Signal<void(NodeWorkerPtr)> nodeWorkerRemoved;

    csapex::slim_signal::Signal<void(TaskGeneratorPtr)> generatorAdded;
    csapex::slim_signal::Signal<void(TaskGeneratorPtr)> generatorRemoved;

    csapex::slim_signal::Signal<void()> panic;

private:
    Graph* graph_;
    Executor& executor_;

    std::vector<csapex::slim_signal::Connection> connections_;
    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;

    std::map<NodeHandle*, NodeWorkerPtr> node_workers_;
};

}

#endif // GRAPH_WORKER_H
