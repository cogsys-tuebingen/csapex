#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <boost/signals2/signal.hpp>
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


    void addNode(NodeWorkerPtr node);

    ConnectionPtr connect(Output* output, Input* input,
                          OutputTransition* ot, InputTransition* it);
    ConnectionPtr connect(NodeHandle* output, int output_id,
                          NodeHandle* input, int input_id);

    TaskGenerator* getTaskGenerator(const UUID& uuid);

    void stop();
    void clearBlock();

    bool isPaused() const;
    void pauseRequest(bool pause);

    void reset();

public:
    boost::signals2::signal<void (bool)> paused;
    boost::signals2::signal<void ()> stopped;

    boost::signals2::signal<void(TaskGeneratorPtr)> generatorAdded;
    boost::signals2::signal<void(TaskGeneratorPtr)> generatorRemoved;

private:
    Graph* graph_;
    Executor& executor_;

    std::vector<boost::signals2::connection> connections_;
    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;
};

}

#endif // GRAPH_WORKER_H
