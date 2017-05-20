#ifndef GRAPH_WORKER_H
#define GRAPH_WORKER_H

/// PROJECT
#include <csapex/model/model_fwd.h>
#include <csapex/msg/msg_fwd.h>
#include <csapex/scheduling/scheduling_fwd.h>
#include <csapex/utility/uuid.h>
#include <csapex/csapex_export.h>
#include <csapex/utility/notifier.h>
#include <csapex/utility/notification.h>
#include <csapex/utility/slim_signal.hpp>
#include <csapex/model/observer.h>

/// SYSTEM
#include <unordered_map>

namespace csapex
{

class CSAPEX_EXPORT GraphFacade : public Observer, public Notifier
{
public:
    typedef std::shared_ptr<GraphFacade> Ptr;

public:
    GraphFacade(ThreadPool& executor, GraphPtr graph, SubgraphNodePtr graph_node, NodeFacadePtr nh = nullptr, GraphFacade* parent = nullptr);
    ~GraphFacade();

    AUUID getAbsoluteUUID() const;
    GraphPtr getGraph();
    SubgraphNodePtr getSubgraphNode();
    NodeHandle* getNodeHandle();
    NodeFacadePtr getNodeFacade();
    GraphFacade* getParent() const;
    GraphFacade* getSubGraph(const UUID& uuid);
    ThreadPool* getThreadPool();

    void addNode(NodeFacadePtr node);

    ConnectionPtr connect(OutputPtr output, InputPtr input);

    ConnectionPtr connect(const UUID& output_id, const UUID& input_id);

    ConnectionPtr connect(NodeHandle* output, const std::string& output_name, NodeHandle* input, const std::string& input_name);
    ConnectionPtr connect(NodeHandlePtr output, const std::string& output_name, NodeHandlePtr input, const std::string& input_name);
    ConnectionPtr connect(NodeHandle* output, const std::string& output_name, const UUID& input_id);
    ConnectionPtr connect(NodeHandlePtr output, const std::string& output_name, const UUID& input_id);
    ConnectionPtr connect(const UUID& output_id, NodeHandle* input, const std::string& input_name);
    ConnectionPtr connect(const UUID& output_id, NodeHandlePtr input, const std::string& input_name);
    ConnectionPtr connect(const UUID& output_id, NodeHandlePtr input, int input_id);
    ConnectionPtr connect(NodeHandlePtr output, int output_id, const UUID& input_id);
    ConnectionPtr connect(NodeHandle* output, int output_id, NodeHandle* input, int input_id);
    ConnectionPtr connect(NodeHandlePtr output, int output_id, NodeHandlePtr input, int input_id);

    ConnectionPtr connect(NodeFacade* output, const std::string& output_name, NodeFacade* input, const std::string& input_name);
    ConnectionPtr connect(NodeFacadePtr output, const std::string& output_name, NodeFacadePtr input, const std::string& input_name);
    ConnectionPtr connect(NodeFacade* output, const std::string& output_name, const UUID& input_id);
    ConnectionPtr connect(NodeFacadePtr output, const std::string& output_name, const UUID& input_id);
    ConnectionPtr connect(const UUID& output_id, NodeFacade* input, const std::string& input_name);
    ConnectionPtr connect(const UUID& output_id, NodeFacadePtr input, const std::string& input_name);
    ConnectionPtr connect(const UUID& output_id, NodeFacadePtr input, int input_id);
    ConnectionPtr connect(NodeFacadePtr output, int output_id, const UUID& input_id);
    ConnectionPtr connect(NodeFacade* output, int output_id, NodeFacade* input, int input_id);
    ConnectionPtr connect(NodeFacadePtr output, int output_id, NodeFacadePtr input, int input_id);

    TaskGenerator* getTaskGenerator(const UUID& uuid);

    void stop();
    void clearBlock();
    void resetActivity();

    bool isPaused() const;
    void pauseRequest(bool pause);

    void clear();

public:
    slim_signal::Signal<void (bool)> paused;
    slim_signal::Signal<void ()> stopped;

    slim_signal::Signal<void(GraphFacadePtr)> child_added;
    slim_signal::Signal<void(GraphFacadePtr)> child_removed;

    slim_signal::Signal<void(NodeFacadePtr)> node_facade_added;
    slim_signal::Signal<void(NodeFacadePtr)> node_facade_removed;

    slim_signal::Signal<void(TaskGeneratorPtr)> generator_added;
    slim_signal::Signal<void(TaskGeneratorPtr)> generator_removed;

    slim_signal::Signal<void()> panic;
    
private:
    void nodeAddedHandler(graph::VertexPtr node);
    void nodeRemovedHandler(graph::VertexPtr node);

    void createSubgraphFacade(NodeFacadePtr nh);

private:
    GraphFacade* parent_;

    AUUID absolute_uuid_;
    GraphPtr graph_;
    SubgraphNodePtr graph_node_;
    NodeFacadePtr graph_handle_;
    ThreadPool& executor_;

    std::unordered_map<UUID, GraphFacadePtr, UUID::Hasher> children_;

    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;

    std::unordered_map<UUID, NodeFacadePtr, UUID::Hasher> node_facades_;
};

}

#endif // GRAPH_WORKER_H
