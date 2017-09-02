#ifndef GRAPH_FACADE_LOCAL_H
#define GRAPH_FACADE_LOCAL_H

#include <csapex/model/graph_facade.h>

namespace csapex
{
class GraphFacadeLocal : public GraphFacade
{
public:
    GraphFacadeLocal(ThreadPool& executor, GraphLocalPtr graph, SubgraphNodePtr graph_node, NodeFacadePtr nh = nullptr, GraphFacadeLocal* parent = nullptr);

    virtual AUUID getAbsoluteUUID() const override;

    virtual GraphFacade* getSubGraph(const UUID& uuid) override;
    GraphFacadeLocalPtr getLocalSubGraph(const UUID& uuid);
    virtual GraphFacade* getParent() const override;

    virtual GraphPtr getGraph() const override;
    GraphLocalPtr getLocalGraph() const;

    GraphFacadeLocal* getLocalParent() const;

    SubgraphNodePtr getSubgraphNode();
    TaskGenerator* getTaskGenerator(const UUID& uuid);
    ThreadPool* getThreadPool();

    void addNode(NodeFacadePtr node);

    virtual void clear() override;
    virtual void stop() override;
    virtual void clearBlock() override;
    virtual void resetActivity() override;

    virtual bool isPaused() const override;
    virtual void pauseRequest(bool pause) override;

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


    virtual std::string makeStatusString() override;

protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) override;
    virtual void nodeRemovedHandler(graph::VertexPtr node) override;

    virtual void createSubgraphFacade(NodeFacadePtr nf) override;

private:
    AUUID absolute_uuid_;

    GraphFacadeLocal* parent_;
    ThreadPool& executor_;
    GraphLocalPtr graph_;
    SubgraphNodePtr graph_node_;

    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;

    std::unordered_map<UUID, GraphFacadeLocalPtr, UUID::Hasher> children_;

    std::unordered_map<UUID, NodeFacadePtr, UUID::Hasher> node_facades_;

};

}

#endif // GRAPH_FACADE_LOCAL_H
