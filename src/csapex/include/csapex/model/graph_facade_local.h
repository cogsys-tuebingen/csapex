#ifndef GRAPH_FACADE_LOCAL_H
#define GRAPH_FACADE_LOCAL_H

#include <csapex/model/graph_facade.h>

namespace csapex
{
class GraphFacadeLocal : public GraphFacade
{
public:
    GraphFacadeLocal(ThreadPool& executor, GraphPtr graph, SubgraphNodePtr graph_node, NodeFacadePtr nh = nullptr, GraphFacadeLocal* parent = nullptr);

    virtual GraphFacade* getParent() const override;
    GraphFacadeLocal* getLocalParent() const;

    TaskGenerator* getTaskGenerator(const UUID& uuid);

    void addNode(NodeFacadePtr node);

    virtual void clear() override;
    virtual void clearBlock() override;
    virtual void resetActivity() override;

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


protected:
    virtual void nodeAddedHandler(graph::VertexPtr node) override;
    virtual void nodeRemovedHandler(graph::VertexPtr node) override;

    virtual void createSubgraphFacade(NodeFacadePtr nf) override;

private:
    GraphFacadeLocal* parent_;

    std::unordered_map<UUID, TaskGeneratorPtr, UUID::Hasher> generators_;

};

}

#endif // GRAPH_FACADE_LOCAL_H
