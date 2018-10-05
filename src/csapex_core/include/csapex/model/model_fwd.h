#ifndef MODEL_FWD_H
#define MODEL_FWD_H

/// shared_ptr
#include <memory>

#define FWD(name)                                                                                                                                                                                      \
    class name;                                                                                                                                                                                        \
    typedef std::shared_ptr<name> name##Ptr;                                                                                                                                                           \
    typedef std::unique_ptr<name> name##UniquePtr;                                                                                                                                                     \
    typedef std::weak_ptr<name> name##WeakPtr;                                                                                                                                                         \
    typedef std::shared_ptr<const name> name##ConstPtr;

namespace csapex
{
namespace graph
{
FWD(Edge)
FWD(Vertex)
}  // namespace graph

FWD(Clonable)
FWD(Connectable)
FWD(ConnectableOwner)
FWD(Connection)
FWD(ConnectionDescription)
FWD(Connector)
FWD(ConnectorDescription)
FWD(Fulcrum)
FWD(GenericState)
FWD(Graph)
FWD(GraphFacade)
FWD(GraphFacadeImplementation)
FWD(GraphImplementation)
FWD(Node)
FWD(NodeCharacteristics)
FWD(NodeConstructor)
FWD(NodeFacade)
FWD(NodeFacadeImplementation)
FWD(NodeFacadeProxy)
FWD(NodeHandle)
FWD(NodeModifier)
FWD(NodeRunner)
FWD(NodeState)
FWD(NodeStatistics)
FWD(NodeWorker)
FWD(Parameterizable)
FWD(SubgraphNode)
FWD(Tag)
FWD(Token)
FWD(TokenData)

}  // namespace csapex

#undef FWD

#endif  // MODEL_FWD_H
