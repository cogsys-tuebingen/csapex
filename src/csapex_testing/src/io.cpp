/// HEADER
#include <csapex_testing/io.h>

/// PROJECT
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/io.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/msg/output.h>

using namespace csapex;

namespace csapex
{
namespace testing
{
NodeFacadeImplementationPtr getNodeFacade(const GraphFacadeImplementation& main_graph_facade, const UUID& id)
{
    return std::dynamic_pointer_cast<NodeFacadeImplementation>(main_graph_facade.findNodeFacade(id));
}
NodeHandlePtr getNodeHandle(const NodeFacadePtr& node)
{
    auto nf = std::dynamic_pointer_cast<NodeFacadeImplementation>(node);
    apex_assert_hard(nf);
    return nf->getNodeHandle();
}

InputPtr getInput(const NodeFacadePtr& node, const std::string& name)
{
    return getNodeHandle(node)->getInput(UUIDProvider::makeDerivedUUID_forced(node->getUUID(), name));
}

OutputPtr getOutput(const NodeFacadePtr& node, const std::string& name)
{
    return getNodeHandle(node)->getOutput(UUIDProvider::makeDerivedUUID_forced(node->getUUID(), name));
}

TokenDataConstPtr getAddedMessage(const OutputPtr& output)
{
    return model::getTokenData(output->getAddedToken());
}

}
}
