#ifndef NODE_CONSTRUCTING_TEST_H
#define NODE_CONSTRUCTING_TEST_H

#include <csapex/model/model_fwd.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex_testing/test_exception_handler.h>
#include <csapex_testing/csapex_test_case.h>

namespace csapex
{
class NodeConstructingTest : public CsApexTestCase
{
protected:
    NodeConstructingTest();

    virtual ~NodeConstructingTest();

    void SetUp() override;

    void TearDown() override;

    std::shared_ptr<NodeFactoryImplementation> node_factory;
    NodeFactoryImplementation& factory;

    TestExceptionHandler eh;

    ThreadPool executor;

    SubgraphNodePtr graph_node;
    GraphImplementationPtr graph;

    slim_signal::Connection abort_connection;
};
}  // namespace csapex

#endif  // NODE_CONSTRUCTING_TEST_H
