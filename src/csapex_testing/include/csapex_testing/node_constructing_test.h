#ifndef NODE_CONSTRUCTING_TEST_H
#define NODE_CONSTRUCTING_TEST_H

#include <csapex/model/model_fwd.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex_testing/test_exception_handler.h>

#include "gtest/gtest.h"

namespace csapex
{

class NodeConstructingTest : public ::testing::Test {
protected:
    NodeConstructingTest();

    virtual ~NodeConstructingTest();

    virtual void SetUp() override;

    virtual void TearDown() override;

    std::shared_ptr<NodeFactoryImplementation> node_factory;
    NodeFactoryImplementation& factory;

    TestExceptionHandler eh;

    ThreadPool executor;

    SubgraphNodePtr graph_node;
    GraphImplementationPtr graph;

    slim_signal::Connection abort_connection;
};
}

#endif // NODE_CONSTRUCTING_TEST_H
