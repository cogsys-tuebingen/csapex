#ifndef STEPPING_TEST_H
#define STEPPING_TEST_H

/// BASE
#include <csapex_testing/node_constructing_test.h>

/// USED BY ALL CLIENTS
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/subgraph_node.h>
#include <csapex_testing/io.h>

namespace csapex
{

class SteppingTest : public NodeConstructingTest
{
protected:
    using stamp_t = std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::milliseconds>;

public:
    SteppingTest();

    void SetUp() override;
    void TearDown() override;

protected:
    void step();
private:
    void waitForEndOfStep();

protected:
    bool end_step_called_more_than_once;
    bool end_step_called;
    bool step_called_only_once;

    std::recursive_mutex end_step_called_mutex;
    std::condition_variable_any step_done;

    GraphFacadeImplementationPtr main_graph_facade;
};

}

#endif // STEPPING_TEST_H
