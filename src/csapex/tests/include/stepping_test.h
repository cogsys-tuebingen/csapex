#ifndef STEPPING_TEST_H
#define STEPPING_TEST_H

#include "node_constructing_test.h"
#include <csapex/model/node_facade_impl.h>

namespace csapex
{

class SteppingTest : public NodeConstructingTest
{
protected:
    using stamp_t = std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::milliseconds>;

public:
    SteppingTest()
    {
        step_called_only_once = true;
    }

    void SetUp()
    {
        NodeConstructingTest::SetUp();

        main_graph_facade = std::make_shared<GraphFacadeImplementation>(executor, graph, graph_node);
        graph->setNodeFacade(main_graph_facade->getLocalNodeFacade());

        executor.setSteppingMode(true);

        executor.setSuppressExceptions(false);

        end_step_called_more_than_once = false;
        end_step_called = false;
        executor.end_step.connect([&]() {
            {
                //TRACE std::cerr << ">> step done" << std::endl;
                std::unique_lock<std::recursive_mutex> lock(end_step_called_mutex);
                if(end_step_called) {
                    end_step_called_more_than_once = true;
                }
                end_step_called = true;
            }

            step_done.notify_all();
        });
    }

    void TearDown()
    {
        executor.end_step.disconnectAll();

        NodeConstructingTest::TearDown();
    }

protected:
    void step()
    {
        //TRACEstd::cerr << "\n\nSTEP\n\n";
        ASSERT_TRUE(step_called_only_once);
        step_called_only_once = false;

        {
            std::unique_lock<std::recursive_mutex> lock(end_step_called_mutex);
            end_step_called = false;
        }
        executor.step();

        ASSERT_NO_FATAL_FAILURE(waitForEndOfStep());
    }

private:
    void waitForEndOfStep()
    {
        std::unique_lock<std::recursive_mutex> lock(end_step_called_mutex);

        auto wait_start = std::chrono::system_clock::now();
        while(!end_step_called) {
            step_done.wait_for(lock, std::chrono::milliseconds(10));

            auto waited_for = std::chrono::system_clock::now() - wait_start;
            if(waited_for > std::chrono::milliseconds(10000)) {
                GTEST_FATAL_FAILURE_("waiting for end of step timed out");
            }
        }
        ASSERT_TRUE(end_step_called);
        ASSERT_FALSE(end_step_called_more_than_once);

        step_called_only_once = true;
    }

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
