/// HEADER
#include <csapex_testing/stepping_test.h>

/// PROJECT
#include <csapex/model/node_facade_impl.h>
#include <csapex/model/graph/graph_impl.h>
#include <csapex/model/graph_facade_impl.h>

using namespace csapex;

SteppingTest::SteppingTest()
{
    step_called_only_once = true;
}

void SteppingTest::SetUp()
{
    NodeConstructingTest::SetUp();

    main_graph_facade = std::make_shared<GraphFacadeImplementation>(executor, graph, graph_node);
    graph->setNodeFacade(main_graph_facade->getLocalNodeFacade().get());

    executor.setSteppingMode(true);

    executor.setSuppressExceptions(false);

    end_step_called_more_than_once = false;
    end_step_called = false;
    executor.end_step.connect([&]() {
        {
            // TRACE std::cerr << ">> step done" << std::endl;
            std::unique_lock<std::recursive_mutex> lock(end_step_called_mutex);
            if (end_step_called) {
                end_step_called_more_than_once = true;
            }
            end_step_called = true;
        }

        step_done.notify_all();
    });
}

void SteppingTest::TearDown()
{
    executor.end_step.disconnectAll();

    NodeConstructingTest::TearDown();
}

void SteppingTest::step()
{
    // TRACEstd::cerr << "\n\nSTEP\n\n";
    ASSERT_TRUE(step_called_only_once);
    step_called_only_once = false;

    {
        std::unique_lock<std::recursive_mutex> lock(end_step_called_mutex);
        end_step_called = false;
    }
    executor.step();

    ASSERT_NO_FATAL_FAILURE(waitForEndOfStep());
}

void SteppingTest::waitForEndOfStep()
{
    std::unique_lock<std::recursive_mutex> lock(end_step_called_mutex);

    auto wait_start = std::chrono::system_clock::now();
    while (!end_step_called) {
        step_done.wait_for(lock, std::chrono::milliseconds(10));

        auto waited_for = std::chrono::system_clock::now() - wait_start;
        if (waited_for > std::chrono::milliseconds(10000)) {
            GTEST_FATAL_FAILURE_("waiting for end of step timed out");
        }
    }
    ASSERT_TRUE(end_step_called);
    ASSERT_FALSE(end_step_called_more_than_once);

    step_called_only_once = true;
}
