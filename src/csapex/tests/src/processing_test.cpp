#include <csapex/model/graph.h>
#include <csapex/model/node.h>
#include <csapex/model/node_facade_impl.h>
#include <csapex/factory/node_factory_impl.h>
#include <csapex/factory/node_wrapper.hpp>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/generic_value_message.hpp>
#include <csapex/core/settings/settings_impl.h>
#include <csapex/msg/io.h>
#include <csapex/model/node_handle.h>
#include <csapex/model/node_worker.h>
#include <csapex/msg/direct_connection.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/core/exception_handler.h>
#include <csapex/scheduling/thread_pool.h>
#include <csapex/model/graph_facade_impl.h>
#include <csapex/model/node.h>
#include <csapex/utility/uuid_provider.h>
#include <csapex/model/subgraph_node.h>
#include <csapex/model/graph/graph_impl.h>

#include "gtest/gtest.h"
#include "mockup_nodes.h"
#include "test_exception_handler.h"

#include <mutex>
#include <condition_variable>

#include "stepping_test.h"

namespace csapex
{
class ProcessingTest : public NodeConstructingTest
{

};

TEST_F(ProcessingTest, DirectProcessing)
{
    NodeFacadeImplementationPtr times_4 = factory.makeNode("StaticMultiplier4", UUIDProvider::makeUUID_without_parent("StaticMultiplier4"), graph);
    Node& node = *times_4->getNode();
    NodeHandle& nh = *times_4->getNodeHandle();

    // no inputs are sent yet -> expect an assertion failure
    ASSERT_THROW(node.process(nh, node), csapex::HardAssertionFailure);

    // set the input message
    TokenPtr token_in = msg::createToken(23);
    InputPtr input = nh.getInput(UUIDProvider::makeUUID_without_parent("StaticMultiplier4:|:in_0"));
    ASSERT_NE(nullptr, input);
    input->setToken(token_in);


    // no inputs are sent now -> node should multiply the input by 4
    node.process(nh, node);

    // commit the messages produced by the node
    OutputPtr output = nh.getOutput(UUIDProvider::makeUUID_without_parent("StaticMultiplier4:|:out_0"));
    ASSERT_NE(nullptr, output);
    output->commitMessages(false);

    // view outputs
    TokenPtr token_out = output->getToken();
    TokenDataConstPtr data_out = token_out->getTokenData();
    ASSERT_NE(nullptr, data_out);

    auto msg_out = std::dynamic_pointer_cast<connection_types::GenericValueMessage<int> const>(data_out);
    ASSERT_NE(nullptr, msg_out);

    ASSERT_EQ(23 * 4, msg_out->value);
}
}

