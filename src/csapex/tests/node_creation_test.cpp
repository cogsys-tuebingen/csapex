#include <csapex/model/node.h>
#include <csapex/model/node_factory.h>
#include <csapex/msg/message.h>
#include <csapex/core/settings.h>
#include "gtest/gtest.h"


namespace csapex {

class MockupNode : public Node
{
public:
    MockupNode()
    {
    }

    void setup()
    {
    }

    void process()
    {
    }
};


/*
 * Mockup Message
 */

struct MockupMessagePayload
{
    float value;
};

struct MockupMessage : public connection_types::Message
{
    typedef boost::shared_ptr<MockupMessage> Ptr;

    MockupMessage()
        : Message("MockupMessage", "/")
    {
    }

    ConnectionType::Ptr clone() {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    ConnectionType::Ptr toType() {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    static ConnectionType::Ptr make(){
        return Ptr(new MockupMessage);
    }


    void writeYaml(YAML::Emitter &) const
    {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    void readYaml(const YAML::Node &)
    {
        throw std::runtime_error ("MOCKUP: not implemented");
    }

    MockupMessagePayload value;
};

/// TRAITS
namespace connection_types {
template <>
struct type<MockupMessage> {
    static std::string name() {
        return "Mockup";
    }
};
template <>
struct type<const MockupMessage> {
    static std::string name() {
        return "Mockup";
    }
};
}

}

#include <boost/mpl/assert.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/equal.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/function_types/parameter_types.hpp>
#include <boost/function_types/components.hpp>
#include <boost/function_types/function_pointer.hpp>
#include <boost/function_types/function_reference.hpp>
#include <boost/type_traits.hpp>

#include <csapex/model/node_modifier.h>
#include <utils_param/parameter_factory.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>

namespace csapex {

template <typename Message>
struct CompositeInput
{
    typedef Message type;
};
template <typename Message>
struct CompositeOutput
{
    typedef Message type;
};
template <typename T>
struct CompositeParameter
{
    typedef T type;
};

template <typename Parameters>
class CompositeNode : public Node
{
    typedef typename boost::mpl::push_front<Parameters, void>::type Sig;
    typedef typename boost::function_types::function_pointer<Sig>::type Callback;

    template <typename T>
    friend class CompositeNodeSetup;
    template <typename T>
    friend class CompositeNodeParameterSetup;

public:
    CompositeNode(Callback cb)
        : cb_(cb)
    {

    }

    void setup();
    void setupParameters();
    void process();

private:
    Callback cb_;
    std::vector<Input*> input_;
    std::vector<Output*> output_;
    std::vector<std::string> params_;
};

template <typename Parameters>
struct CompositeNodeSetup {
    CompositeNodeSetup(CompositeNode<Parameters>* instance)
        : instance_(instance)
    {

    }

    template<typename U>
    void operator()(CompositeInput<U>) {
        std::string label = connection_types::type<U>::name();
        instance_->input_.push_back(instance_->modifier_->template addInput<U>(label));
    }
    template<typename U>
    void operator()(CompositeOutput<U>) {
        std::string label = connection_types::type<U>::name();
        instance_->output_.push_back(instance_->modifier_->template addOutput<U>(label));

    }
    template<typename U>
    void operator()(CompositeParameter<U>) {}

    CompositeNode<Parameters>* instance_;
};

template <typename Parameters>
struct CompositeNodeParameterSetup {
    CompositeNodeParameterSetup(CompositeNode<Parameters>* instance)
        : instance_(instance)
    {}

    template<typename U>
    void operator()(CompositeInput<U>) {}
    template<typename U>
    void operator()(CompositeOutput<U>) {}

    template<typename U>
    void operator()(CompositeParameter<U>) {
        std::string name = "param";
        param::Parameter::Ptr p = param::ParameterFactory::declareValue<U>(name, 0);
        instance_->addParameter(p);
        instance_->params_.push_back(name);
    }

    CompositeNode<Parameters>* instance_;
};

struct wrap_parameters
{
    template <typename I>
    struct apply
    {
        typedef typename boost::remove_reference<I>::type RawType;

        typedef typename boost::mpl::if_< boost::is_reference<I>,
        typename boost::mpl::if_< boost::is_const<RawType>, CompositeInput<RawType>, CompositeOutput<RawType> >::type,
        CompositeParameter<RawType> >::type type;
    };
};


template <typename Parameters>
void CompositeNode<Parameters>::setup()
{
    boost::mpl::for_each<Parameters, wrap_parameters>(CompositeNodeSetup<Parameters>(this));
}

template <typename Parameters>
void CompositeNode<Parameters>::setupParameters()
{
    boost::mpl::for_each<Parameters, wrap_parameters>(CompositeNodeParameterSetup<Parameters>(this));
}

template <int count>
struct Call
{
};

template <typename Parameters>
void CompositeNode<Parameters>::process()
{
    // finish!
    MockupMessage::Ptr in = input_[0]->template getMessage<MockupMessage>();
    MockupMessage::Ptr in2 = input_[1]->template getMessage<MockupMessage>();

    MockupMessage::Ptr out(new MockupMessage);

    int param = readParameter<int>(params_[0]);

    //boost::bind(cb_, boost::ref(*in), boost::ref(*in2), param, boost::ref(*out))();
    Call <boost::mpl::size<Parameters>::value > caller;

    output_[0]->publish(out);
}



template<typename F>
Node::Ptr wrapFunction(F f)
{
    typedef typename boost::function_types::parameter_types<F>::type params;

    Node::Ptr result(new CompositeNode<params>(f));
    return result;
}

void functionToBeWrappedIntoANode(const MockupMessage& input, const MockupMessage& input2, int parameter, MockupMessage& output)
{
    output.value.value = input.value.value + input2.value.value + parameter;
}


class NodeCreationTest : public ::testing::Test {
protected:
    Settings settings;
    NodeFactory factory;

    NodeCreationTest()
        : factory(settings)
    {
        settings.set("headless", true);

        std::vector<TagPtr> tags;
        csapex::NodeConstructor::Ptr mockup_constructor(new csapex::NodeConstructor(
                                                            settings,
                                                            "MockupNode", "A mockup node",
                                                            ":/no_icon.png", tags,
                                                            boost::bind(&NodeCreationTest::makeMockup)));
        factory.register_box_type(mockup_constructor);


        csapex::NodeConstructor::Ptr wrapped_constructor(new csapex::NodeConstructor(
                                                             settings,
                                                             "WrappedFunctionNode", "functionToBeWrappedIntoANode",
                                                             ":/no_icon.png", tags,
                                                             boost::bind(&NodeCreationTest::makeWrappedFunctionNode)));
        factory.register_box_type(wrapped_constructor);
    }

    virtual ~NodeCreationTest() {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:

    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    static NodePtr makeMockup() {
        return NodePtr(new MockupNode);
    }

    static NodePtr makeWrappedFunctionNode() {
        //        return NodePtr(new WrapFunction<const MockupMessage, MockupMessage, functionToBeWrappedIntoANode>);
        //        return NodePtr(new WrappedFunction(&functionToBeWrappedIntoANode));
        return NodePtr(wrapFunction(functionToBeWrappedIntoANode));
    }

};

TEST_F(NodeCreationTest, NodeCanBeMadeInAFactory) {
    UUID node_id = UUID::make_forced("foobarbaz");
    NodePtr node = factory.makeNode("MockupNode", node_id);

    ASSERT_TRUE(node != NULL);
    EXPECT_EQ(node_id, node->getUUID());
}

TEST_F(NodeCreationTest, GenericNodeCanBeMadeInAFactory) {
    UUID node_id = UUID::make_forced("foobarbaz");
    NodePtr node = factory.makeNode("WrappedFunctionNode", node_id);

    ASSERT_TRUE(node != NULL);

    std::vector<Input*> inputs = node->getMessageInputs();
    ASSERT_EQ(inputs.size(), 2);

    std::vector<Output*> outputs = node->getMessageOutputs();
    ASSERT_EQ(outputs.size(), 1);

    std::vector<param::Parameter::Ptr> params = node->getParameters();
    ASSERT_EQ(params.size(), 1);

    MockupMessage::Ptr msg(new MockupMessage);
    msg->setSequenceNumber(0);
    msg->value.value = 23;

    MockupMessage::Ptr msg2(new MockupMessage);
    msg2->setSequenceNumber(0);
    msg2->value.value = 17;

    param::Parameter::Ptr param = params[0];
    param->set(2);

    Input* in = inputs[0];
    in->inputMessage(msg);

    Input* in2 = inputs[1];
    in2->inputMessage(msg2);

    // TODO: make this possible:
    /// currently there is a worker spawned automatically!
    //node->process();

    Output* out = outputs[0];
    ConnectionType::Ptr out_msg = out->getMessage();

    ASSERT_TRUE(out_msg != NULL);

    MockupMessage::Ptr result = boost::dynamic_pointer_cast<MockupMessage>(out_msg);

    ASSERT_TRUE(result != NULL);

    ASSERT_EQ(result->value.value, 42);
}

}
