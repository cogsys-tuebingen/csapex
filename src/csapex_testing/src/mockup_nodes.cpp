/// HEADER
#include <csapex_testing/mockup_nodes.h>

/// PROJECT
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/token.h>
#include <csapex/param/parameter_factory.h>
#include <csapex/msg/generic_value_message.hpp>


using namespace csapex;

MockupDynamicMultiplierNode::MockupDynamicMultiplierNode()
{
}

void MockupDynamicMultiplierNode::setup(csapex::NodeModifier& node_modifier)
{
    input_a_ = node_modifier.addInput<int>("input_a");
    input_b_ = node_modifier.addInput<int>("input_b");
    output_ = node_modifier.addOutput<int>("output");
}

void MockupDynamicMultiplierNode::setupParameters(Parameterizable& /*parameters*/)
{

}

void MockupDynamicMultiplierNode::process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
{
    int a = msg::getValue<int>(input_a_);
    int b = msg::getValue<int>(input_b_);
    msg::publish(output_, a * b);
}


MockupSource::MockupSource()
{
}

void MockupSource::setup(NodeModifier& node_modifier)
{
    out = node_modifier.addOutput<int>("output");
}

void MockupSource::setupParameters(Parameterizable& parameters)
{
    parameters.addHiddenParameter(param::ParameterFactory::declareValue<int>("value", 0));
}

void MockupSource::process()
{
    int i = readParameter<int>("value");
    //        ainfo << "publish " << i << std::endl;
    msg::publish(out, i);
    setParameter("value", i+1);
}

int MockupSource::getValue() const
{
    return readParameter<int>("value");
}



MockupSink::MockupSink()
    : aborted(false),
      value(-1)
{

}

void MockupSink::setup(NodeModifier& node_modifier)
{
    in = node_modifier.addInput<int>("input");
    in->setEssential(true);
}

void MockupSink::setupParameters(Parameterizable& parameters)
{
    parameters.addHiddenParameter(param::ParameterFactory::declareValue<int>("value", -1));
}

void MockupSink::process(NodeModifier& node_modifier, Parameterizable& parameters)
{
    int i = msg::getValue<int>(in);
    setParameter("value", i);
}

int MockupSink::getValue() const
{
    return readParameter<int>("value");
}


void MockupSink::abort()
{
    aborted = true;
}




AnySink::AnySink()
{

}

void AnySink::setup(NodeModifier& node_modifier)
{
    in = node_modifier.addInput<connection_types::AnyMessage>("input");
}

void AnySink::process(NodeModifier& node_modifier, Parameterizable& parameters)
{
}

