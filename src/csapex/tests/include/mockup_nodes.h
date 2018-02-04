#ifndef MOCKUP_NODES_H
#define MOCKUP_NODES_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input.h>
#include <csapex/msg/output.h>
#include <csapex/model/token.h>
#include <csapex/param/parameter_factory.h>

namespace csapex
{

template <int factor>
class MockupStaticMultiplierNode
{
public:
    void setup(NodeModifier& node_modifier)
    {
        in = node_modifier.addInput<int>("input");
        out = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int val = msg::getValue<int>(in);
        val *= factor;


        msg::publish(out, val);
    }

private:
    Input* in;
    Output* out;
};


class MockupDynamicMultiplierNode
{
public:
    MockupDynamicMultiplierNode()
    {
    }

    void setup(csapex::NodeModifier& node_modifier)
    {
        input_a_ = node_modifier.addInput<int>("input_a");
        input_b_ = node_modifier.addInput<int>("input_b");
        output_ = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int a = msg::getValue<int>(input_a_);
        int b = msg::getValue<int>(input_b_);
        msg::publish(output_, a * b);
    }

private:
    Input* input_a_;
    Input* input_b_;
    Output* output_;
};


class MockupSource : public Node
{
public:
    MockupSource()
    {
    }

    void setup(NodeModifier& node_modifier) override
    {
        out = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addHiddenParameter(param::ParameterFactory::declareValue<int>("value", 0));
    }

    void process() override
    {
        int i = readParameter<int>("value");
//        ainfo << "publish " << i << std::endl;
        msg::publish(out, i);
        setParameter("value", i+1);
    }

    int getValue() const
    {
        return readParameter<int>("value");
    }

private:
    Output* out;
};

class MockupSink : public Node
{
public:
    MockupSink()
        : aborted(false),
          value(-1)
    {

    }

    void setup(NodeModifier& node_modifier) override
    {
        in = node_modifier.addInput<int>("input");
        in->setEssential(true);
    }

    void setupParameters(Parameterizable& parameters) override
    {
        parameters.addHiddenParameter(param::ParameterFactory::declareValue<int>("value", -1));
    }

    void process(NodeModifier& node_modifier, Parameterizable& parameters) override
    {
        int i = msg::getValue<int>(in);
        setParameter("value", i);
    }

    int getValue() const
    {
        return readParameter<int>("value");
    }


    void abort()
    {
        aborted = true;
    }

private:
    Input* in;

    bool aborted;
    int value;
};

}

#endif // MOCKUP_NODES_H
