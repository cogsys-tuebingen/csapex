#ifndef MOCKUP_NODES_H
#define MOCKUP_NODES_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>
#include <csapex/msg/io.h>
#include <csapex/msg/input.h>

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
        : i(0)
    {
    }

    void setup(NodeModifier& node_modifier) override
    {
        out = node_modifier.addOutput<int>("output");
    }

    void setupParameters(Parameterizable& /*parameters*/) override
    {

    }

    void process() override
    {
//        ainfo << "publish " << i << std::endl;
        msg::publish(out, i++);
    }

    int getValue() const
    {
        return i;
    }

private:
    Output* out;

    int i;
};

class MockupSink
{
public:
    MockupSink()
        : aborted(false),
//          waiting(false),
          value(-1)
    {

    }

    void setup(NodeModifier& node_modifier)
    {
        in = node_modifier.addInput<int>("input");
        in->setEssential(true);
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int i = msg::getValue<int>(in);
      //  std::cerr << " < sink " << i << std::endl;
        value = i;

//        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
//        waiting = false;
//        stepping_done.notify_all();
    }

    int getValue() const
    {
        return value;
    }

//    void setWaiting(bool w)
//    {
//        waiting = w;
//    }

//    void wait()
//    {
//        if(aborted) {
//            return;
//        }
//        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
//        while(waiting) {
//            stepping_done.wait(lock);
//        }
//    }

    void abort()
    {
        aborted = true;

//        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
//        if(waiting) {
//            waiting = false;
//            stepping_done.notify_all();
//        }
    }

private:
    Input* in;

    bool aborted;
//    bool waiting;

//    std::recursive_mutex wait_mutex;
//    std::condition_variable_any stepping_done;

    int value;
};

}

#endif // MOCKUP_NODES_H
