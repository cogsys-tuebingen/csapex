#ifndef MOCKUP_NODES_H
#define MOCKUP_NODES_H

/// PROJECT
#include <csapex/model/node.h>
#include <csapex/model/node_modifier.h>

namespace csapex
{

template <int factor>
class MockupNode
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

class MockupSource : public Node
{
public:
    MockupSource()
        : i(0)
    {
    }

    void setup(NodeModifier& node_modifier) override
    {
        out = node_modifier.addOutput<int>("generator");
    }

    void setupParameters(Parameterizable& /*parameters*/) override
    {

    }

    void process() override
    {
        msg::publish(out, i++);
    }

private:
    Output* out;

    int i;
};

class MockupSink
{
public:
    MockupSink()
        : waiting(false), value(-1)
    {

    }

    void setup(NodeModifier& node_modifier)
    {
        in = node_modifier.addInput<int>("sink");
    }

    void setupParameters(Parameterizable& /*parameters*/)
    {

    }

    void process(NodeModifier& node_modifier, Parameterizable& /*parameters*/)
    {
        int i = msg::getValue<int>(in);
        value = i;

        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
        waiting = false;
        stepping_done.notify_all();
    }

    int getValue() const
    {
        return value;
    }

    void setWaiting(bool w)
    {
        waiting = w;
    }

    void wait()
    {
        std::unique_lock<std::recursive_mutex> lock(wait_mutex);
        while(waiting) {
            stepping_done.wait(lock);
        }
    }

private:
    Input* in;

    bool waiting;

    std::recursive_mutex wait_mutex;
    std::condition_variable_any stepping_done;

    int value;
};

}

#endif // MOCKUP_NODES_H
