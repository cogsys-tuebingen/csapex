#ifndef NODE_WRAPPER_HPP
#define NODE_WRAPPER_HPP

/// PROJECT
#include <csapex/model/node.h>

namespace csapex
{

template <class Impl>
class NodeWrapper : public Node, public Impl
{
    virtual void setup(csapex::NodeModifier& node_modifier) override
    {
        Impl::setup(node_modifier);
    }

    virtual void setupParameters(Parameterizable& parameters) override
    {
        Impl::setupParameters(parameters);
    }

    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters) override
    {
        Impl::process(node_modifier, parameters);
    }
};

}

#endif // NODE_WRAPPER_HPP
