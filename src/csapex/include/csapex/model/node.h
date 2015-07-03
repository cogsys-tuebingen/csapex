#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/parameterizable.h>
#include <csapex/utility/stream_relay.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/timable.h>

namespace csapex {

class Node : public Parameterizable, public Timable
{
public:
    typedef std::shared_ptr<Node> Ptr;

public:
    Node();
    virtual ~Node();

    void initialize(const UUID &uuid, NodeModifier *node_modifier);
    void doSetup();

    virtual void process(csapex::Parameterizable& parameters);
    virtual void process(csapex::Parameterizable& parameters, std::function<void(std::function<void ()>)> continuation);

public:
    virtual void setup(csapex::NodeModifier& node_modifier) = 0;
    virtual void setupParameters(Parameterizable& parameters);

    virtual bool isAsynchronous() const;

    virtual bool canTick();
    virtual void tick();

    virtual void abort();

    virtual void stateChanged();

protected:
    virtual void process(); /*deprecated*/

public:
    StreamRelay adebug;
    StreamRelay ainfo;
    StreamRelay awarn;
    StreamRelay aerr;

protected:
    NodeModifier* modifier_;
};

}

#endif // NODE_H_
