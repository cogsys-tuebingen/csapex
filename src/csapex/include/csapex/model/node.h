#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/msg/msg_fwd.h>
#include <csapex/signal/signal_fwd.h>
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

    UUID getUUID() const;
    virtual void initialize(csapex::NodeHandle* node_handle, const UUID &uuid);

public: /* API */
    virtual void setup(csapex::NodeModifier& node_modifier) = 0;

    virtual void setupParameters(Parameterizable& parameters);

    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters,
                         std::function<void(std::function<void (csapex::NodeModifier&, Parameterizable &)>)> continuation);
    virtual void process(csapex::NodeModifier& node_modifier, csapex::Parameterizable& parameters);
    virtual void process();

    virtual void processMarker(const connection_types::MessageConstPtr &marker);

    virtual void activation();
    virtual void deactivation();

    virtual void reset();

    virtual void tearDown();

    virtual bool isAsynchronous() const;
    virtual bool isIsolated() const;

    virtual void stateChanged();

    virtual void getProperties(std::vector<std::string>& properties) const;

public:
    StreamRelay adebug;
    StreamRelay ainfo;
    StreamRelay awarn;
    StreamRelay aerr;

protected:
    UUID uuid_;
    csapex::NodeModifier* node_modifier_;
    csapex::Parameterizable* parameters_;

    csapex::NodeHandle* node_handle_;

    long guard_;
};

}

#endif // NODE_H_
