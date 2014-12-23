#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/error_state.h>
#include <csapex/model/parameterizable.h>
#include <csapex/utility/stream_relay.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/timable.h>

namespace csapex {

class Node : public ErrorState, public Parameterizable, public Timable
{
public:
    typedef boost::shared_ptr<Node> Ptr;

public:
    Node();
    virtual ~Node();

    void initialize(const std::string &type, const UUID &uuid,
                    NodeWorker *node_worker, Settings *settings);
    void doSetup();

    NodeWorker* getNodeWorker() const;


public:
    virtual void setup() = 0;
    virtual void setupParameters();
    virtual void messageArrived(Input* source);
    virtual void process() = 0;
    virtual void stateChanged();
    virtual bool canTick();
    virtual void tick();
    virtual void abort();

protected:
    /* CAN THIS BIS REMOVED???? */
    void triggerModelChanged();

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);


public:
    StreamRelay adebug;
    StreamRelay ainfo;
    StreamRelay awarn;
    StreamRelay aerr;

protected:
    NodeModifier* modifier_;
    Settings* settings_;

private:
    NodeWorker* worker_;
};

}

#endif // NODE_H_
