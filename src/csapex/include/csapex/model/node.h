#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/error_state.h>
#include <csapex/model/unique.h>
#include <csapex/model/parameterizable.h>
#include <csapex/utility/stream_relay.h>
#include <csapex/utility/assert.h>
#include <csapex/utility/timable.h>

namespace csapex {

// TODO: less inheritance!
class Node : public ErrorState, public Unique, public Parameterizable, public Timable
{
    // TODO: less friends!
    friend class NodeState;
    friend class NodeAdapter;
    friend class NodeWorker;
    friend class NodeConstructor;
    friend class NodeModifier;
    friend class NodeStatistics;
    friend class DefaultNodeAdapter;
    friend class NodeBox;
    friend class GraphIO;
    friend class Graph;

    friend class command::AddConnector;

public:
    typedef boost::shared_ptr<Node> Ptr;

public:
    Node(const UUID &uuid = UUID::NONE);
    virtual ~Node();

    void initialize(const std::string &type, const UUID &uuid,
                    NodeWorker *node_worker, Settings *settings);
    void setCommandDispatcher(CommandDispatcher* d);


    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;
    void setNodeState(NodeStatePtr memento);

    virtual MementoPtr getParameterState() const;
    virtual void setParameterState(Memento::Ptr memento);

    std::string getType() const;
    NodeWorker* getNodeWorker() const;

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

    virtual Input* getInput(const UUID& uuid) const;
    virtual Output* getOutput(const UUID& uuid) const;

    std::vector<Input*> getAllInputs() const;
    std::vector<Output*> getAllOutputs() const;

    std::vector<Input*> getMessageInputs() const;
    std::vector<Output*> getMessageOutputs() const;

    std::vector<Input*> getManagedInputs() const;
    std::vector<Output*> getManagedOutputs() const;

    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);


public:
    virtual void setup() = 0;
    virtual void setupParameters();
    virtual void messageArrived(Input* source);
    virtual void process() = 0;
    virtual void stateChanged();
    virtual void tick();

protected:
    void triggerModelChanged();

private:
    /*???*/ void errorEvent(bool error, const std::string &msg, ErrorLevel level);

protected:
    std::string type_;

    NodeModifier* modifier_;
    Settings* settings_;
    CommandDispatcher* dispatcher_;

    StreamRelay ainfo;
    StreamRelay awarn;
    StreamRelay aerr;
    StreamRelay alog;

private:
    NodeWorker* worker_;

    NodeStatePtr node_state_;
};

}

#endif // NODE_H_
