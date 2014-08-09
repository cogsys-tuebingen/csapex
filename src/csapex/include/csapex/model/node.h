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

/// PROJECT
#include <utils_param/param_fwd.h>

/// SYSTEM
#include <boost/utility.hpp>

namespace csapex {

class Node : public ErrorState, public Unique, public Parameterizable, public Timable
{
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

    std::string getType() const;

    /*poor naming*/ virtual void stop();

    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;
    void setNodeState(NodeStatePtr memento);

    /*poor naming*/ virtual MementoPtr getChildState() const;


    NodeWorker* getNodeWorker() const;

    /*poor naming*/ void initialize(const std::string &type, const UUID &uuid,
                                    NodeWorker *node_worker, Settings *settings);

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

    virtual Connectable* getConnector(const UUID& uuid) const;
    virtual Input* getInput(const UUID& uuid) const;
    virtual Output* getOutput(const UUID& uuid) const;

    /* experimental */ Input* getParameterInput(const std::string& name) const;
    /* experimental */ Output* getParameterOutput(const std::string& name) const;

    std::vector<Input*> getAllInputs() const;
    std::vector<Output*> getAllOutputs() const;

    std::vector<Input*> getMessageInputs() const;
    std::vector<Output*> getMessageOutputs() const;

    std::vector<Input*> getManagedInputs() const;
    std::vector<Output*> getManagedOutputs() const;

    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);

    void setCommandDispatcher(CommandDispatcher* d);

public:
    virtual void setup() = 0;
    virtual void setupParameters();
    virtual void messageArrived(Input* source);
    virtual void process() = 0;
    virtual void stateChanged();

    virtual void tick();

protected:
    virtual void setState(Memento::Ptr memento);

    template <typename T>
    /* ?? */ void updateParameter(param::Parameter*);
    /* ?? */ void updateParameters();

    void triggerModelChanged();

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

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
    std::map<std::string, Input*> param_2_input_;
    std::map<std::string, Output*> param_2_output_;

    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;
};

}

#endif // NODE_H_
