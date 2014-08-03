#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/error_state.h>
#include <csapex/model/unique.h>
#include <csapex/model/parameterizable.h>
#include <csapex/utility/timable.h>
#include <csapex/utility/stream_relay.h>
#include <csapex/utility/assert.h>

/// PROJECT
#include <utils_param/param_fwd.h>

/// SYSTEM
#include <QObject>
#include <boost/utility.hpp>

namespace csapex {

class Node : public QObject, public ErrorState, public Unique, public Parameterizable, public Timable
{
    Q_OBJECT

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

    void setType(const std::string& type);
    std::string getType() const;

    virtual void pause(bool pause);
    /* ?? */ virtual void clearBlock();
    virtual void stop();

    /* ?? */ virtual void useTimer(Timer* timer);

    virtual void setEnabled(bool e);
    bool isEnabled();

    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;
    void setNodeState(NodeStatePtr memento);

    virtual MementoPtr getChildState() const;

    void setSettings(Settings* settings);

    virtual void setNodeWorker(NodeWorker* nw);
    NodeWorker* getNodeWorker() const;

    void doSetup();

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

    void addInput(Input* in) __attribute__ ((deprecated));
    void addOutput(Output* out) __attribute__ ((deprecated));

    void manageInput(Input* in);
    void manageOutput(Output* out);

    int countInputs() const;
    int countOutputs() const;
    int countManagedInputs() const;
    int countManagedOutputs() const;

    Input* getInput(const unsigned int index) const;
    Output* getOutput(const unsigned int index) const;

    Input* getManagedInput(const unsigned int index) const;
    Output* getManagedOutput(const unsigned int index) const;

    Input* getParameterInput(const std::string& name) const;
    Output* getParameterOutput(const std::string& name) const;

    virtual Connectable* getConnector(const UUID& uuid) const;
    virtual Input* getInput(const UUID& uuid) const;
    virtual Output* getOutput(const UUID& uuid) const;

    virtual std::vector<Input*> getInputs() const;
    virtual std::vector<Output*> getOutputs() const;

    void removeInput(Input *in);
    void removeOutput(Output *out);

    void setCommandDispatcher(CommandDispatcher* d);

    /* ?? */ bool canReceive();


public:
    virtual void setup() = 0;
    virtual void setupParameters();
    virtual void messageArrived(Input* source);
    virtual void process() = 0;

    virtual void tick();

    /*ALL TO BE REMOVED / MADE NON SLOT*/
public Q_SLOTS:
    virtual void connectorChanged();

    virtual void updateModel();

    virtual void checkIO();

    void enableIO(bool enable);
    void enableInput(bool enable);
    void enableOutput(bool enable);

    void setIOError(bool error);
    void setMinimized(bool min);


/*ALL TO BE REMOVED*/
Q_SIGNALS:
    void stateChanged();
    void modelChanged();
    void enabled(bool);

    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone();
    void connectionStart();

    void connectorCreated(Connectable*);
    void connectorRemoved(Connectable*);

    void connectorEnabled(Connectable*);
    void connectorDisabled(Connectable*);    

    void nodeError(bool error, const std::string &msg, int level);


protected:
    /* ?? */ void connectConnector(Connectable* c);
    /* ?? */ void disconnectConnector(Connectable* c);

    void setUUID(const UUID& uuid);

    virtual void setState(Memento::Ptr memento);

    template <typename T>
    /* ?? */ void updateParameter(param::Parameter*);
    /* ?? */ void updateParameters();

    void triggerModelChanged();

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

    /* ?? */ void registerInput(Input* in);
    /* ?? */ void registerOutput(Output* out);

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

    std::vector<Input*> inputs_;
    std::vector<Output*> outputs_;

    std::vector<Input*> managed_inputs_;
    std::vector<Output*> managed_outputs_;

    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;
};

}

#endif // NODE_H_
