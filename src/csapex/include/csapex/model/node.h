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
    virtual void clearBlock();
    virtual void stop();

    virtual void useTimer(Timer* timer);

protected:
    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

    void setUUID(const UUID& uuid);

public:
    virtual bool canBeDisabled() const;
    bool isEnabled();

public:
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;
    void setNodeState(NodeStatePtr memento);

    void setSettings(Settings* settings);

    virtual void setNodeWorker(NodeWorker* nw);
    NodeWorker* getNodeWorker() const;

    void doSetup();

    ConnectorIn* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    ConnectorOut* addOutput(ConnectionTypePtr type, const std::string& label);

    void addInput(ConnectorIn* in) __attribute__ ((deprecated));
    void addOutput(ConnectorOut* out) __attribute__ ((deprecated));

    void manageInput(ConnectorIn* in);
    void manageOutput(ConnectorOut* out);

    int countInputs() const;
    int countOutputs() const;
    int countManagedInputs() const;
    int countManagedOutputs() const;

    ConnectorIn* getInput(const unsigned int index) const;
    ConnectorOut* getOutput(const unsigned int index) const;

    ConnectorIn* getManagedInput(const unsigned int index) const;
    ConnectorOut* getManagedOutput(const unsigned int index) const;

    ConnectorIn* getParameterInput(const std::string& name) const;
    ConnectorOut* getParameterOutput(const std::string& name) const;

    virtual ConnectorIn* getInput(const UUID& uuid) const;
    virtual ConnectorOut* getOutput(const UUID& uuid) const;

    virtual Connectable* getConnector(const UUID& uuid) const;
    virtual std::vector<ConnectorIn*> getInputs() const;
    virtual std::vector<ConnectorOut*> getOutputs() const;

    void removeInput(ConnectorIn *in);
    void removeOutput(ConnectorOut *out);

    void setSynchronizedInputs(bool) __attribute__ ((deprecated))__attribute__ ((warning("not needed anymore, everything is synchronized now"))) {}

    int nextInputId();
    int nextOutputId();

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher* d);

    CommandPtr removeAllConnectionsCmd();

    bool canReceive();

public:
    virtual MementoPtr getChildState() const;

protected:
    virtual void setState(Memento::Ptr memento);

    Settings& getSettings();

    template <typename T>
    void updateParameter(param::Parameter*);
    void updateParameters();

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

    void registerInput(ConnectorIn* in);
    void registerOutput(ConnectorOut* out);

public Q_SLOTS:
    virtual void setup() = 0;
    virtual void setupParameters();
    virtual void messageArrived(ConnectorIn* source);
    virtual void process() = 0;

    virtual void tick();

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void updateModel();

    virtual void checkIO();

    void enableIO(bool enable);
    void enableInput(bool enable);
    void enableOutput(bool enable);

    void setIOError(bool error);
    void setMinimized(bool min);

    void triggerModelChanged();

Q_SIGNALS:
    void stateChanged();
    void modelChanged();
    void toggled(bool);
    void enabled(bool);
    void started();

    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone();
    void connectionStart();

    void connectorCreated(Connectable*);
    void connectorRemoved(Connectable*);

    void connectorEnabled(Connectable*);
    void connectorDisabled(Connectable*);    

    void nodeError(bool error, const std::string &msg, int level);

protected:
    std::string type_;

    NodeModifier* modifier_;

    StreamRelay ainfo;
    StreamRelay awarn;
    StreamRelay aerr;
    StreamRelay alog;

private:
    Settings* settings_;

    NodeWorker* worker_;

    NodeStatePtr node_state_;
    std::map<std::string, ConnectorIn*> param_2_input_;
    std::map<std::string, ConnectorOut*> param_2_output_;

    std::vector<ConnectorIn*> inputs_;
    std::vector<ConnectorOut*> outputs_;

    std::vector<ConnectorIn*> managed_inputs_;
    std::vector<ConnectorOut*> managed_outputs_;

    CommandDispatcher* dispatcher_;

    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;
};

}

#endif // NODE_H_
