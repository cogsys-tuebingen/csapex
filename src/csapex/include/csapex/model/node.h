#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/tag.h>
#include <csapex/model/memento.h>
#include <csapex/model/error_state.h>

/// SYSTEM
#include <QObject>
#include <QIcon>

namespace csapex {

class Node : public QObject, public ErrorState
{
    Q_OBJECT

    friend class NodeState;
    friend class Box;
    friend class GraphIO;
    friend class Graph;
    friend class BoxedObjectConstructor;

    friend class command::AddConnector;

public:
    typedef boost::shared_ptr<Node> Ptr;

public:
    Node(const std::string& uuid);
    virtual ~Node();
    virtual void setup();

    void setType(const std::string& type);
    std::string getType();

    void setCategory(const std::string& category) __attribute__ ((deprecated));

    void addTag(const Tag& tag);
    std::vector<Tag> getTags() const;

    void setIcon(QIcon icon);
    QIcon getIcon();

    void stop();

private:
    void setUUID(const std::string& uuid);

    void connectConnector(Connector* c);
    void disconnectConnector(Connector* c);

public:
    std::string UUID() const;

    virtual bool canBeDisabled() const;
    bool isEnabled();

private:
    void setNodeState(NodeStatePtr memento);

public:
    void setNodeStateLater(NodeStatePtr state);
    NodeStatePtr getNodeState();

    /// TODO: get rid of this
    virtual void setBox(Box* box);
    Box* getBox() const;
    NodeWorker* getNodeWorker() const;

    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false) {
        return addInput(T::make(), label, optional);
    }

    template <typename T>
    ConnectorOut* addOutput(const std::string& label) {
        return addOutput(T::make(), label);
    }

    ConnectorIn* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    ConnectorOut* addOutput(ConnectionTypePtr type, const std::string& label);

    void addInput(ConnectorIn* in) __attribute__ ((deprecated));
    void addOutput(ConnectorOut* out) __attribute__ ((deprecated));

    int countInputs();
    int countOutputs();

    ConnectorIn* getInput(const unsigned int index);
    ConnectorOut *getOutput(const unsigned int index);

    ConnectorIn* getInput(const std::string& uuid);
    ConnectorOut* getOutput(const std::string& uuid);

    void removeInput(ConnectorIn *in);
    void removeOutput(ConnectorOut *out);

    void setSynchronizedInputs(bool sync);

    int nextInputId();
    int nextOutputId();

    QPoint getPosition() const;
    void setPosition(const QPoint& pos);

    CommandDispatcher* getCommandDispatcher() const;
    void setCommandDispatcher(CommandDispatcher* d);

    CommandPtr removeAllConnectionsCmd();

    ///
    /// IO
    ///

    YAML::Emitter& save(YAML::Emitter& out) const;
    void read(const YAML::Node &doc);

protected:
    virtual void setState(Memento::Ptr memento);
    virtual Memento::Ptr getState() const;

    void makeThread();

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

    void registerInput(ConnectorIn* in);
    void registerOutput(ConnectorOut* out);

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);
    virtual void allConnectorsArrived();

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void tick();

    virtual void updateModel();
    void eventGuiChanged();

    void messageProcessed();

    void killContent();

Q_SIGNALS:
    void stateChanged();
    void modelChanged();
    void toggled(bool);
    void started();

    void connectionInProgress(Connector*, Connector*);
    void connectionDone();
    void connectionStart();

    void connectorCreated(Connector*);
    void connectorRemoved(Connector*);

    void connectorEnabled(Connector*);
    void connectorDisabled(Connector*);

protected:
    std::string type_;
    mutable std::vector<Tag> tags_;
    QIcon icon_;

private:
    Box* box_;

    QThread* private_thread_;
    QMutex worker_mutex_;

    NodeWorker* worker_;
    std::string uuid_;

    NodeStatePtr state;

    std::vector<ConnectorIn*> input;

    std::vector<ConnectorOut*> output;

    CommandDispatcher* dispatcher_;

    bool loaded_state_available_;
};

}

#endif // NODE_H_
