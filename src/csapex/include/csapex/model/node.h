#ifndef NODE_H_
#define NODE_H_

/// COMPONENT
#include <csapex/csapex_fwd.h>
#include <csapex/model/tag.h>
#include <csapex/model/memento.h>
#include <csapex/model/error_state.h>
#include <csapex/model/generic_state.h>
#include <csapex/model/unique.h>
#include <csapex/model/message.h>
#include <csapex/utility/timable.h>

/// PROJECT
#include <utils_param/parameter.h>

/// SYSTEM
#include <QObject>
#include <QIcon>
#include <QTreeWidgetItem>
#include <boost/utility.hpp>

namespace csapex {

/// FORWARD
template <typename T>
class RosMessageConversionT;

class Node : public QObject, public ErrorState, public Unique, public Timable, public param::Parameter::access
{
    Q_OBJECT

    friend class NodeState;
    friend class NodeAdapter;
    friend class Box;
    friend class GraphIO;
    friend class Graph;
    friend class NodeConstructor;

    friend class command::AddConnector;

public:
    typedef boost::shared_ptr<Node> Ptr;

public:
    Node(const UUID &uuid = UUID::NONE);
    virtual ~Node();

    void setType(const std::string& type);
    std::string getType() const;

    void setCategory(const std::string& category) __attribute__ ((deprecated));

    void addTag(const Tag& tag);
    std::vector<Tag> getTags() const;

    void addParameter(const param::Parameter::Ptr& param);
    void addParameter(const param::Parameter::Ptr& param, boost::function<void(param::Parameter *)> cb);
    void addConditionalParameter(const param::Parameter::Ptr& param, boost::function<bool()> enable_condition);
    void addConditionalParameter(const param::Parameter::Ptr& param, boost::function<bool()> enable_condition, boost::function<void(param::Parameter *)> cb);

    std::vector<param::Parameter::Ptr> getParameters() const;

    template <typename T>
    const T param(const std::string& name) const
    {
        try {
            return getParameter(name)->as<T>();
        } catch(const std::out_of_range& e) {
            throw std::runtime_error(std::string("unknown parameter '") + name + "'");
        }
    }
    template <typename T>
    typename T::Ptr getParameter(const std::string& name) const
    {
        return boost::dynamic_pointer_cast<T> (getParameter(name));
    }

    param::Parameter::Ptr getParameter(const std::string& name) const;

    bool isParameterEnabled(const std::string& name) const;
    void setParameterEnabled(const std::string& name, bool enabled);

    ConnectorIn* getParameterInput(const std::string& name) const;
    ConnectorOut* getParameterOutput(const std::string& name) const;

    void setIcon(QIcon icon);
    QIcon getIcon() const;

    virtual void stop();

    std::string getLabel() const;

    virtual void useTimer(Timer* timer);

protected:
    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

public:
    virtual bool canBeDisabled() const;
    bool isEnabled();

private:
    void setNodeState(NodeStatePtr memento);

public:
    void setNodeStateLater(NodeStatePtr state);
    NodeStatePtr getNodeState();

    void setSettings(Settings* settings);

    NodeWorker* getNodeWorker() const;

    void doSetup();

    /// "real" messages
    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addInput(T::make(), label, optional, async);
    }
    template <typename Container, typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addInput(Container::template make<T>(), label, optional, async);
    }

    /// "direct" messages
    template <typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addInput(connection_types::GenericMessage<T>::make(), label, optional, async);
    }
    template <typename Container, typename T>
    ConnectorIn* addInput(const std::string& label, bool optional = false, bool async = false,
                          typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addInput(Container::template make<T>(), label, optional, async);
    }

    /// "real" messages
    template <typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addOutput(T::make(), label);
    }
    template <typename Container, typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::enable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        return addOutput(Container::template make<T>(), label);
    }

    /// "direct" messages
    template <typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addOutput(connection_types::GenericMessage<T>::make(), label);
    }
    template <typename Container, typename T>
    ConnectorOut* addOutput(const std::string& label,
                            typename boost::disable_if<boost::is_base_of<ConnectionType, T> >::type* dummy = 0) {
        RosMessageConversionT<T>::registerConversion();
        return addOutput(Container::template make<T>(), label);
    }

    ConnectorIn* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    ConnectorOut* addOutput(ConnectionTypePtr type, const std::string& label);

    void addInput(ConnectorIn* in) __attribute__ ((deprecated));
    void addOutput(ConnectorOut* out) __attribute__ ((deprecated));

    void manageInput(ConnectorIn* in);
    void manageOutput(ConnectorOut* out);

    int countInputs() const;
    int countOutputs() const;

    ConnectorIn* getInput(const unsigned int index) const;
    ConnectorOut* getOutput(const unsigned int index) const;

    virtual ConnectorIn* getInput(const UUID& uuid) const;
    virtual ConnectorOut* getOutput(const UUID& uuid) const;

    virtual Connectable* getConnector(const UUID& uuid) const;
    virtual std::vector<ConnectorIn*> getInputs() const;
    virtual std::vector<ConnectorOut*> getOutputs() const;

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

    QTreeWidgetItem* createDebugInformation() const;

    ///
    /// IO
    ///

    YAML::Emitter& save(YAML::Emitter& out) const;
    void read(const YAML::Node &doc);

    bool canReceive();

public:
    virtual Memento::Ptr getState() const;
protected:
    virtual void setState(Memento::Ptr memento);

    void makeThread();
    void finishProcessing();

    Settings& getSettings();

    template <typename T>
    void updateParameter(param::Parameter*);
    void updateParameters();

private:
    void errorEvent(bool error, const std::string &msg, ErrorLevel level);

    void registerInput(ConnectorIn* in);
    void registerOutput(ConnectorOut* out);

    QTreeWidgetItem * createDebugInformationConnector(Connectable *connector) const;

protected:
    virtual void allConnectorsArrived() __attribute__ ((deprecated));

public Q_SLOTS:
    virtual void messageArrived(ConnectorIn* source);
    virtual void process() = 0;
    virtual void setup() = 0;

    virtual void enable(bool e);
    virtual void enable();
    virtual void disable(bool e);
    virtual void disable();
    virtual void connectorChanged();

    virtual void tick();

    virtual void updateModel();
    void eventGuiChanged();

    virtual void checkIfDone();
    virtual void checkInputs();

    void killContent();

    void enableIO(bool enable);
    void enableInput(bool enable);
    void enableOutput(bool enable);

    void setIOError(bool error);
    void setLabel(const std::string& label);
    void setMinimized(bool min);

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
    mutable std::vector<Tag> tags_;
    QIcon icon_;


private:
    Settings* settings_;

    QThread* private_thread_;
    QMutex worker_mutex_;

    NodeWorker* worker_;

    NodeStatePtr node_state_;
    std::map<std::string, ConnectorIn*> param_2_input_;
    std::map<std::string, ConnectorOut*> param_2_output_;

    std::vector<ConnectorIn*> inputs_;
    std::vector<ConnectorOut*> outputs_;

    std::vector<ConnectorIn*> managed_inputs_;
    std::vector<ConnectorOut*> managed_outputs_;

    CommandDispatcher* dispatcher_;

    bool loaded_state_available_;

    GenericState state;
    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;
};

}

#endif // NODE_H_
