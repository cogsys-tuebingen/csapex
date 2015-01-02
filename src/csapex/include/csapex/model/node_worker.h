#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>
#include <csapex/model/unique.h>
#include <csapex/utility/uuid.h>

/// SYSTEM
#include <QObject>
#include <QTimer>
#include <QThread>
#include <QMutex>
#include <map>
#include <boost/function.hpp>
#include <QWaitCondition>
#include <vector>

namespace csapex {

struct NodeWorker : public QObject, public Unique
{
    Q_OBJECT

    friend class ProfilingWidget;
    friend class Node;

public:
    enum ActivityType {
        TICK,
        PROCESS
    };

public:
    typedef boost::shared_ptr<NodeWorker> Ptr;

    static const double DEFAULT_FREQUENCY = 30.0;

public:
    NodeWorker(const std::string& type, const UUID& uuid, Settings& settings, NodePtr node);
    ~NodeWorker();

    void setNodeState(NodeStatePtr memento);
    NodeStatePtr getNodeState();
    NodeStatePtr getNodeStateCopy() const;


    void stop();
    void waitUntilFinished();
    void reset();

    void triggerSwitchThreadRequest(QThread* thread, int id);
    void triggerPanic();

    Node* getNode() const;

    std::string getType() const;

    bool isEnabled() const;
    void setEnabled(bool e);

    bool isPaused() const;

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);
    Slot* addSlot(const std::string& label, boost::function<void ()> callback);
    Trigger* addTrigger(const std::string& label);

    Connectable* getConnector(const UUID& uuid) const;
    Input* getInput(const UUID& uuid) const;
    Output* getOutput(const UUID& uuid) const;
    Slot* getSlot(const UUID& uuid) const;
    Trigger* getTrigger(const UUID& uuid) const;

    /* experimental */ void makeParameterConnectable(param::Parameter*);
    /* experimental */ Input* getParameterInput(const std::string& name) const;
    /* experimental */ Output* getParameterOutput(const std::string& name) const;

    /* NAMING */ void registerInput(Input* in);
    /* NAMING */ void registerOutput(Output* out);
    /* NAMING */ void registerSlot(Slot* s);
    /* NAMING */ void registerTrigger(Trigger* t);

    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);
    void removeSlot(const UUID& uuid);
    void removeTrigger(const UUID& uuid);

    std::vector<Connectable*> getAllConnectors() const;
    std::vector<Input*> getAllInputs() const;
    std::vector<Output*> getAllOutputs() const;

    std::vector<Input*> getMessageInputs() const;
    std::vector<Output*> getMessageOutputs() const;
    std::vector<Slot*> getSlots() const;
    std::vector<Trigger*> getTriggers() const;

    std::vector<Input*> getParameterInputs() const;
    std::vector<Output*> getParameterOutputs() const;

    bool canReceive();
    bool areAllInputsAvailable();

    void makeParametersConnectable();

    bool isTickEnabled() const;
    void setTickEnabled(bool enabled);

    bool isSource() const;
    void setIsSource(bool source);

    bool isSink() const;
    void setIsSink(bool sink);

public Q_SLOTS:
    void messageArrived(Connectable* source);
    void processMessages();

    void parameterMessageArrived(Connectable* source);


    void checkParameters();    
    void checkIO();

    void enableIO(bool enable);
    void enableInput(bool enable);
    void enableOutput(bool enable);

    void setTickFrequency(double f);
    void tick();

    void triggerError(bool e, const std::string& what);

    void pause(bool pause);
    void killExecution();

    bool canSendMessages();
    void trySendMessages();
    void resetInputs();

Q_SIGNALS:
    void messageProcessed();
    void ticked();

    void enabled(bool);
    void messagesWaitingToBeSent(bool);

    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone(Connectable*);
    void connectionStart(Connectable*);

    void connectorCreated(Connectable*);
    void connectorRemoved(Connectable*);

    void connectorEnabled(Connectable*);
    void connectorDisabled(Connectable*);

    void nodeStateChanged();
    void nodeModelChanged();
    void threadChanged();

    void threadSwitchRequested(QThread*, int);
    void tickRequested();

    void timerStarted(NodeWorker* worker, int type, long stamp);
    void timerStopped(NodeWorker* worker, long stamp);

    void panic();

private Q_SLOTS:
    void switchThread(QThread* thread, int id);
    void checkIfOutputIsReady(Connectable*);
    void checkIfInputsCanBeProcessed();

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);
    void removeSlot(Slot *out);
    void removeTrigger(Trigger *out);

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

    template <typename T>
    void makeParameterConnectableImpl(param::Parameter*);
    void publishParameters();
    void publishParameter(param::Parameter *p);

    void processInputs(bool all_inputs_are_present);

    void assertNotInGuiThread();

    void triggerNodeStateChanged();

    void finishTimer(TimerPtr t);

private:
    Settings& settings_;

    std::string node_type_;
    NodePtr node_;    
    NodeStatePtr node_state_;


    bool is_setup_;

    std::vector<Input*> inputs_;
    std::vector<Output*> outputs_;
    std::vector<Trigger*> triggers_;
    std::vector<Slot*> slots_;

    std::vector<Input*> parameter_inputs_;
    std::vector<Output*> parameter_outputs_;

    std::map<std::string, Input*> param_2_input_;
    std::map<std::string, Output*> param_2_output_;

    std::map<Input*,param::Parameter*> input_2_param_;
    std::map<Output*,param::Parameter*> output_2_param_;

    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;

    QTimer* tick_timer_;
    bool tick_enabled_;
    bool tick_immediate_;
    int ticks_;

    bool source_;
    bool sink_;

    QMutex sync;
    bool messages_waiting_to_be_sent;

    int timer_history_pos_;
    std::vector<TimerPtr> timer_history_;

    bool thread_initialized_;
    bool paused_;
    bool stop_;
    QMutex stop_mutex_;
    QMutex pause_mutex_;
    QWaitCondition continue_;
};

}

#endif // NODE_WORKER_H
