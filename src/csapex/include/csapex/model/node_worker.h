#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>
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

struct NodeWorker : public QObject
{
    Q_OBJECT

    friend class ProfilingWidget;
    friend class Node;

public:
    typedef boost::shared_ptr<NodeWorker> Ptr;

    static const double DEFAULT_FREQUENCY = 30.0;

public:
    NodeWorker(Settings& settings, NodePtr node);
    ~NodeWorker();

    void stop();
    void waitUntilFinished();
    void reset();

    void triggerSwitchThreadRequest(QThread* thread, int id);
    void triggerPanic();

    Node* getNode();
    UUID getNodeUUID() const;

    bool isEnabled() const;
    void setEnabled(bool e);

    bool isPaused() const;

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

    virtual Input* getInput(const UUID& uuid) const;
    virtual Output* getOutput(const UUID& uuid) const;

    /* experimental */ Input* getParameterInput(const std::string& name) const;
    /* experimental */ Output* getParameterOutput(const std::string& name) const;

    /* NAMING */ void manageInput(Input* in);
    /* NAMING */ void manageOutput(Output* out);

    /* NAMING */ void registerInput(Input* in);
    /* NAMING */ void registerOutput(Output* out);

    void removeInput(const UUID& uuid);
    void removeOutput(const UUID& uuid);

    std::vector<Input*> getAllInputs() const;
    std::vector<Output*> getAllOutputs() const;

    std::vector<Input*> getMessageInputs() const;
    std::vector<Output*> getMessageOutputs() const;

    std::vector<Input*> getManagedInputs() const;
    std::vector<Output*> getManagedOutputs() const;

    bool canReceive();
    bool areAllInputsAvailable();

    void makeParametersConnectable();

public Q_SLOTS:
    void messageArrived(Connectable* source);
    void processMessages();

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

    void panic();

private Q_SLOTS:
    void switchThread(QThread* thread, int id);
    void checkIfOutputIsReady(Connectable*);
    void checkIfInputsCanBeProcessed();

private:
    void removeInput(Input *in);
    void removeOutput(Output *out);

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

    template <typename T>
    void makeParameterConnectable(param::Parameter*);


    void processInputs(bool all_inputs_are_present);

private:
    Settings& settings_;
    NodePtr node_;

    std::vector<Input*> inputs_;
    std::vector<Output*> outputs_;

    std::vector<Input*> managed_inputs_;
    std::vector<Output*> managed_outputs_;

    std::map<std::string, Input*> param_2_input_;
    std::map<std::string, Output*> param_2_output_;

    std::vector<boost::signals2::connection> connections;
    std::vector<QObject*> callbacks;

    QTimer* tick_timer_;
    bool tick_immediate_;
    int ticks_;

    QMutex sync;
    std::map<Input*, bool> has_msg_;
    bool messages_waiting_to_be_sent;
    std::map<Output*, bool> is_ready_;

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
