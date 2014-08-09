#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>

/// SYSTEM
#include <QObject>
#include <QTimer>
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

public:
    NodeWorker(Node* node);
    ~NodeWorker();

    void stop();

    /*??*/ void makeThread();

    bool isEnabled() const;
    void setEnabled(bool e);

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

    Input* addInput(ConnectionTypePtr type, const std::string& label, bool optional, bool async);
    Output* addOutput(ConnectionTypePtr type, const std::string& label);

    /* NAMING */ void manageInput(Input* in);
    /* NAMING */ void manageOutput(Output* out);

    /* NAMING */ void registerInput(Input* in);
    /* NAMING */ void registerOutput(Output* out);

    bool canReceive();

public Q_SLOTS:
    void forwardMessage(Connectable* source);

    void forwardMessageSynchronized(Input* source);

    void clearInput(Input* source);

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

    void sendMessages();

Q_SIGNALS:
    void messagesReceived();
    void messageProcessed();

    void enabled(bool);

    void connectionInProgress(Connectable*, Connectable*);
    void connectionDone();
    void connectionStart();

    void connectorCreated(Connectable*);
    void connectorRemoved(Connectable*);

    void connectorEnabled(Connectable*);
    void connectorDisabled(Connectable*);

    void nodeStateChanged();
    void nodeModelChanged();

private:
    static const double DEFAULT_FREQUENCY = 30.0;

    void removeInput(Input *in);
    void removeOutput(Output *out);

    void connectConnector(Connectable* c);
    void disconnectConnector(Connectable* c);

private:
    Node* node_;

    std::vector<Input*> inputs_;
    std::vector<Output*> outputs_;

    std::vector<Input*> managed_inputs_;
    std::vector<Output*> managed_outputs_;

    QTimer* tick_timer_;

    QThread* private_thread_;

    std::map<Input*, bool> has_msg_;

    int timer_history_pos_;
    std::vector<TimerPtr> timer_history_;

    bool thread_initialized_;
    bool paused_;
    bool stop_;
    QMutex stop_mutex_;
    QMutex pause_mutex_;
    QMutex message_mutex_;
    QWaitCondition continue_;
};

}

#endif // NODE_WORKER_H
