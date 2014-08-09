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

public:
    typedef boost::shared_ptr<NodeWorker> Ptr;

public:
    NodeWorker(Node* node);
    ~NodeWorker();

    void stop();

    void makeThread();

    bool isEnabled() const;
    void setEnabled(bool e);

    void setIOError(bool error);

    /* REMOVE => UI*/ void setMinimized(bool min);

public Q_SLOTS:
    void forwardMessage(Connectable* source);

    void forwardMessageSynchronized(Input* source);

    void addInput(Input* source);
    void checkInputs();
    void clearInput(Input* source);
    void removeInput(Input* source);

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

private:
    static const double DEFAULT_FREQUENCY = 30.0;

private:
    Node* node_;
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
