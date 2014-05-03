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
#include <deque>
#include <boost/function.hpp>
#include <QWaitCondition>

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

public Q_SLOTS:
    void forwardMessage(Connectable* source);

    void forwardMessageSynchronized(ConnectorIn* source);

    void addInput(ConnectorIn* source);
    void removeInput(ConnectorIn* source);

    void eventGuiChanged();

    void checkParameters();

    void setTickFrequency(double f);
    void tick();

    void triggerError(bool e, const std::string& what);


    void pause(bool pause);

Q_SIGNALS:
    void messagesReceived();
    void messageProcessed();

private:
    static const double DEFAULT_FREQUENCY = 30.0;

private:
    Node* node_;
    QTimer* timer_;

    std::map<ConnectorIn*, bool> has_msg_;

    std::deque<TimerPtr> timer_history_;

    bool thread_initialized_;
    bool paused_;
    QMutex pause_mutex_;
    QWaitCondition continue_;
};

}

#endif // NODE_WORKER_H
