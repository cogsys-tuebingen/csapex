#ifndef NODE_WORKER_H
#define NODE_WORKER_H

/// COMPONENT
#include <csapex/csapex_fwd.h>

/// PROJECT
#include <utils_param/parameter.h>

/// SYSTEM
#include <QObject>
#include <QMutex>
#include <QTimer>
#include <map>
#include <deque>
#include <boost/function.hpp>
#include <QWaitCondition>

namespace csapex {

struct NodeWorker : public QObject, public param::Parameter::access
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

    void addParameter(param::Parameter* param);
    void addParameterCallback(param::Parameter *param, boost::function<void(param::Parameter *)> cb);
    void addParameterCondition(param::Parameter* param, boost::function<bool()> enable_condition);

    void checkConditions(bool silent);

    void pause(bool pause);

Q_SIGNALS:
    void messagesReceived();
    void messageProcessed();

private:
    void parameterChanged(param::Parameter* param);
    void parameterChanged(param::Parameter* param, boost::function<void(param::Parameter *)> cb);
    void parameterEnabled(param::Parameter* param, bool enabled);

private:
    static const double DEFAULT_FREQUENCY = 30.0;

private:
    Node* node_;
    QTimer* timer_;


    std::map<ConnectorIn*, bool> has_msg_;

    QMutex changed_params_mutex_;
    std::vector<std::pair<param::Parameter*, boost::function<void(param::Parameter *)> > > changed_params_;
    std::map<param::Parameter*, boost::function<bool()> > conditions_;

    std::deque<TimerPtr> timer_history_;

    bool thread_initialized_;
    bool paused_;
    QMutex pause_mutex_;
    QWaitCondition continue_;

    std::vector<boost::signals2::connection> connections_;
};

}

#endif // NODE_WORKER_H
